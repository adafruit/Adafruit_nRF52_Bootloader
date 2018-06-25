
#include "uf2.h"
#include "nrf_nvmc.h"
#include <string.h>
#include "tusb.h"

#include "bootloader_settings.h"
#include "bootloader.h"
#include "pstorage.h"


typedef struct {
    uint8_t JumpInstruction[3];
    uint8_t OEMInfo[8];
    uint16_t SectorSize;
    uint8_t SectorsPerCluster;
    uint16_t ReservedSectors;
    uint8_t FATCopies;
    uint16_t RootDirectoryEntries;
    uint16_t TotalSectors16;
    uint8_t MediaDescriptor;
    uint16_t SectorsPerFAT;
    uint16_t SectorsPerTrack;
    uint16_t Heads;
    uint32_t HiddenSectors;
    uint32_t TotalSectors32;
    uint8_t PhysicalDriveNum;
    uint8_t Reserved;
    uint8_t ExtendedBootSig;
    uint32_t VolumeSerialNumber;
    uint8_t VolumeLabel[11];
    uint8_t FilesystemIdentifier[8];
} __attribute__((packed)) FAT_BootBlock;

typedef struct {
    char name[8];
    char ext[3];
    uint8_t attrs;
    uint8_t reserved;
    uint8_t createTimeFine;
    uint16_t createTime;
    uint16_t createDate;
    uint16_t lastAccessDate;
    uint16_t highStartCluster;
    uint16_t updateTime;
    uint16_t updateDate;
    uint16_t startCluster;
    uint32_t size;
} __attribute__((packed)) DirEntry;

STATIC_ASSERT(sizeof(DirEntry) == 32);

struct TextFile {
    const char name[11];
    const char *content;
};

#define NUM_FAT_BLOCKS UF2_NUM_BLOCKS

#define STR0(x) #x
#define STR(x) STR0(x)
const char infoUf2File[] = //
    "UF2 Bootloader " UF2_VERSION "\r\n"
    "Model: " PRODUCT_NAME "\r\n"
    "Board-ID: " BOARD_ID "\r\n"
    "Bootloader: " BOOTLOADER_ID "\r\n";

const char indexFile[] = //
    "<!doctype html>\n"
    "<html>"
    "<body>"
    "<script>\n"
    "location.replace(\"" INDEX_URL "\");\n"
    "</script>"
    "</body>"
    "</html>\n";

static const struct TextFile info[] = {
    {.name = "INFO_UF2TXT", .content = infoUf2File},
    {.name = "INDEX   HTM", .content = indexFile},
    {.name = "CURRENT UF2"},
};
#define NUM_INFO (sizeof(info) / sizeof(info[0]))

#define UF2_SIZE           (get_flash_size() * 2)
#define UF2_SECTORS        (UF2_SIZE / 512)
#define UF2_FIRST_SECTOR   (NUM_INFO + 1)
#define UF2_LAST_SECTOR    (UF2_FIRST_SECTOR + UF2_SECTORS - 1)

#define RESERVED_SECTORS   1
#define ROOT_DIR_SECTORS   4
#define SECTORS_PER_FAT    ((NUM_FAT_BLOCKS * 2 + 511) / 512)

#define START_FAT0         RESERVED_SECTORS
#define START_FAT1         (START_FAT0 + SECTORS_PER_FAT)
#define START_ROOTDIR      (START_FAT1 + SECTORS_PER_FAT)
#define START_CLUSTERS     (START_ROOTDIR + ROOT_DIR_SECTORS)

static const FAT_BootBlock BootBlock = {
    .JumpInstruction      = {0xeb, 0x3c, 0x90},
    .OEMInfo              = "UF2 UF2 ",
    .SectorSize           = 512,
    .SectorsPerCluster    = 1,
    .ReservedSectors      = RESERVED_SECTORS,
    .FATCopies            = 2,
    .RootDirectoryEntries = (ROOT_DIR_SECTORS * 512 / 32),
    .TotalSectors16       = NUM_FAT_BLOCKS - 2,
    .MediaDescriptor      = 0xF8,
    .SectorsPerFAT        = SECTORS_PER_FAT,
    .SectorsPerTrack      = 1,
    .Heads                = 1,
    .ExtendedBootSig      = 0x29,
    .VolumeSerialNumber   = 0x00420042,
    .VolumeLabel          = VOLUME_LABEL,
    .FilesystemIdentifier = "FAT16   ",
};

#define NO_CACHE 0xffffffff

#define NRF_LOG_DEBUG(...)
#define NRF_LOG_WARNING(...)

uint32_t flashAddr = NO_CACHE;
uint8_t flashBuf[FLASH_PAGE_SIZE] __attribute__((aligned(4)));
bool firstFlush = true;
//bool hadWrite = false;

static WriteState uf2_wr_state = { 0 };
static WriteState* state = &uf2_wr_state;

volatile bool _is_flashing = false;

static pstorage_handle_t _fat_psh = { .module_id = 0, .block_id = USER_FLASH_END } ;


static uint32_t get_flash_size(void)
{
  static uint32_t flash_sz = 0;

  // only need to compute once
  if ( flash_sz == 0 )
  {
    // return 1 block of 256 bytes
    if ( !bootloader_app_is_valid(DFU_BANK_0_REGION_START) )
    {
      flash_sz = 256;
    }else
    {
      const bootloader_settings_t * boot_setting;
      bootloader_util_settings_get(&boot_setting);

      flash_sz = boot_setting->bank_0_size;
    }
  }

  return flash_sz;
}

void fat_pstorage_cb(pstorage_handle_t * p_handle, uint8_t op_code, uint32_t result, uint8_t  * p_data, uint32_t  data_len)
{
  if ( result != NRF_SUCCESS )
  {
    TU_ASSERT(false, );
  }

  if ( PSTORAGE_CLEAR_OP_CODE == op_code)
  {
    // erase complete start writing
    _fat_psh.block_id = p_handle->block_id;
    TU_ASSERT( pstorage_store(&_fat_psh, flashBuf, FLASH_PAGE_SIZE, 0), );
  }
  else if ( PSTORAGE_STORE_OP_CODE ==  op_code)
  {
    // write completes
    _is_flashing = false;

    // whole uf2 file is written, inform bootloader to update setting and reset
    if ( state->numWritten >= state->numBlocks )
    {
      dfu_update_status_t update_status;

      memset(&update_status, 0, sizeof(dfu_update_status_t ));
      update_status.status_code = DFU_UPDATE_APP_COMPLETE;
      update_status.app_crc     = 0; // skip CRC checking with uf2 upgrade
      update_status.app_size    = state->numBlocks*256;

      bootloader_dfu_update_process(update_status);
    }
  }
}

void ghostfat_init(void)
{
  pstorage_module_param_t  fat_psp = { .cb = fat_pstorage_cb};
  pstorage_register(&fat_psp, &_fat_psh);
}


void flushFlash() {
    if (flashAddr == NO_CACHE)
        return;

    if (firstFlush) {
        firstFlush = false;
#if 0
        if (sdRunning) {
            // disable SD - we need sync access to flash, and we might be also overwriting the SD
            nrf_sdh_disable_request();
            nrf_dfu_settings_init(false);
        }


        s_dfu_settings.write_offset = 0;
        s_dfu_settings.sd_size = 0;
        s_dfu_settings.bank_layout = NRF_DFU_BANK_LAYOUT_DUAL;
        s_dfu_settings.bank_current = NRF_DFU_CURRENT_BANK_0;

        memset(&s_dfu_settings.bank_0, 0, sizeof(s_dfu_settings.bank_0));
        memset(&s_dfu_settings.bank_1, 0, sizeof(s_dfu_settings.bank_1));

        nrf_dfu_settings_write(NULL);
#endif
    }

//    int32_t sz = flashAddr + FLASH_PAGE_SIZE;
//    if (s_dfu_settings.bank_0.image_size < sz)
//        s_dfu_settings.bank_0.image_size = sz;

    NRF_LOG_DEBUG("Flush at %x", flashAddr);
    if (memcmp(flashBuf, (void *)flashAddr, FLASH_PAGE_SIZE) != 0) {
        NRF_LOG_DEBUG("Write flush at %x", flashAddr);
        _is_flashing = true;

        // Writing to flash will be done in erase complete callback
        _fat_psh.block_id = flashAddr;
        TU_ASSERT ( pstorage_clear(&_fat_psh, FLASH_PAGE_SIZE), );
    }

    flashAddr = NO_CACHE;
}



void flash_write(uint32_t dst, const uint8_t *src, int len) {
    uint32_t newAddr = dst & ~(FLASH_PAGE_SIZE - 1);

    // hadWrite = true;

    if (newAddr != flashAddr) {
        flushFlash();

        // writing previous cached data, skip current data until flashing is done
        // tinyusb stack will invoke write_block() with the same parameters later on
        if ( _is_flashing ) return;

        flashAddr = newAddr;
        memcpy(flashBuf, (void *)newAddr, FLASH_PAGE_SIZE);
    }
    memcpy(flashBuf + (dst & (FLASH_PAGE_SIZE - 1)), src, len);
}

#if 0
void uf2_timer(void *p_context) {
    UNUSED_PARAMETER(p_context);
    if (hadWrite) {
        flushFlash();
        s_dfu_settings.bank_0.bank_code = NRF_DFU_BANK_VALID_APP;
        int32_t start = SD_MAGIC_OK() ? MAIN_APPLICATION_START_ADDR : MBR_SIZE;
        int32_t sz = s_dfu_settings.bank_0.image_size - start;
        if (sz > 0)
            s_dfu_settings.bank_0.image_size = sz;
        nrf_dfu_settings_write(NULL);
    }
    NVIC_SystemReset();
}

void uf2_timer_start(int ms);
#endif

void padded_memcpy(char *dst, const char *src, int len) {
    for (int i = 0; i < len; ++i) {
        if (*src)
            *dst = *src++;
        else
            *dst = ' ';
        dst++;
    }
}


/*------------------------------------------------------------------*/
/*
 *------------------------------------------------------------------*/
void read_block(uint32_t block_no, uint8_t *data) {
    memset(data, 0, 512);
    uint32_t sectionIdx = block_no;

    if (block_no == 0) {
        memcpy(data, &BootBlock, sizeof(BootBlock));
        data[510] = 0x55;
        data[511] = 0xaa;
        // logval("data[0]", data[0]);
    } else if (block_no < START_ROOTDIR) {
        sectionIdx -= START_FAT0;
        // logval("sidx", sectionIdx);
        if (sectionIdx >= SECTORS_PER_FAT)
            sectionIdx -= SECTORS_PER_FAT;
        if (sectionIdx == 0) {
            data[0] = 0xf0;
            for (int i = 1; i < NUM_INFO * 2 + 4; ++i) {
                data[i] = 0xff;
            }
        }
        for (int i = 0; i < 256; ++i) {
            uint32_t v = sectionIdx * 256 + i;
            if (UF2_FIRST_SECTOR <= v && v <= UF2_LAST_SECTOR)
                ((uint16_t *)(void *)data)[i] = v == UF2_LAST_SECTOR ? 0xffff : v + 1;
        }
    } else if (block_no < START_CLUSTERS) {
        sectionIdx -= START_ROOTDIR;
        if (sectionIdx == 0) {
            DirEntry *d = (void *)data;
            padded_memcpy(d->name, (const char *)BootBlock.VolumeLabel, 11);
            d->attrs = 0x28;
            for (int i = 0; i < NUM_INFO; ++i) {
                d++;
                const struct TextFile *inf = &info[i];
                d->size = inf->content ? strlen(inf->content) : UF2_SIZE;
                d->startCluster = i + 2;
                padded_memcpy(d->name, inf->name, 11);
            }
        }
    } else {
        sectionIdx -= START_CLUSTERS;
        if (sectionIdx < NUM_INFO - 1) {
            memcpy(data, info[sectionIdx].content, strlen(info[sectionIdx].content));
        } else {
            sectionIdx -= NUM_INFO - 1;
            uint32_t addr = USER_FLASH_START + sectionIdx * 256;
            if (addr < USER_FLASH_START+FLASH_SIZE) {
                UF2_Block *bl = (void *)data;
                bl->magicStart0 = UF2_MAGIC_START0;
                bl->magicStart1 = UF2_MAGIC_START1;
                bl->magicEnd = UF2_MAGIC_END;
                bl->blockNo = sectionIdx;
                bl->numBlocks = FLASH_SIZE / 256;
                bl->targetAddr = addr;
                bl->payloadSize = 256;
                memcpy(bl->data, (void *)addr, bl->payloadSize);
            }
        }
    }
}

/** Write an block
 *
 * @return number of bytes processed, only 3 following values
 *  -1 : if not an uf2 block
 * 512 : write is successful
 *   0 : is busy with flashing, tinyusb stack will call write_block again with the same parameters later on
 */
int write_block(uint32_t block_no, uint8_t *data, bool quiet/*, WriteState *state*/) {
    UF2_Block *bl = (void *)data;

     NRF_LOG_DEBUG("Write magic: %x", bl->magicStart0);

    if (!is_uf2_block(bl)) {
        return -1;
    }

    // still busy with flashing previous blocks
    if ( _is_flashing ) return 0;

    if ((bl->flags & UF2_FLAG_NOFLASH) || bl->payloadSize > 256 || (bl->targetAddr & 0xff) ||
        bl->targetAddr < USER_FLASH_START || bl->targetAddr + bl->payloadSize > USER_FLASH_END) {
#if USE_DBG_MSC
        if (!quiet)
            logval("invalid target addr", bl->targetAddr);
#endif
        NRF_LOG_WARNING("Skip block at %x", bl->targetAddr);
        // this happens when we're trying to re-flash CURRENT.UF2 file previously
        // copied from a device; we still want to count these blocks to reset properly
    } else {
        // logval("write block at", bl->targetAddr);
        NRF_LOG_DEBUG("Write block at %x", bl->targetAddr);
        flash_write(bl->targetAddr, bl->data, bl->payloadSize);
    }

    // flash_write cause a flush to write previous cached data, this write data is not consumed yet
    if ( _is_flashing ) return 0;

//    bool isSet = false;

    if (state && bl->numBlocks) {
        if (state->numBlocks != bl->numBlocks) {
            if (bl->numBlocks >= MAX_BLOCKS || state->numBlocks)
                state->numBlocks = 0xffffffff;
            else
                state->numBlocks = bl->numBlocks;
        }
        if (bl->blockNo < MAX_BLOCKS) {
            uint8_t mask = 1 << (bl->blockNo % 8);
            uint32_t pos = bl->blockNo / 8;
            if (!(state->writtenMask[pos] & mask)) {
                // logval("incr", state->numWritten);
                state->writtenMask[pos] |= mask;
                state->numWritten++;
            }
            if (state->numWritten >= state->numBlocks) {
                // wait a little bit before resetting, to avoid Windows transmit error
                // https://github.com/Microsoft/uf2-samd21/issues/11
                if (!quiet) {
                    // uf2_timer_start(30);
                    // isSet = true;
                }

                // flush last blocks
                flushFlash();
            }
        }
        NRF_LOG_DEBUG("wr %d=%d (of %d)", state->numWritten, bl->blockNo, bl->numBlocks);
    }

//    if (!isSet && !quiet) {
//        // uf2_timer_start(500);
//    }

    return 512;
}

