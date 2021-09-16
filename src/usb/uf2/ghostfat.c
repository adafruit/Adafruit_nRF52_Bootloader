/*
 * The MIT License (MIT)
 *
 * Copyright (c) Microsoft Corporation
 * Copyright (c) 2020 Ha Thach for Adafruit Industries
 * Copyright (c) 2020 Henry Gabryjelski
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "compile_date.h"

#include "uf2.h"
#include "configkeys.h"
#include "flash_nrf5x.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "bootloader_settings.h"
#include "bootloader.h"

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

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
  char const name[11];
  char const *content;
};


//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

#define BPB_SECTOR_SIZE           ( 512)
#define BPB_SECTORS_PER_CLUSTER   (   1)
#define BPB_RESERVED_SECTORS      (   1)
#define BPB_NUMBER_OF_FATS        (   2)
#define BPB_ROOT_DIR_ENTRIES      (  64)
#define BPB_TOTAL_SECTORS         CFG_UF2_NUM_BLOCKS
#define BPB_MEDIA_DESCRIPTOR_BYTE (0xF8)
#define FAT_ENTRY_SIZE            (2)
#define FAT_ENTRIES_PER_SECTOR    (BPB_SECTOR_SIZE / FAT_ENTRY_SIZE)
// NOTE: MS specification explicitly allows FAT to be larger than necessary
#define BPB_SECTORS_PER_FAT       ( (BPB_TOTAL_SECTORS / FAT_ENTRIES_PER_SECTOR) + \
                                   ((BPB_TOTAL_SECTORS % FAT_ENTRIES_PER_SECTOR) ? 1 : 0))
#define DIRENTRIES_PER_SECTOR     (BPB_SECTOR_SIZE/sizeof(DirEntry))
#define ROOT_DIR_SECTOR_COUNT     (BPB_ROOT_DIR_ENTRIES/DIRENTRIES_PER_SECTOR)

STATIC_ASSERT(BPB_SECTOR_SIZE                              ==       512); // GhostFAT does not support other sector sizes (currently)
STATIC_ASSERT(BPB_SECTORS_PER_CLUSTER                      ==         1); // GhostFAT presumes one sector == one cluster (for simplicity)
STATIC_ASSERT(BPB_NUMBER_OF_FATS                           ==         2); // FAT highest compatibility
STATIC_ASSERT(sizeof(DirEntry)                             ==        32); // FAT requirement
STATIC_ASSERT(BPB_SECTOR_SIZE % sizeof(DirEntry)           ==         0); // FAT requirement
STATIC_ASSERT(BPB_ROOT_DIR_ENTRIES % DIRENTRIES_PER_SECTOR ==         0); // FAT requirement
STATIC_ASSERT(BPB_SECTOR_SIZE * BPB_SECTORS_PER_CLUSTER    <= (32*1024)); // FAT requirement (64k+ has known compatibility problems)
STATIC_ASSERT(FAT_ENTRIES_PER_SECTOR                       ==       256); // FAT requirement

#define STR0(x) #x
#define STR(x) STR0(x)

char infoUf2File[128*3] =
    "UF2 Bootloader " UF2_VERSION "\r\n"
    "Model: " UF2_PRODUCT_NAME "\r\n"
    "Board-ID: " UF2_BOARD_ID "\r\n"
    "Date: " __DATE__ "\r\n";

const char indexFile[] =
    "<!doctype html>\n"
    "<html>"
    "<body>"
    "<script>\n"
    "location.replace(\"" UF2_INDEX_URL "\");\n"
    "</script>"
    "</body>"
    "</html>\n";

static struct TextFile const info[] = {
    {.name = "INFO_UF2TXT", .content = infoUf2File},
    {.name = "INDEX   HTM", .content = indexFile},

    // current.uf2 must be the last element and its content must be NULL
    {.name = "CURRENT UF2", .content = NULL},
};
STATIC_ASSERT(ARRAY_SIZE(infoUf2File) < BPB_SECTOR_SIZE); // GhostFAT requires files to fit in one sector
STATIC_ASSERT(ARRAY_SIZE(indexFile)   < BPB_SECTOR_SIZE); // GhostFAT requires files to fit in one sector

#define NUM_FILES          (ARRAY_SIZE(info))
#define NUM_DIRENTRIES     (NUM_FILES + 1) // Code adds volume label as first root directory entry
#define REQUIRED_ROOT_DIRECTORY_SECTORS ( ((NUM_DIRENTRIES+1) / DIRENTRIES_PER_SECTOR) + \
                                         (((NUM_DIRENTRIES+1) % DIRENTRIES_PER_SECTOR) ? 1 : 0))
STATIC_ASSERT(ROOT_DIR_SECTOR_COUNT >= REQUIRED_ROOT_DIRECTORY_SECTORS);         // FAT requirement -- Ensures BPB reserves sufficient entries for all files
STATIC_ASSERT(NUM_DIRENTRIES < (DIRENTRIES_PER_SECTOR * ROOT_DIR_SECTOR_COUNT)); // FAT requirement -- end directory with unused entry
STATIC_ASSERT(NUM_DIRENTRIES < BPB_ROOT_DIR_ENTRIES);                            // FAT requirement -- Ensures BPB reserves sufficient entries for all files
STATIC_ASSERT(NUM_DIRENTRIES < DIRENTRIES_PER_SECTOR); // GhostFAT bug workaround -- else, code overflows buffer

#define NUM_SECTORS_IN_DATA_REGION (BPB_TOTAL_SECTORS - BPB_RESERVED_SECTORS - (BPB_NUMBER_OF_FATS * BPB_SECTORS_PER_FAT) - ROOT_DIR_SECTOR_COUNT)
#define CLUSTER_COUNT              (NUM_SECTORS_IN_DATA_REGION / BPB_SECTORS_PER_CLUSTER)

// Ensure cluster count results in a valid FAT16 volume!
STATIC_ASSERT( CLUSTER_COUNT >= 0x0FF5 && CLUSTER_COUNT < 0xFFF5 );

// Many existing FAT implementations have small (1-16) off-by-one style errors
// So, avoid being within 32 of those limits for even greater compatibility.
STATIC_ASSERT( CLUSTER_COUNT >= 0x1015 && CLUSTER_COUNT < 0xFFD5 );


#define UF2_FIRMWARE_BYTES_PER_SECTOR 256
#define TRUE_USER_FLASH_SIZE (USER_FLASH_END-USER_FLASH_START)
STATIC_ASSERT(TRUE_USER_FLASH_SIZE % UF2_FIRMWARE_BYTES_PER_SECTOR == 0); // UF2 requirement -- overall size must be integral multiple of per-sector payload?

#define UF2_SECTORS        ( (TRUE_USER_FLASH_SIZE / UF2_FIRMWARE_BYTES_PER_SECTOR) + \
                            ((TRUE_USER_FLASH_SIZE % UF2_FIRMWARE_BYTES_PER_SECTOR) ? 1 : 0))
#define UF2_SIZE           (UF2_SECTORS * BPB_SECTOR_SIZE)

STATIC_ASSERT(UF2_SECTORS == ((UF2_SIZE/2) / 256)); // Not a requirement ... ensuring replacement of literal value is not a change

#define UF2_FIRST_SECTOR   ((NUM_FILES + 1) * BPB_SECTORS_PER_CLUSTER) // WARNING -- code presumes each non-UF2 file content fits in single sector
#define UF2_LAST_SECTOR    ((UF2_FIRST_SECTOR + UF2_SECTORS - 1) * BPB_SECTORS_PER_CLUSTER)

#define FS_START_FAT0_SECTOR      BPB_RESERVED_SECTORS
#define FS_START_FAT1_SECTOR      (FS_START_FAT0_SECTOR + BPB_SECTORS_PER_FAT)
#define FS_START_ROOTDIR_SECTOR   (FS_START_FAT1_SECTOR + BPB_SECTORS_PER_FAT)
#define FS_START_CLUSTERS_SECTOR  (FS_START_ROOTDIR_SECTOR + ROOT_DIR_SECTOR_COUNT)


static FAT_BootBlock const BootBlock = {
    .JumpInstruction      = {0xeb, 0x3c, 0x90},
    .OEMInfo              = "UF2 UF2 ",
    .SectorSize           = BPB_SECTOR_SIZE,
    .SectorsPerCluster    = BPB_SECTORS_PER_CLUSTER,
    .ReservedSectors      = BPB_RESERVED_SECTORS,
    .FATCopies            = BPB_NUMBER_OF_FATS,
    .RootDirectoryEntries = BPB_ROOT_DIR_ENTRIES,
    .TotalSectors16       = (BPB_TOTAL_SECTORS > 0xFFFF) ? 0 : BPB_TOTAL_SECTORS,
    .MediaDescriptor      = BPB_MEDIA_DESCRIPTOR_BYTE,
    .SectorsPerFAT        = BPB_SECTORS_PER_FAT,
    .SectorsPerTrack      = 1,
    .Heads                = 1,
    .TotalSectors32       = (BPB_TOTAL_SECTORS > 0xFFFF) ? BPB_TOTAL_SECTORS : 0,
    .PhysicalDriveNum     = 0x80, // to match MediaDescriptor of 0xF8
    .ExtendedBootSig      = 0x29,
    .VolumeSerialNumber   = 0x00420042,
    .VolumeLabel          = UF2_VOLUME_LABEL,
    .FilesystemIdentifier = "FAT16   ",
};

// Use bootloaderConfig to detect BOOTLOADER ID when updating bootloader
// This helps to prevent incorrect uf2 from other boards.
extern const uint32_t bootloaderConfig[];

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
static inline bool is_uf2_block (UF2_Block const *bl)
{
  return (bl->magicStart0 == UF2_MAGIC_START0) &&
         (bl->magicStart1 == UF2_MAGIC_START1) &&
         (bl->magicEnd == UF2_MAGIC_END) &&
         (bl->flags & UF2_FLAG_FAMILYID) &&
         !(bl->flags & UF2_FLAG_NOFLASH) &&
         (bl->payloadSize == UF2_FIRMWARE_BYTES_PER_SECTOR) &&
         !(bl->targetAddr & 0xff);
}

// used when upgrading application
static inline bool in_app_space (uint32_t addr)
{
  return USER_FLASH_START <= addr && addr < USER_FLASH_END;
}

// used when upgrading bootloader
static inline bool in_bootloader_space (uint32_t addr)
{
  return BOOTLOADER_ADDR_START <= addr && addr < BOOTLOADER_ADDR_END;
}

// used when upgrading bootloader
static inline bool in_uicr_space(uint32_t addr)
{
  return addr == 0x10001000;
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

void uf2_init(void)
{
  strcat(infoUf2File, "SoftDevice: ");

  if ( is_sd_existed() )
  {
    uint32_t const sd_id      = SD_ID_GET(MBR_SIZE);
    uint32_t const sd_version = SD_VERSION_GET(MBR_SIZE);

    uint32_t ver[3];
    ver[0] = sd_version / 1000000;
    ver[1] = (sd_version - ver[0]*1000000)/1000;
    ver[2] = (sd_version - ver[0]*1000000 - ver[1]*1000);

    char str[10];
    utoa(sd_id, str, 10);

    strcat(infoUf2File, "S");
    strcat(infoUf2File, str);
    strcat(infoUf2File, " ");

    utoa(ver[0], str, 10);
    strcat(infoUf2File, str);
    strcat(infoUf2File, ".");

    utoa(ver[1], str, 10);
    strcat(infoUf2File, str);
    strcat(infoUf2File, ".");

    utoa(ver[2], str, 10);
    strcat(infoUf2File, str);
    strcat(infoUf2File, "\r\n");
  }else
  {
    strcat(infoUf2File, "not found\r\n");
  }
}

/*------------------------------------------------------------------*/
/* Read CURRENT.UF2
 *------------------------------------------------------------------*/
void padded_memcpy (char *dst, char const *src, int len)
{
  for ( int i = 0; i < len; ++i )
  {
    if ( *src ) {
      *dst = *src++;
    } else {
      *dst = ' ';
    }
    dst++;
  }
}

void read_block(uint32_t block_no, uint8_t *data) {
    memset(data, 0, BPB_SECTOR_SIZE);
    uint32_t sectionIdx = block_no;

    if (block_no == 0) { // Requested boot block
        memcpy(data, &BootBlock, sizeof(BootBlock));
        data[510] = 0x55; // Always at offsets 510/511, even when BPB_SECTOR_SIZE is larger
        data[511] = 0xaa; // Always at offsets 510/511, even when BPB_SECTOR_SIZE is larger
        // logval("data[0]", data[0]);
    } else if (block_no < FS_START_ROOTDIR_SECTOR) {  // Requested FAT table sector
        sectionIdx -= FS_START_FAT0_SECTOR;
        // logval("sidx", sectionIdx);
        if (sectionIdx >= BPB_SECTORS_PER_FAT) {
            sectionIdx -= BPB_SECTORS_PER_FAT; // second FAT is same as the first...
        }
        if (sectionIdx == 0) {
            // first FAT entry must match BPB MediaDescriptor
            data[0] = BPB_MEDIA_DESCRIPTOR_BYTE;
            // WARNING -- code presumes only one NULL .content for .UF2 file
            //            and all non-NULL .content fit in one sector
            //            and requires it be the last element of the array
            uint32_t const end = (NUM_FILES * FAT_ENTRY_SIZE) + (2 * FAT_ENTRY_SIZE);
            for (uint32_t i = 1; i < end; ++i) {
                data[i] = 0xff;
            }
        }
        for (uint32_t i = 0; i < FAT_ENTRIES_PER_SECTOR; ++i) { // Generate the FAT chain for the firmware "file"
            uint32_t v = (sectionIdx * FAT_ENTRIES_PER_SECTOR) + i;
            if (UF2_FIRST_SECTOR <= v && v <= UF2_LAST_SECTOR)
                ((uint16_t *)(void *)data)[i] = v == UF2_LAST_SECTOR ? 0xffff : v + 1;
        }
    } else if (block_no < FS_START_CLUSTERS_SECTOR) { // Requested root directory sector

        sectionIdx -= FS_START_ROOTDIR_SECTOR;

        DirEntry *d = (void *)data;
        int remainingEntries = DIRENTRIES_PER_SECTOR;
        if (sectionIdx == 0) { // volume label first
            // volume label is first directory entry
            padded_memcpy(d->name, (char const *) BootBlock.VolumeLabel, 11);
            d->attrs = 0x28;
            d++;
            remainingEntries--;
        }

        for (uint32_t i = DIRENTRIES_PER_SECTOR * sectionIdx;
             remainingEntries > 0 && i < NUM_FILES;
             i++, d++) {

            // WARNING -- code presumes all but last file take exactly one sector
            uint16_t startCluster = i + 2;

            struct TextFile const * inf = &info[i];
            padded_memcpy(d->name, inf->name, 11);
            d->createTimeFine   = __SECONDS_INT__ % 2 * 100;
            d->createTime       = __DOSTIME__;
            d->createDate       = __DOSDATE__;
            d->lastAccessDate   = __DOSDATE__;
            d->highStartCluster = startCluster >> 16;
            // DIR_WrtTime and DIR_WrtDate must be supported
            d->updateTime       = __DOSTIME__;
            d->updateDate       = __DOSDATE__;
            d->startCluster     = startCluster & 0xFFFF;
            d->size = (inf->content ? strlen(inf->content) : UF2_SIZE);
        }

    } else if (block_no < BPB_TOTAL_SECTORS) {

        sectionIdx -= FS_START_CLUSTERS_SECTOR;
        if (sectionIdx < NUM_FILES - 1) {
            memcpy(data, info[sectionIdx].content, strlen(info[sectionIdx].content));
        } else { // generate the UF2 file data on-the-fly
            sectionIdx -= NUM_FILES - 1;
            uint32_t addr = USER_FLASH_START + (sectionIdx * UF2_FIRMWARE_BYTES_PER_SECTOR);
            if (addr < CFG_UF2_FLASH_SIZE) {
                UF2_Block *bl = (void *)data;
                bl->magicStart0 = UF2_MAGIC_START0;
                bl->magicStart1 = UF2_MAGIC_START1;
                bl->magicEnd = UF2_MAGIC_END;
                bl->blockNo = sectionIdx;
                bl->numBlocks = UF2_SECTORS;
                bl->targetAddr = addr;
                bl->payloadSize = UF2_FIRMWARE_BYTES_PER_SECTOR;
                bl->flags = UF2_FLAG_FAMILYID;
                bl->familyID = CFG_UF2_BOARD_APP_ID;
                memcpy(bl->data, (void *)addr, bl->payloadSize);
            }
        }

    }
}

/*------------------------------------------------------------------*/
/* Write UF2
 *------------------------------------------------------------------*/

/**
 * Write an uf2 block wrapped by 512 sector.
 * @return number of bytes processed, only 3 following values
 *  -1 : if not an uf2 block
 * 512 : write is successful (BPB_SECTOR_SIZE == 512)
 *   0 : is busy with flashing, tinyusb stack will call write_block again with the same parameters later on
 */
int write_block (uint32_t block_no, uint8_t *data, WriteState *state)
{
  UF2_Block *bl = (void*) data;

  if ( !is_uf2_block(bl) ) return -1;

  switch ( bl->familyID )
  {

    case CFG_UF2_BOARD_APP_ID:  // board-specific app
    case CFG_UF2_FAMILY_APP_ID: // family app
      /* Upgrading Application
       *
       * SoftDevice is considered as part of application and can be (or not) included in uf2.
       *
       *                          -------------         -------------
       *                         |             |       |             |
       *                         |  Bootloader |       |  Bootloader |
       *  BOOTLOADER_ADDR_START--|-------------|       |-------------|
       *                         |  App Data   |       |  App Data   |
       *       USER_FLASH_END ---|-------------|       |-------------|
       *                         |             |       |             |
       *                         |             |       |     New     |
       *                         | Application | ----> | Application |
       *                         |             |       |             |
       *       USER_FLASH_START--|-------------|       |-------------|
       *                         |     MBR     |       |     MBR     |
       *                          -------------         -------------
       */
      if ( in_app_space(bl->targetAddr) )
      {
        PRINTF("Write addr = 0x%08lX, block = %ld (%ld of %ld)\r\n", bl->targetAddr, bl->blockNo, state->numWritten, bl->numBlocks);
        flash_nrf5x_write(bl->targetAddr, bl->data, bl->payloadSize, true);
      }else if ( bl->targetAddr < USER_FLASH_START )
      {
        // do nothing if writing to MBR, occurs when SD hex is included
        // keep going as successful write
        PRINTF("skip writing to MBR\r\n");
      }else
      {
        return -1;
      }
    break;

    case CFG_UF2_FAMILY_BOOT_ID:
      /* Upgrading Bootloader
       *
       * - For simplicity, the Bootloader Start Address is fixed for now.
       * - Since SoftDevice is not part of Bootloader, it MUST NOT be included as part of uf2 file.
       * - To prevent corruption/disconnection while transferring we don't directly write over Bootloader.
       * Instead it is written to highest possible address in Application region. Once everything is received
       * and verified, it is safely activated using MBR COPY BL command.
       *
       * - Along with bootloader code, UCIR (at 0x1000100) is also included containing
       * 0x10001014 (bootloader address), and 0x10001018 (MBR Params address).
       *
       * Note: part of the existing application can be affected when updating bootloader.
       * TODO May be worth to have some kind crc/application integrity checking
       *
       *                         -------------         -------------         -------------
       *                        |             |       |             |     + |     New     |
       *                        | Bootloader  |       | Bootloader  |    +  | Bootloader  |
       * BOOTLOADER_ADDR_START--|-------------|       |-------------|   +   |-------------|
       *                        |  App Data   |       |  App Data   |  +    |   App Data  |
       *       USER_FLASH_END --|-------------|       | ----------  | +     |------------ |
       *                        |             |       |     New     |+      |             |
       *                        |             | --->  |  Bootloader |       |             |
       *                        |             |       |   ++++++    |       |             |
       *                        | Application |       | Application |       | Application |
       *                        |             |       |             |       |             |
       *                        |             |       |             |       |             |
       *      USER_FLASH_START--|-------------|       |-------------|       |-------------|
       *                        |     MBR     |       |     MBR     |       |     MBR     |
       *                         -------------         -------------         -------------
       */
      PRINTF("addr = 0x%08lX, block = %ld (%ld of %ld)\r\n", bl->targetAddr, bl->blockNo, state->numWritten, bl->numBlocks);

      state->update_bootloader = true;
      if ( in_uicr_space(bl->targetAddr) )
      {
        /* UCIR contains bootloader & MBR address as follow:
         * - 0x10001014 bootloader address
         * - 0x10001018 MBR Params: mostly fixed
         *
         * Since the bootloader start address is fixed, we only use this for verification
         */
        uint32_t uicr_boot_addr;
        uint32_t uicr_mbr_param;

        memcpy(&uicr_boot_addr, bl->data + 0x14, 4);
        memcpy(&uicr_mbr_param, bl->data + 0x18, 4);

        // Check MBR params is fixed and prohibited to change and
        // Bootloader address against its new size
        if ( (uicr_boot_addr != BOOTLOADER_ADDR_START)  ||
             (uicr_mbr_param != BOOTLOADER_MBR_PARAMS_PAGE_ADDRESS) )
        {
          PRINTF("Incorrect UICR value");
          PRINT_HEX(uicr_boot_addr);
          PRINT_HEX(uicr_mbr_param);

          state->aborted = true;
          return -1;
        }

        state->has_uicr = true;
      }
      else if ( in_bootloader_space(bl->targetAddr) )
      {
        // Bootloader CF2 config
        if ( !state->boot_id_matches && (bl->targetAddr >= ((uint32_t) bootloaderConfig)) )
        {
          // check if bootloader ID matches current VID/PID
          for (uint32_t i=0; i < bl->payloadSize; i += 8)
          {
            uint32_t key;
            memcpy(&key, bl->data+i, 4);

            if ( key == CFG_BOOTLOADER_BOARD_ID )
            {
              uint32_t value;
              memcpy(&value, bl->data+i+4, 4);

              PRINTF("Bootloader ID = 0x%08lX and ", value);
              if ( value == ((USB_DESC_VID << 16) | USB_DESC_UF2_PID) )
              {
                PRINTF("matches our VID/PID\r\n");
                state->boot_id_matches = true;
                break;
              }
              else
              {
                PRINTF("DOES NOT match our VID/PID\r\n");
                state->aborted = true;
                return -1;
              }
            }
          }
        }

        // Offset to write the new bootloader address (skipping the App Data)
        uint32_t const offset_addr = BOOTLOADER_ADDR_END-USER_FLASH_END;
        flash_nrf5x_write(bl->targetAddr-offset_addr, bl->data, bl->payloadSize, true);
      }
#if 0 // don't allow bundle SoftDevice to prevent confusion
      else if ( in_app_space(bl->targetAddr) )
      {
        // Should be Softdevice
        flash_nrf5x_write(bl->targetAddr, bl->data, bl->payloadSize, true);
      }
#endif
      else if ( bl->targetAddr < USER_FLASH_START )
      {
        PRINTF("skip writing to MBR\r\n");
      }
      else
      {
        state->aborted = true;
        return -1;
      }
    break;

    // unknown family ID
    default: return -1;
  }

  //------------- Update written blocks -------------//
  if ( bl->numBlocks )
  {
    // Update state num blocks if needed
    if ( state->numBlocks != bl->numBlocks )
    {
      if ( bl->numBlocks >= MAX_BLOCKS || state->numBlocks )
        state->numBlocks = 0xffffffff;
      else
        state->numBlocks = bl->numBlocks;
    }

    if ( bl->blockNo < MAX_BLOCKS )
    {
      uint8_t const mask = 1 << (bl->blockNo % 8);
      uint32_t const pos = bl->blockNo / 8;

      // only increase written number with new write (possibly prevent overwriting from OS)
      if ( !(state->writtenMask[pos] & mask) )
      {
        state->writtenMask[pos] |= mask;
        state->numWritten++;
      }

      // flush last blocks
      // TODO numWritten can be smaller than numBlocks if return early
      if ( state->numWritten >= state->numBlocks )
      {
        flash_nrf5x_flush(true);

        // Failed if update bootloader without UCIR value
        if ( state->update_bootloader && !state->has_uicr )
        {
          state->aborted = true;
        }
      }
    }
  }

  return BPB_SECTOR_SIZE;
}
