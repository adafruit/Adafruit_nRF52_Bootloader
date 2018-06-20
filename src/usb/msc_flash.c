/**************************************************************************/
/*!
    @file     msc_flash.c
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2018, Adafruit Industries (adafruit.com)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#include "msc_flash.h"

#if CFG_TUD_MSC

#include "pstorage.h"

// for formatting fatfs when Softdevice is not enabled
#include "nrf_nvmc.h"

/*------------------------------------------------------------------*/
/* MACRO TYPEDEF CONSTANT ENUM
 *------------------------------------------------------------------*/
#define BOOTSECT_SIGNATURE   0xAA55

enum
{
  WRITE10_IDLE,
  WRITE10_ERASING,
  WRITE10_ERASED,
  WRITE10_WRITING,
  WRITE10_WRITTEN,
  WRITE10_FAILED
};

enum { FL_PAGE_SIZE = 4096 };

/*------------------------------------------------------------------*/
/* VARIABLES
 *------------------------------------------------------------------*/
static uint8_t _page_cached[FL_PAGE_SIZE] ATTR_ALIGNED(4);

volatile static uint8_t _wr10_state;
static pstorage_handle_t _fat_psh = { .module_id = 0, .block_id = MSC_FLASH_ADDR_START } ;

static scsi_inquiry_data_t const mscd_inquiry_data =
{
    .is_removable         = 1,
    .version              = 2,
    .response_data_format = 2,
    .vendor_id            = "Adafruit",
    .product_id           = "Feather52840",
    .product_revision     = "1.0"
};

static scsi_read_capacity10_data_t const mscd_read_capacity10_data =
{
    .last_lba   = ENDIAN_BE(MSC_FLASH_BLOCK_NUM-1), // read capacity
    .block_size = ENDIAN_BE(MSC_FLASH_BLOCK_SIZE)
};

static scsi_sense_fixed_data_t mscd_sense_data =
{
    .response_code        = 0x70,
    .sense_key            = 0, // no errors
    .additional_sense_len = sizeof(scsi_sense_fixed_data_t) - 8
};

static scsi_read_format_capacity_data_t const mscd_format_capacity_data =
{
    .list_length     = 8,
    .block_num       = ENDIAN_BE(MSC_FLASH_BLOCK_NUM), // write capacity
    .descriptor_type = 2, // TODO formatted media, refractor to const
    .block_size_u16  = ENDIAN_BE16(MSC_FLASH_BLOCK_SIZE)
};

static scsi_mode_parameters_t const msc_dev_mode_para =
{
    .mode_data_length        = 3,
    .medium_type             = 0,
    .device_specific_para    = 0,
    .block_descriptor_length = 0
};

/*------------------------------------------------------------------*/
/*
 *------------------------------------------------------------------*/
static inline uint32_t lba2addr(uint32_t lba)
{
  return MSC_FLASH_ADDR_START + lba*MSC_FLASH_BLOCK_SIZE;
}

static inline bool fat12_formatted(void);
static void fat12_mkfs(void);


/*------------------------------------------------------------------*/
/*
 *------------------------------------------------------------------*/
static void fat_pstorage_cb(pstorage_handle_t * p_handle, uint8_t op_code, uint32_t result, uint8_t  * p_data, uint32_t  data_len)
{
  if ( result != NRF_SUCCESS )
  {
    _wr10_state = WRITE10_FAILED;
    TU_ASSERT(false, );
  }

  if ( PSTORAGE_CLEAR_OP_CODE == op_code)
  {
    if ( WRITE10_ERASING == _wr10_state) _wr10_state = WRITE10_ERASED;
  }
  else if ( PSTORAGE_STORE_OP_CODE ==  op_code)
  {
    if ( WRITE10_WRITING == _wr10_state) _wr10_state = WRITE10_WRITTEN;
  }
}

/*------------------------------------------------------------------*/
/* API
 *------------------------------------------------------------------*/
void msc_flash_init(void)
{
  pstorage_module_param_t  fat_psp = { .cb = fat_pstorage_cb};
  pstorage_register(&fat_psp, &_fat_psh);

  if ( !fat12_formatted() )
  {
    // SoftDevice must not be enabled yet since we will use raw flash API
    fat12_mkfs();
  }
}

void msc_flash_mount(void)
{
  _wr10_state = WRITE10_IDLE;
}

void msc_flash_umount(void)
{

}

//--------------------------------------------------------------------+
// tinyusb callbacks
//--------------------------------------------------------------------+
int32_t tud_msc_scsi_cb (uint8_t rhport, uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize)
{
  // read10 & write10 has their own callback and MUST not be handled here

  void const* ptr = NULL;
  uint16_t len = 0;

  // most scsi handled is input
  bool in_xfer = true;

  switch (scsi_cmd[0])
  {
    case SCSI_CMD_INQUIRY:
      ptr = &mscd_inquiry_data;
      len = sizeof(scsi_inquiry_data_t);
    break;

    case SCSI_CMD_READ_CAPACITY_10:
      ptr = &mscd_read_capacity10_data;
      len = sizeof(scsi_read_capacity10_data_t);
    break;

    case SCSI_CMD_REQUEST_SENSE:
      ptr = &mscd_sense_data;
      len = sizeof(scsi_sense_fixed_data_t);
    break;

    case SCSI_CMD_READ_FORMAT_CAPACITY:
      ptr = &mscd_format_capacity_data;
      len = sizeof(scsi_read_format_capacity_data_t);
    break;

    case SCSI_CMD_MODE_SENSE_6:
      ptr = &msc_dev_mode_para;
      len = sizeof(msc_dev_mode_para);
    break;

    case SCSI_CMD_TEST_UNIT_READY:
      ptr = NULL;
      len = 0;
    break;

    case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
      ptr = NULL;
      len = 0;
    break;

    default:
      // negative is error -> Data stage is STALL, status = failed
      return -1;
  }

  // return len must not larger than bufsize
  TU_ASSERT( bufsize >= len );

  if ( ptr && len )
  {
    if(in_xfer)
    {
      memcpy(buffer, ptr, len);
    }else
    {
      // SCSI output
    }
  }

  //------------- clear sense data if it is not request sense command -------------//
  if ( SCSI_CMD_REQUEST_SENSE != scsi_cmd[0] )
  {
    mscd_sense_data.sense_key                  = SCSI_SENSEKEY_NONE;
    mscd_sense_data.additional_sense_code      = 0;
    mscd_sense_data.additional_sense_qualifier = 0;
  }

  return len;
}

/*------------------------------------------------------------------*/
/* Tinyusb Flash READ10 & WRITE10
 *------------------------------------------------------------------*/
static bool fl_page_erase(uint32_t addr)
{
  _fat_psh.block_id = addr;
  return NRF_SUCCESS == pstorage_clear(&_fat_psh, FL_PAGE_SIZE);
}

static bool fl_page_write(uint32_t addr, uint8_t* buf, uint16_t bufsize)
{
  _fat_psh.block_id = addr;
  return NRF_SUCCESS == pstorage_store(&_fat_psh, buf, bufsize, 0);
}

int32_t tud_msc_read10_cb (uint8_t rhport, uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize)
{
  (void) rhport; (void) lun;

  uint32_t addr = lba2addr(lba) + offset;
  memcpy(buffer, (uint8_t*) addr, bufsize);

  return bufsize;
}

int32_t tud_msc_write10_cb (uint8_t rhport, uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize)
{
  (void) rhport; (void) lun;

  uint32_t addr = lba2addr(lba) + offset;

  /* 0. Check if flash is the same as data -> skip if matches
   * 1. queue flash erase pstorage_clear(), return 0 until erasing is done
   * 2. queue flash writing, return 0 until writing is complete
   * 3. return written bytes.
   *
   * Note since CFG_TUD_MSC_BUFSIZE is 4KB, bufsize is cap at 4KB
   */

  switch(_wr10_state)
  {
    case WRITE10_IDLE:
    {
      // No need to write if flash's content matches with data
      if ( 0 == memcmp(buffer, (void*) addr, bufsize) ) return bufsize;

      uint32_t page_addr = align4k(addr);
      uint32_t off4k     = offset4k(addr);

      // Cache contents from start of page to current address
      if ( off4k )
      {
        memcpy(_page_cached, (uint8_t*) page_addr, off4k);
      }

      // Copy new data
      memcpy(_page_cached+off4k, buffer, bufsize);

      // Cache contents after data to end of page
      if ( off4k + bufsize < FL_PAGE_SIZE)
      {
        memcpy(_page_cached+off4k+bufsize, (uint8_t*) (addr+bufsize), FL_PAGE_SIZE - (off4k + bufsize ) );
      }

      // Start erasing
      TU_ASSERT( fl_page_erase(align4k(addr)), -1);

      _wr10_state = WRITE10_ERASING;

      // Tell tinyusb that we are not ready to consume its data
      // The stack will keep the data and call again
      return 0;
    }
    break;

    case WRITE10_ERASING:
      // still erasing, nothing else to do
      return 0;
    break;

    case WRITE10_ERASED:
      // Start writing
      TU_ASSERT( fl_page_write(align4k(addr), _page_cached, FL_PAGE_SIZE), -1);
      _wr10_state = WRITE10_WRITING;
      return 0;
    break;
    
    case WRITE10_WRITING:
      return 0;
    break;

    case WRITE10_WRITTEN:
      _wr10_state = WRITE10_IDLE; // back to idle

      // positive return means we complete the operation, tinyusb can receiving next write10
      return bufsize;
    break;

    case WRITE10_FAILED:
      _wr10_state = WRITE10_IDLE;
      return -1;
    break;

    default: return -1; break;
  }
}


//--------------------------------------------------------------------+
// FAT12
//--------------------------------------------------------------------+
typedef struct ATTR_PACKED {
  uint8_t  jump_code[3]       ; ///< Assembly instruction to jump to boot code.
  uint8_t  oem_name[8]        ; ///< OEM Name in ASCII.

  // Bios Parameter Block
  uint16_t sector_sz          ; ///< Bytes per sector. Allowed values include 512, 1024, 2048, and 4096.
  uint8_t  sector_per_cluster ; ///< Sectors per cluster (data unit). Allowed values are powers of 2, but the cluster size must be 32KB or smaller.
  uint16_t reserved_sectors   ; ///< Size in sectors of the reserved area.

  uint8_t  fat_copies         ; ///< Number of FATs. Typically two for redundancy, but according to Microsoft it can be one for some small storage devices.
  uint16_t root_entry_count   ; ///< Maximum number of files in the root directory for FAT12 and FAT16. This is 0 for FAT32 and typically 512 for FAT16.
  uint16_t sector_count       ; ///< 16-bit number of sectors in file system. If the number of sectors is larger than can be represented in this 2-byte value, a 4-byte value exists later in the data structure and this should be 0.
  uint8_t  media_type         ; ///< 0xf8 should be used for fixed disks and 0xf0 for removable.
  uint16_t sector_per_fat     ; ///< 16-bit size in sectors of each FAT for FAT12 and FAT16. For FAT32, this field is 0.
  uint16_t sector_per_track   ; ///< Sectors per track of storage device.
  uint16_t head_num           ; ///< Number of heads in storage device.
  uint32_t not_used1          ; ///< Number of sectors before the start of partition.
  uint32_t not_used2          ; ///< 32-bit value of number of sectors in file system. Either this value or the 16-bit value above must be 0.

  // Extended BPB
  uint8_t  drive_number       ; ///< Physical drive number (0x00 for (first) removable media, 0x80 for (first) fixed disk
  uint8_t  not_used3          ; ///< Some OS uses this as drive letter (e.g 0 = C:, 1 = D: etc ...), should be 0 when formatted
  uint8_t  ext_boot_signature ; ///< should be 0x29
  uint32_t volume_id          ; ///< Volume serial number, which some versions of Windows will calculate based on the creation date and time.
  uint8_t  volume_label[11]   ;
  uint8_t  fs_type[8]         ; ///< File system type label in ASCII, padded with blank (0x20). Standard values include "FAT," "FAT12," and "FAT16," but nothing is required.
//  uint8_t  not_used4[448]     ;
//  uint16_t signature          ; ///< Signature value (0xAA55).
}fat12_boot_sector_t;

VERIFY_STATIC(sizeof(fat12_boot_sector_t) == 62, "size is not correct");

typedef ATTR_PACKED_STRUCT(struct) {
  uint8_t name[11];

  ATTR_PACKED_STRUCT(struct){
    uint8_t readonly       : 1;
    uint8_t hidden         : 1;
    uint8_t system         : 1;
    uint8_t volume_label   : 1;
    uint8_t directory      : 1;
    uint8_t archive        : 1;
  } attr; // Long File Name = 0x0f

  uint8_t reserved;
  uint8_t created_time_tenths_of_seconds;
  uint16_t created_time;
  uint16_t created_date;
  uint16_t accessed_date;
  uint16_t cluster_high;
  uint16_t written_time;
  uint16_t written_date;
  uint16_t cluster_low;
  uint32_t file_size;
}fat_directory_t;

VERIFY_STATIC(sizeof(fat_directory_t) == 32, "size is not correct");


fat12_boot_sector_t const _boot_sect =
{
    .jump_code          = { 0xEB, 0xFE, 0x90 },
    .oem_name           = "MSDOS5.0",

    .sector_sz          = MSC_FLASH_BLOCK_SIZE,
    .sector_per_cluster = 1,
    .reserved_sectors   = 1,

    .fat_copies         = 1,
    .root_entry_count   = 16*8,
    .sector_count       = MSC_FLASH_BLOCK_NUM,
    .media_type         = 0xf8, // fixed disk
    .sector_per_fat     = 7,
    .sector_per_track   = 1,
    .head_num           = 1,
    .not_used1          = 0,
    .not_used2          = 0,

    .drive_number       = 0,
    .not_used3          = 0,
    .ext_boot_signature = 0x29,
    .volume_id          = 0x00420042, // change later to typically date + time
    .volume_label       = MSC_FLASH_VOL_LABEL,
    .fs_type            = "FAT12   "
};

/* 0  ______________
 *   | Boot Sector |  Fat copies = 1, Fat size = 7
 *   |_____________|  Root Dir = 16*8 (4KB), sector per cluster = 1
 * 1 |    FAT0     |
 * . |             |
 * . |  ( 7x512 )  |
 * 7 |_____________|
 * 8 |   Root Dir  |
 * . |             |
 * . |  ( 8x512 )  |
 * 15|_____________|
 * 16|             |
 * . |             |
 * . |             |
 * . |    Data     |
 * . |    Area     |
 * . |             |
 * . |             |
 *   |_____________|
 */

static inline bool fat12_formatted(void)
{
  const uint8_t* boot_sect = (uint8_t* ) lba2addr(0);
  return (boot_sect[510] == 0x55) && (boot_sect[511] == 0xAA);
}

static void fat12_write_sector(uint32_t lba, uint8_t const* buf, uint32_t bufsize)
{
  uint32_t addr = lba2addr(lba);

  nrf_nvmc_page_erase( addr );
  nrf_nvmc_write_words(addr, (uint32_t const*) buf, bufsize/4);
}

static void fat12_mkfs(void)
{
  memclr_(_page_cached, sizeof(_page_cached));

  /*------------- Sector 0: Boot Sector -------------*/
  memcpy(_page_cached, &_boot_sect, sizeof(_boot_sect));
  _page_cached[510] = 0x55;
  _page_cached[511] = 0xAA;

  //------------- Sector 1: FAT12 Table  -------------//
  // first 2 entries are FF8 and FF8,
  // 3rd entry is cluster end of readme file FFF, 4th is unused 000
  memcpy(_page_cached+MSC_FLASH_BLOCK_SIZE, "\xF8\x8F\xFF\xFF\x0F\x00", 6);

  // Erase and Write first cluster.
  fat12_write_sector(0, _page_cached, FL_PAGE_SIZE);

  //------------- Root Directory cluster -------------//
  uint8_t const readme_contents[] = "Adafruit Feather nRF52840";

  memclr_(_page_cached, sizeof(_page_cached));
  fat_directory_t* p_entry = (fat_directory_t*) _page_cached;

  // first entry is volume label
  (*p_entry) = (fat_directory_t)
  {
    .name = MSC_FLASH_VOL_LABEL,
    .attr.volume_label = 1,
  };

  p_entry += 1; // advance to second entry, which is readme file
  (*p_entry) = (fat_directory_t)
  {
    .name = "README  TXT",

    .attr = { 0 },
    .reserved = 0,
    .created_time_tenths_of_seconds = 0,

    .created_time = 0x6D52,
    .created_date = 0x4365,
    .accessed_date = 0x4365,

    .cluster_high = 0,

    .written_time = 0x6D52,
    .written_date = 0x4365,

    .cluster_low = 2,
    .file_size = sizeof(readme_contents)-1 // exclude NULL
  };

  // Erase and Write cluster.
  fat12_write_sector(8, _page_cached, FL_PAGE_SIZE);

  //------------- Readme Content cluster -------------//
  memclr_(_page_cached, sizeof(_page_cached));
  memcpy(_page_cached, readme_contents, sizeof(readme_contents)-1);

  // Erase and Write cluster.
  fat12_write_sector(16, _page_cached, FL_PAGE_SIZE);
}

#endif
