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

static uint8_t _wr10_state;
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

  /* 1. queue flash erase pstorage_clear(), return 0 until erasing is done
   * 2. queue flash writing, return 0 until writing is complete
   * 3. return written bytes.
   *
   * Note since CFG_TUD_MSC_BUFSIZE is 4KB, bufsize is cap at 4KB
   */

  switch(_wr10_state)
  {
    case WRITE10_IDLE:
    {
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
  uint16_t byte_per_sector    ; ///< Bytes per sector. Allowed values include 512, 1024, 2048, and 4096.
  uint8_t  sector_per_cluster ; ///< Sectors per cluster (data unit). Allowed values are powers of 2, but the cluster size must be 32KB or smaller.
  uint16_t reserved_sectors   ; ///< Size in sectors of the reserved area.
  uint8_t  fat_num            ; ///< Number of FATs. Typically two for redundancy, but according to Microsoft it can be one for some small storage devices.
  uint16_t root_entry_count   ; ///< Maximum number of files in the root directory for FAT12 and FAT16. This is 0 for FAT32 and typically 512 for FAT16.
  uint16_t sector_count       ; ///< 16-bit number of sectors in file system. If the number of sectors is larger than can be represented in this 2-byte value, a 4-byte value exists later in the data structure and this should be 0.
  uint8_t  media_type         ; ///< 0xf8 should be used for fixed disks and 0xf0 for removable.
  uint16_t sector_per_fat     ; ///< 16-bit size in sectors of each FAT for FAT12 and FAT16. For FAT32, this field is 0.
  uint16_t sector_per_track   ; ///< Sectors per track of storage device.
  uint16_t head_num           ; ///< Number of heads in storage device.
  uint32_t not_used1          ; ///< Number of sectors before the start of partition.
  uint32_t not_used2          ; ///< 32-bit value of number of sectors in file system. Either this value or the 16-bit value above must be 0.

  uint8_t  drive_number       ; ///< Physical drive number (0x00 for (first) removable media, 0x80 for (first) fixed disk
  uint8_t  not_used3          ;
  uint8_t  ext_boot_signature ; ///< should be 0x29
  uint32_t volume_id          ; ///< Volume serial number, which some versions of Windows will calculate based on the creation date and time.
  uint8_t  volume_label[11]   ;
  uint8_t  fs_type[8]         ; ///< File system type label in ASCII, padded with blank (0x20). Standard values include "FAT," "FAT12," and "FAT16," but nothing is required.
  uint8_t  not_used4[448]     ;
  uint16_t signature          ; ///< Signature value (0xAA55).
}fat12_boot_sector_t;

VERIFY_STATIC(sizeof(fat12_boot_sector_t) == 512, "size is not correct");

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

static inline void set16(void* ptr, uint16_t value)
{
  memcpy(ptr, &value, 2);
}

static inline void set32(void* ptr, uint32_t value)
{
  memcpy(ptr, &value, 4);
}


static inline bool fat12_formatted(void)
{
  const fat12_boot_sector_t* boot_sect = (fat12_boot_sector_t* ) lba2addr(0);
  return boot_sect->signature == BOOTSECT_SIGNATURE;
}

static void fat12_write_cluster(uint16_t cluster_num, uint8_t const* buf, uint32_t bufsize)
{
  uint32_t addr = lba2addr(cluster_num*8);

  nrf_nvmc_page_erase( addr );
  nrf_nvmc_write_words(addr, (uint32_t const*) buf, bufsize/4);
}

static void fat12_mkfs(void)
{
  memclr_(_page_cached, sizeof(_page_cached));

  /*------------- Boot Sector cluster -------------*/
  fat12_boot_sector_t* boot_sect = (fat12_boot_sector_t*) _page_cached;

  memcpy(boot_sect->jump_code, "\xEB\xFE\x90", 3);
  memcpy(boot_sect->oem_name, "MSDOS5.0", 8);

  set16(&boot_sect->byte_per_sector, MSC_FLASH_BLOCK_SIZE);
  boot_sect->sector_per_cluster      = MSC_FLASH_CLUSTER_SIZE/MSC_FLASH_BLOCK_SIZE;
  set16(&boot_sect->reserved_sectors, 1);

  boot_sect->fat_num                 = 1;
  set16(&boot_sect->root_entry_count, 512);
  set16(&boot_sect->sector_count, MSC_FLASH_BLOCK_NUM);

  boot_sect->media_type              = 0xf8; // fixed disk
  set16(&boot_sect->sector_per_fat, 1);
  set16(&boot_sect->sector_per_track, 63);
  set16(&boot_sect->head_num, 255);

  boot_sect->drive_number            = 0x80;
  boot_sect->ext_boot_signature = 0x29;
  set32(&boot_sect->volume_id, tusb_hal_millis()); // typically date + time
  memcpy(boot_sect->volume_label   , MSC_FLASH_VOL_LABEL, 11);
  memcpy(boot_sect->fs_type,  "FAT12   ", 8);
  set16(&boot_sect->signature, BOOTSECT_SIGNATURE);

  // Erase and Write cluster.
  fat12_write_cluster(0, _page_cached, FL_PAGE_SIZE);

  //------------- FAT12 Table cluster -------------//
  // first 2 entries are F8FF, third entry is cluster end of readme file
  memclr_(_page_cached, sizeof(_page_cached));
  memcpy(_page_cached, "\xF8\xFF\xFF\xFF\x0F", 5);

  // Erase and Write cluster.
  fat12_write_cluster(1, _page_cached, FL_PAGE_SIZE);

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
  fat12_write_cluster(2, _page_cached, FL_PAGE_SIZE);

  //------------- Readme Content cluster -------------//
  memclr_(_page_cached, sizeof(_page_cached));
  memcpy(_page_cached, readme_contents, sizeof(readme_contents)-1);

  // Erase and Write cluster.
  fat12_write_cluster(3, _page_cached, FL_PAGE_SIZE);
}

#if 0
CFG_TUSB_ATTR_USBRAM
uint8_t msc_device_ramdisk[DISK_BLOCK_NUM][DISK_BLOCK_SIZE] =
{
  //------------- Boot Sector -------------//
  // byte_per_sector    = DISK_BLOCK_SIZE; fat12_sector_num_16  = DISK_BLOCK_NUM;
  // sector_per_cluster = 1; reserved_sectors = 1;
  // fat_num            = 1; fat12_root_entry_num = 16;
  // sector_per_fat     = 1; sector_per_track = 1; head_num = 1; hidden_sectors = 0;
  // drive_number       = 0x80; media_type = 0xf8; extended_boot_signature = 0x29;
  // filesystem_type    = "FAT12   "; volume_serial_number = 0x1234; volume_label = "tinyusb msc";
  [0] =
  {
      0xEB, 0x3C, 0x90, 0x4D, 0x53, 0x44, 0x4F, 0x53, 0x35, 0x2E, 0x30, 0x00, 0x02, 0x01, 0x01, 0x00,
      0x01, 0x10, 0x00, 0x10, 0x00, 0xF8, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x29, 0x34, 0x12, 0x00, 0x00, 0x74, 0x69, 0x6E, 0x79, 0x75,
      0x73, 0x62, 0x20, 0x6D, 0x73, 0x63, 0x46, 0x41, 0x54, 0x31, 0x32, 0x20, 0x20, 0x20, 0x00, 0x00,
      [510] = 0x55, [511] = 0xAA // FAT magic code
  },

  //------------- FAT12 Table -------------//
  [1] =
  {
      0xF8, 0xFF, 0xFF, 0xFF, 0x0F // // first 2 entries must be F8FF, third entry is cluster end of readme file
  },

  //------------- Root Directory -------------//
  [2] =
  {
      // first entry is volume label
      0x54, 0x49, 0x4E, 0x59, 0x55, 0x53, 0x42, 0x20, 0x4D, 0x53, 0x43, 0x08, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x6D, 0x65, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      // second entry is readme file
      'R' , 'E' , 'A' , 'D' , 'M' , 'E' , ' ' , ' ' , 'T' , 'X' , 'T' , 0x20, 0x00, 0xC6, 0x52, 0x6D,
      0x65, 0x43, 0x65, 0x43, 0x00, 0x00, 0x88, 0x6D, 0x65, 0x43, 0x02, 0x00,
      sizeof(README_CONTENTS)-1, 0x00, 0x00, 0x00 // readme's filesize (4 Bytes)
  },

  //------------- Readme Content -------------//
  [3] = README_CONTENTS
};
#endif

#endif
