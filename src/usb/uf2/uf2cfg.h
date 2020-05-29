#include "boards.h"
#include "dfu_types.h"

// Family ID for updating Application
#define CFG_UF2_FAMILY_APP_ID     0xADA52840

// Family ID for updating Bootloader
#define CFG_UF2_FAMILY_BOOT_ID    0xd663823c

#define CFG_UF2_NUM_BLOCKS        0x10109     // just under 32MB
#define CFG_UF2_FLASH_SIZE        (1024*1024) // 1 MB

// Application Address Space
#define USER_FLASH_START          MBR_SIZE // skip MBR included in SD hex
#define USER_FLASH_END            0xAD000

// Bootloader start address
#define BOOTLOADER_ADDR_START         BOOTLOADER_REGION_START

// Bootloader end address
#define BOOTLOADER_ADDR_END           BOOTLOADER_MBR_PARAMS_PAGE_ADDRESS

// Address where new bootloader is written before activation (skip application data)
#define BOOTLOADER_ADDR_NEW_RECIEVED  (USER_FLASH_END-DFU_BL_IMAGE_MAX_SIZE)
