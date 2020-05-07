#include "boards.h"
#include "dfu_types.h"

#define CFG_UF2_FAMILY_ID               0xADA52840
#define CFG_UF2_BOOTLOADER_ID           ((USB_DESC_VID << 16) | USB_DESC_UF2_PID)

#define CFG_UF2_NUM_BLOCKS              8000        // at least 4,1 MB for FAT16
#define CFG_UF2_FLASH_SIZE              (1024*1024) // 1 MB

// Application Address Space
#define USER_FLASH_START                MBR_SIZE // skip MBR included in SD hex
#define USER_FLASH_END                  0xAD000

// Bootloader Address Space
#define CFG_UF2_BOOTLOADER_ADDR_START   BOOTLOADER_REGION_START
#define CFG_UF2_BOOTLOADER_ADDR_END     BOOTLOADER_MBR_PARAMS_PAGE_ADDRESS
