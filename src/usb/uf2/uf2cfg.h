#include "boards.h"
#include "dfu_types.h"

#define CFG_UF2_FAMILY_ID               0xADA52840
#define CFG_UF2_BOOTLOADER_ID           ((USB_DESC_VID << 16) | USB_DESC_UF2_PID)

#define CFG_UF2_NUM_BLOCKS              8000        // at least 4,1 MB for FAT16
#define CFG_UF2_FLASH_SIZE              (1024*1024) // 1 MB

// Softdevice Address Space
#define CFG_UF2_SOFTDEVICE_ADDR_START   0
#define CFG_UF2_SOFTDEVICE_ADDR_END     USER_FLASH_START

// Application Address Space
#ifdef SOFTDEVICE_PRESENT
  #define USER_FLASH_START   (SD_FLASH_SIZE + MBR_SIZE)
#else
  #define USER_FLASH_START   0x1000 // MBR is still required
#endif

#define USER_FLASH_END                  0xAD000

// Bootloader Address Space
#define CFG_UF2_BOOTLOADER_ADDR_START   BOOTLOADER_REGION_START
#define CFG_UF2_BOOTLOADER_ADDR_END     BOOTLOADER_MBR_PARAMS_PAGE_ADDRESS
