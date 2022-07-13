#include "boards.h"
#include "dfu_types.h"

// Virtual disk size: just under 32MB
#define CFG_UF2_NUM_BLOCKS            0x10109

// Family ID for updating Bootloader
#define CFG_UF2_FAMILY_BOOT_ID        0xd663823c

// Board-specific ID for board-specific Application
#if defined(USB_DESC_VID) && defined(USB_DESC_UF2_PID) && USB_DESC_VID && USB_DESC_UF2_PID
    #define CFG_UF2_BOARD_APP_ID      ((USB_DESC_VID << 16) | USB_DESC_UF2_PID)
#endif

// Family ID and size for updating generic Application
#if defined(NRF52840_XXAA)
  #define CFG_UF2_FAMILY_APP_ID       0xADA52840
  #define CFG_UF2_FLASH_SIZE          (1024*1024) // 1 MB
#elif defined(NRF52833_XXAA)
  #define CFG_UF2_FAMILY_APP_ID       0x621E937A
  #define CFG_UF2_FLASH_SIZE          (512*1024)  // 512 kB
#endif

// Application Address Space
#define USER_FLASH_START              MBR_SIZE // skip MBR included in SD hex
#define USER_FLASH_END                (BOOTLOADER_REGION_START - DFU_APP_DATA_RESERVED)

// Bootloader start address
#define BOOTLOADER_ADDR_START         BOOTLOADER_REGION_START

// Bootloader end address
#define BOOTLOADER_ADDR_END           BOOTLOADER_MBR_PARAMS_PAGE_ADDRESS

// Address where new bootloader is written before activation (skip application data)
#define BOOTLOADER_ADDR_NEW_RECIEVED  (USER_FLASH_END-DFU_BL_IMAGE_MAX_SIZE)
