#include "boards.h"

#define UF2_NUM_BLOCKS        8000   // at least 4,1 MB for FAT16
#define FLASH_SIZE            (USER_FLASH_END-USER_FLASH_START) // Max flash size

// Only allow to write application TODO dynamic depending on SD size
#define USER_FLASH_START      (SD_FLASH_SIZE + MBR_SIZE)
#define USER_FLASH_END        0xAD000 // Fat Fs start here

#define CFG_UF2_FAMILY_ID     0xADA52840
#define CFG_UF2_BOOTLOADER_ID ((USB_DESC_VID << 16) | USB_DESC_UF2_PID)
