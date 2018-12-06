#include "boards.h"

#ifndef PRODUCT_NAME
  #define PRODUCT_NAME     DIS_MODEL
#endif

#ifndef BOARD_ID
#define BOARD_ID           "NRF52-Bluefruit-v0"
#endif

#ifndef INDEX_URL
#define INDEX_URL          "https://www.adafruit.com/"
#endif

#define BOOTLOADER_ID      MK_DIS_FIRMWARE

#define UF2_NUM_BLOCKS     8000   // at least 4,1 MB for FAT16

#ifndef VOLUME_LABEL
#define VOLUME_LABEL       "NRF52BOOT  "
#endif

#define FLASH_SIZE         (USER_FLASH_END-USER_FLASH_START) // Max flash size

// Only allow to write application TODO dynamic depending on SD size
#define USER_FLASH_START   0x26000
#define USER_FLASH_END     0xAD000 // Fat Fs start here

#define FLASH_PAGE_SIZE    4096

#define UF2_FAMILY_ID      0xADA52840
