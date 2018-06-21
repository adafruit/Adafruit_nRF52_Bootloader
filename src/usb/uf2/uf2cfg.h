#define UF2_VERSION "1.00"
#define PRODUCT_NAME "Adafruit Bluefruit nRF52"
#define BOARD_ID "NRF52-Bluefruit-v0"
#define INDEX_URL "https://www.adafruit.com/product/0000"
#define UF2_NUM_BLOCKS 8000
#define VOLUME_LABEL "NRF52BOOT"
#define FLASH_SIZE (BOOTLOADER_REGION_START-USER_FLASH_START)

// Only allow to write application
#define USER_FLASH_START 0x26000
#define USER_FLASH_END BOOTLOADER_REGION_START

#define FLASH_PAGE_SIZE 4096
