#include "boards.h"

#ifdef BOOTLOADER_CONFIG_CF2

__attribute__((used, section(".bootloaderConfig")))
const uint32_t bootloaderConfig[] = {
  BOOTLOADER_CONFIG_CF2
};

#endif
