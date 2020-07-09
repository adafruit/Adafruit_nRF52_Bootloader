#include "boards.h"
#include "uf2/configkeys.h"

__attribute__((used, section(".bootloaderConfig")))
const uint32_t bootloaderConfig[] =
{
  /* CF2 START */
    CFG_MAGIC0, CFG_MAGIC1, // magic
    31, 100,  // used entries, total entries
    4, 0x1d, // PIN_BTN_A = PA29
    7, 0x29, // PIN_DOTSTAR_CLOCK = PB09
    8, 0x8, // PIN_DOTSTAR_DATA = PA08
    13, 0x6, // PIN_LED = PA06
    18, 0x14, // PIN_MISO = PA20
    19, 0xf, // PIN_MOSI = PA15
    21, 0x19, // PIN_RX = PA25
    23, 0xd, // PIN_SCK = PA13
    24, 0xe, // PIN_SCL = PA14
    25, 0x10, // PIN_SDA = PA16
    28, 0x18, // PIN_TX = PA24
    100, 0x4, // PIN_A0 = PA04
    101, 0x1e, // PIN_A1 = PA30
    102, 0x1c, // PIN_A2 = PA28
    103, 0x1f, // PIN_A3 = PA31
    104, 0x2, // PIN_A4 = PA02
    105, 0x3, // PIN_A5 = PA03
    152, 0x22, // PIN_D2 = PB02
    155, 0x1b, // PIN_D5 = PA27
    157, 0x28, // PIN_D7 = PB08
    159, 0x7, // PIN_D9 = PA07
    160, 0x5, // PIN_D10 = PA05
    161, 0x1a, // PIN_D11 = PA26
    162, 0xb, // PIN_D12 = PA11
    163, 0xc, // PIN_D13 = PA12
    201, 0x1, // NUM_DOTSTARS = 1
    204, 0x100000, // FLASH_BYTES = 0x100000
    205, 0x40000, // RAM_BYTES = 0x40000
    208, (USB_DESC_VID << 16) | USB_DESC_UF2_PID, // BOOTLOADER_BOARD_ID = USB VID+PID, used for verification when updating bootloader via uf2
    209, 0xada52840, // UF2_FAMILY = 0xada52840
    210, 0x20, // PINS_PORT_SIZE = PA_32
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  /* CF2 END */
};
