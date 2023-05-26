#include "boards.h"
#include "board.h"
#include "uf2/configkeys.h"

__attribute__((used, section(".bootloaderConfig")))
const uint32_t bootloaderConfig[] =
{
  /* CF2 START */
  CFG_MAGIC0, CFG_MAGIC1,                       // magic
  5, 100,                                       // used entries, total entries

  204, 0x100000,                                // FLASH_BYTES = 0x100000
  205, 0x40000,                                 // RAM_BYTES = 0x40000
  208, (USB_DESC_VID << 16) | USB_DESC_UF2_PID, // BOOTLOADER_BOARD_ID = USB VID+PID, used for verification when updating bootloader via uf2
  209, 0xada52840,                              // UF2_FAMILY = 0xada52840
  210, 0x20,                                    // PINS_PORT_SIZE = PA_32

  0, 0, 0, 0, 0, 0, 0, 0
  /* CF2 END */
};

static void power_on(void)
{
  // Turn on the regulator
  nrf_gpio_cfg_output(LDO_CONTROL_PIN);
  nrf_gpio_pin_write(LDO_CONTROL_PIN, 1);
}

void board_init2(void)
{
  power_on();

  // configure P0.05 for ESC/BUTTON_2
  // P0.05 --- |<- --- / --- P0.19
  //         diode  sw(esc)
  // mode: output, push-pull, low
  nrf_gpio_cfg(
    _PINNUM(0, 5),
    NRF_GPIO_PIN_DIR_OUTPUT,
    NRF_GPIO_PIN_INPUT_DISCONNECT,
    NRF_GPIO_PIN_NOPULL,
    NRF_GPIO_PIN_H0D1,
    NRF_GPIO_PIN_NOSENSE
  );
  nrf_gpio_pin_write(_PINNUM(0, 5), 0);

  // Wait the buttons stable.
  // This is mandatory, or the keyboard will enter bootloader whenever
  // booted by pressing the button at back (same with BUTTON_1)
  NRFX_DELAY_MS(300);
}

void board_teardown2(void)
{
  // re-enable the battery
  power_on();
}