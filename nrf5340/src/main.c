/**
 * Program downloads uf2 binary by opening mass storage device.
 * ota - disabled
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "bootloader.h"
#include "bootloader_util.h"

#include "boards.h"

void usb_init(bool cdc_only);
void usb_teardown(void);

// tinyusb function that handles power event (detected, ready, removed)
// We must call it within SD's SOC event handler, or set it as power event handler if SD is not enabled.
extern void tusb_hal_nrf_power_event(uint32_t event);

bool _ota_dfu = false;

bool is_ota(void)
{
  return _ota_dfu;
}

static void check_dfu_mode(void);


int main(void)
{
  // Populate Boot Address and MBR Param into MBR if not already
  // MBR_BOOTLOADER_ADDR/MBR_PARAM_PAGE_ADDR are used if available, else UICR registers are used
  // Note: skip it for now since this will prevent us to change the size of bootloader in the future
  //bootloader_mbr_addrs_populate();

  board_init();

  led_control(LED_DFU, LED_STATE_ON);
 
  bootloader_init();

  PRINTF("Bootloader Start \n\r");

  /* Check inputs and enter DFU if needed */
  check_dfu_mode();

  /* Reset peripherals */
  board_teardown();

  /* Jump to application if valid */
  if (bootloader_app_is_valid())
  {
    PRINTF("App is valid\r\n");

    /* start application */
    bootloader_app_start();
  }
  else
  {
    PRINTF("App NOT valid exit bootloader\n\r");
  }

  while(1){}
  NVIC_SystemReset();
}

static void check_dfu_mode(void)
{
  bool dfu_start;

  PRINTF("Check_dfu_mode\n\r");

  /* ------------- Check if DFU button pressed --------------*/
  dfu_start = button_pressed(BUTTON_DFU);
  NRFX_DELAY_US(8000);

  if(dfu_start && button_pressed(BUTTON_DFU))
  {
    /* Enter DFU mode */
    dfu_start = true;
    PRINTF("Enter DFU bootloader \n\r");
    usb_init(false);
    bootloader_dfu_start(false, 0, false);
    /* Exit DFU mode */
    usb_teardown();
  }
  else
  {
    PRINTF("Exit DFU bootloader \n\r");
  }
}

//--------------------------------------------------------------------+
// Error Handler
//--------------------------------------------------------------------+
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
  volatile uint32_t* ARM_CM_DHCSR =  ((volatile uint32_t*) 0xE000EDF0UL); /* Cortex M CoreDebug->DHCSR */
  if ( (*ARM_CM_DHCSR) & 1UL ) __asm("BKPT #0\n"); /* Only halt mcu if debugger is attached */
  NVIC_SystemReset();
}

void assert_nrf_callback (uint16_t line_num, uint8_t const * p_file_name)
{
  app_error_fault_handler(0xDEADBEEF, 0, 0);
}

//--------------------------------------------------------------------+
// RTT printf retarget for Debug
//--------------------------------------------------------------------+
#ifdef CFG_DEBUG

#include "SEGGER_RTT.h"

__attribute__ ((used))
int _write (int fhdl, const void *buf, size_t count)
{
  (void) fhdl;
  SEGGER_RTT_Write(0, (char*) buf, (int) count);
  return count;
}

__attribute__ ((used))
int _read (int fhdl, char *buf, size_t count)
{
  (void) fhdl;
  return SEGGER_RTT_Read(0, buf, count);
}

#endif
