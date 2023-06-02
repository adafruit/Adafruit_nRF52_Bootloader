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
#include "uf2_block_flip.h"

void usb_init(bool cdc_only);
void usb_teardown(void);

bool _ota_dfu = false;

__attribute__ ((section(".uf2_flip_data")))
uf2_message_item_t uf2_message_item __attribute__((aligned(4)));

bool is_ota(void)
{
  return _ota_dfu;
}

static void check_dfu_mode(void)
{
  bool dfu_start;

  PRINTF("Check_dfu_mode\n\r");

  /* Check if DFU button pressed */
  dfu_start = button_pressed(BUTTON_DFU);
  NRFX_DELAY_US(8000);

  if(dfu_start && button_pressed(BUTTON_DFU))
  {
    /* Enter DFU mode */
    led_control(LED_DFU_APP, LED_STATE_ON);
    dfu_start = true;
    PRINTF("Enter DFU bootloader \n\r");
    usb_init(false);
    bootloader_dfu_start(false, 0, false);    
    /* Exit DFU mode */
    usb_teardown();
    led_control(LED_DFU_APP, LED_STATE_OFF);
  }
  else
  {
    PRINTF("Exit DFU bootloader \n\r");
  }
}

static int grant_periph_access_to_net_core(void)
{
  NRF_P0_S->PIN_CNF[23] = (GPIO_PIN_CNF_MCUSEL_AppMCU << GPIO_PIN_CNF_MCUSEL_Pos); /* bootloader button */
  NRF_P0_S->PIN_CNF[28] = (GPIO_PIN_CNF_MCUSEL_AppMCU << GPIO_PIN_CNF_MCUSEL_Pos);
  NRF_P0_S->PIN_CNF[30] = (GPIO_PIN_CNF_MCUSEL_AppMCU << GPIO_PIN_CNF_MCUSEL_Pos);

  NRF_P0_S->PIN_CNF[29] = (GPIO_PIN_CNF_MCUSEL_NetworkMCU << GPIO_PIN_CNF_MCUSEL_Pos);
  NRF_P0_S->PIN_CNF[31] = (GPIO_PIN_CNF_MCUSEL_NetworkMCU << GPIO_PIN_CNF_MCUSEL_Pos);

  NRF_SPU->EXTDOMAIN[0].PERM = 1 << 4;
}

static void start_net_core(void)
{
  /* Start networking core, 'Release force off signal' */
  NRF_RESET->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Release;
}

static void stop_net_core(void)
{
  /* Stop networking core */
  NRF_RESET->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Hold;
}


int main(void)
{
  bool loop = true;

  stop_net_core();
  grant_periph_access_to_net_core();
  set_net_bootloader_state(NET_IDLE_REQ);

  while(loop)
  {
    start_net_core();
    INTERCORE_MUTEX_LOCK;

    /* Make sure network core is in idle state */
    if (NET_IDLE == get_net_bootloader_state())
    {
      loop = false;
    }
    INTERCORE_MUTEX_UNLOCK;
  }

  board_init();

  bootloader_init();

  PRINTF("Bootloader Start \n\r");

  /* Check inputs and enter DFU if needed */
  check_dfu_mode();

  loop = true;
  while(loop)
  {
    INTERCORE_MUTEX_LOCK;

    if ((NET_APPL_FW_FLASH_FLUSH_DONE == get_net_bootloader_state()) || (NET_IDLE == get_net_bootloader_state()))
    {
      loop = false;
    }
    INTERCORE_MUTEX_UNLOCK;
  }

  set_net_bootloader_state(NET_APPL_START_REQ);

  loop = true;
  while(loop)
  {
    INTERCORE_MUTEX_LOCK;

    if (NET_APPL_START_DONE == get_net_bootloader_state())
    {
      loop = false;
    }
    INTERCORE_MUTEX_UNLOCK;
  }

  NRF_RESET->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Hold;

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

  NVIC_SystemReset();
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
