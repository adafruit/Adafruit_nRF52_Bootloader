/**
 * Program downloads uf2 binary sent by application core via shared RAM.
 * ota - disabled
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "bootloader.h"
#include "boards.h"
#include "uf2_block_flip.h"
#include "flash_nrf5x.h"

bool _ota_dfu = false;

__attribute__ ((section(".uf2_flip_data")))
uf2_message_item_t uf2_message_item __attribute__((aligned(4)));

bool is_ota(void)
{
  return _ota_dfu;
}

bool is_bootloader_app_valid(void)
{
  uint32_t const app_addr = APP_START_ADDRESS;

  enum { EMPTY_FLASH = 0xFFFFFFFFUL };

  /* Application is invalid if first 2 words are all 0xFFFFFFF */
  if ( *((uint32_t *)app_addr    ) == EMPTY_FLASH &&
       *((uint32_t *)(app_addr+4)) == EMPTY_FLASH )
  {
    return false;
  }
  else
  {
    return true;
  }
}

int main(void)
{
  int loop = 1;

  board_init();

  led_control(LED_DFU_NET, LED_STATE_ON);

  while (loop)
  {
    INTERCORE_MUTEX_LOCK;

    switch (get_net_bootloader_state())
    {
      case NET_IDLE_REQ:
        uf2_message_item.net_bootloader_state = NET_IDLE;

      case NET_IDLE:
        /* Do nothing */
        break;

      case NET_APPL_FW_FLASH_REQ:
        /* Flash received FW block */
        get_data_from_flip_section(NULL);
        uf2_message_item.net_bootloader_state = NET_APPL_FW_FLASH_DONE;
      case NET_APPL_FW_FLASH_DONE:
        break;

      case NET_APPL_FW_FLASH_FLUSH_REQ:
        /* Flush memory */
        flash_nrf5x_flush(true);
        uf2_message_item.net_bootloader_state = NET_APPL_FW_FLASH_FLUSH_DONE;
      case NET_APPL_FW_FLASH_FLUSH_DONE:
        break;

      case NET_APPL_START_REQ:
      case NET_APPL_START_DONE:
        led_control(LED_DFU_NET, LED_STATE_OFF);
        /* Do nothing */
        PRINTF("CPU1 starting appl\n\r");
        loop = 0;
        uf2_message_item.net_bootloader_state = NET_APPL_START_DONE;
        break;

      default:
         break;
    }

    INTERCORE_MUTEX_UNLOCK;
  }

  /* Reset peripherals */
  board_teardown();

  /* Jump to application if valid */
  if (is_bootloader_app_valid())
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
  SEGGER_RTT_Write(1, (char*) buf, (int) count);
  return count;
}

__attribute__ ((used))
int _read (int fhdl, char *buf, size_t count)
{
  (void) fhdl;
  return SEGGER_RTT_Read(1, buf, count);
}

#endif
