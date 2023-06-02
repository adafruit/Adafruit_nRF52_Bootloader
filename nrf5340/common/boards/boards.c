/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ha Thach for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "boards.h"
#include "app_scheduler.h"
#include "app_timer.h"


//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
#define SCHED_MAX_EVENT_DATA_SIZE           sizeof(app_timer_event_t)        /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                    30                               /**< Maximum number of events in the scheduler queue. */

void button_init(uint32_t pin)
{
  if ( BUTTON_PULL == NRF_GPIO_PIN_PULLDOWN )
  {
    nrf_gpio_cfg_sense_input(pin, BUTTON_PULL, NRF_GPIO_PIN_SENSE_HIGH);
  }
  else
  {
    nrf_gpio_cfg_sense_input(pin, BUTTON_PULL, NRF_GPIO_PIN_SENSE_LOW);
  }
}

bool button_pressed(uint32_t pin)
{
  uint32_t const active_state = (BUTTON_PULL == NRF_GPIO_PIN_PULLDOWN ? 1 : 0);
  return nrf_gpio_pin_read(pin) == active_state;
}

void led_init(uint32_t led_pin)
{
  nrf_gpio_cfg_output(led_pin);
  nrf_gpio_pin_write(led_pin, LED_STATE_OFF);
}

void led_control(uint32_t pin_number, uint32_t value)
{
  nrf_gpio_pin_write(pin_number, value);
}

// This is declared so that a board specific init can be called from here.
void __attribute__((weak)) board_init_extra(void) { }
void board_init(void)
{
  // stop LF clock just in case we jump from application without reset
  NRF_CLOCK->TASKS_LFCLKSTOP = 1UL;

  // Use Internal OSC to compatible with all boards
  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_LFRC;
  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;

#if defined(NRF5340_XXAA_APPLICATION) 
  button_init(BUTTON_DFU);

  led_init(LED_DFU_APP);
#else
  led_init(LED_DFU_NET);
#endif
  NRFX_DELAY_US(100); // wait for the pin state is stable



#if ENABLE_DCDC_0 == 1
  NRF_POWER->DCDCEN0 = 1;
#endif
#if ENABLE_DCDC_1 == 1
  NRF_POWER->DCDCEN = 1;
#endif
  // Make sure any custom inits are performed
  //board_init_extra();

// When board is supplied on VDDH (and not VDD), this specifies what voltage the GPIO should run at
// and what voltage is output at VDD. The default (0xffffffff) is 1.8V; typically you'll want
//     #define UICR_REGOUT0_VALUE UICR_REGOUT0_VOUT_3V3
// in board.h when using that power configuration.
#ifdef UICR_REGOUT0_VALUE
  if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) !=
      (UICR_REGOUT0_VALUE << UICR_REGOUT0_VOUT_Pos))
  {
      NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
      NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
                          (UICR_REGOUT0_VALUE << UICR_REGOUT0_VOUT_Pos);

      NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

      NVIC_SystemReset();
  }
#endif

  // Init scheduler
  APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

  // Init app timer (use RTC1)
  app_timer_init();

  // Configure Systick for led blinky
  NVIC_SetPriority(SysTick_IRQn, 7);
  SysTick_Config(SystemCoreClock/1000);
}

void board_teardown(void)
{
  // Disable systick, turn off LEDs
  SysTick->CTRL = 0;


  // Stop RTC1 used by app_timer
  NVIC_DisableIRQ(RTC1_IRQn);
  NRF_RTC1->EVTENCLR    = RTC_EVTEN_COMPARE0_Msk;
  NRF_RTC1->INTENCLR    = RTC_INTENSET_COMPARE0_Msk;
  NRF_RTC1->TASKS_STOP  = 1;
  NRF_RTC1->TASKS_CLEAR = 1;

  // Stop LF clock
  NRF_CLOCK->TASKS_LFCLKSTOP = 1UL;

  // make sure all pins are back in reset state
  // NUMBER_OF_PINS is defined in nrf_gpio.h
  for (int i = 0; i < NUMBER_OF_PINS; ++i)
  {
    nrf_gpio_cfg_default(i);
  }
}

static uint32_t _systick_count = 0;
void SysTick_Handler(void)
{
  _systick_count++;

  led_tick();
}




void led_tick()
{

}

void led_state(uint32_t state)
{

}

