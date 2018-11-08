/**************************************************************************/
/*!
    @file     boards.c
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2018, Adafruit Industries (adafruit.com)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#include "boards.h"
#include "app_scheduler.h"
#include "app_timer.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
#define SCHED_MAX_EVENT_DATA_SIZE           sizeof(app_timer_event_t)        /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                    30                               /**< Maximum number of events in the scheduler queue. */

/* use PWM for blinky to prevent inconsistency due to MCU blocking in flash operation
 * clock = 125khz --> resolution = 8us
 * top value = 25000 -> period = 200 ms
 * Mode up -> toggle every 100 ms = fast blink
 * Mode up and down = 400 ms = slow blink
 */
#define PWM_MAXCOUNT      25000
#define PWM_CHANNEL_NUM   4


uint16_t _pwm_red_seq [PWM_CHANNEL_NUM] = { PWM_MAXCOUNT/2, 0, 0 , 0 };
uint16_t _pwm_blue_seq[PWM_CHANNEL_NUM] = { PWM_MAXCOUNT/2, 0, 0 , 0 };

//------------- IMPLEMENTATION -------------//

void board_init(void)
{
  // stop LF clock just in case we jump from application without reset
  NRF_CLOCK->TASKS_LFCLKSTOP = 1UL;

  // Use Internal OSC to compatible with all boards
  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_RC;
  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;

  // stop WDT if started by application, when jumping from application using BLE DFU
  if ( NRF_WDT->RUNSTATUS )
  {
    NRF_WDT->TASKS_START = 0;
  }

  button_init(BUTTON_DFU);
  button_init(BUTTON_FRESET);
  NRFX_DELAY_US(100); // wait for the pin state is stable

  // LED init
  nrf_gpio_cfg_output(LED_RED);
  nrf_gpio_cfg_output(LED_BLUE);
  led_off(LED_RED);
  led_off(LED_BLUE);

  // use PMW0 for LED RED
  led_pwm_init(LED_RED);

  // Init scheduler
  APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

  // Init app timer (use RTC1)
  app_timer_init();
}

void board_teardown(void)
{
  // Disable and reset PWM for LED
  led_pwm_teardown(LED_RED);

  led_off(LED_BLUE);
  led_off(LED_RED);

  // Button

  // Stop RTC1 used by app_timer
  NVIC_DisableIRQ(RTC1_IRQn);
  NRF_RTC1->EVTENCLR    = RTC_EVTEN_COMPARE0_Msk;
  NRF_RTC1->INTENCLR    = RTC_INTENSET_COMPARE0_Msk;
  NRF_RTC1->TASKS_STOP  = 1;
  NRF_RTC1->TASKS_CLEAR = 1;

  // Stop LF clock
  NRF_CLOCK->TASKS_LFCLKSTOP = 1UL;
}

uint32_t tusb_hal_millis(void)
{
  return ( ( ((uint64_t)app_timer_cnt_get())*1000*(APP_TIMER_CONFIG_RTC_FREQUENCY+1)) / APP_TIMER_CLOCK_FREQ );
}

void led_pwm_init(uint32_t led_pin)
{
  NRF_PWM_Type* pwm    = (led_pin == LED_RED) ? NRF_PWM0 : NRF_PWM1;

  pwm->MODE            = PWM_MODE_UPDOWN_UpAndDown;
  pwm->COUNTERTOP      = PWM_MAXCOUNT;
  pwm->PRESCALER       = PWM_PRESCALER_PRESCALER_DIV_128;
  pwm->DECODER         = PWM_DECODER_LOAD_Individual;
  pwm->LOOP            = 0;

  pwm->SEQ[0].PTR      = (uint32_t) (led_pin == LED_RED ? _pwm_red_seq : _pwm_blue_seq);
  pwm->SEQ[0].CNT      = PWM_CHANNEL_NUM; // default mode is Individual --> count must be 4
  pwm->SEQ[0].REFRESH  = 0;
  pwm->SEQ[0].ENDDELAY = 0;

  pwm->PSEL.OUT[0] = led_pin;

  pwm->ENABLE = 1;
  pwm->TASKS_SEQSTART[0] = 1;
}

void led_pwm_teardown(uint32_t led_pin)
{
  NRF_PWM_Type* pwm = (led_pin == LED_RED) ? NRF_PWM0 : NRF_PWM1;

  pwm->TASKS_SEQSTART[0] = 0;
  pwm->ENABLE            = 0;

  pwm->PSEL.OUT[0] = 0xFFFFFFFF;

  pwm->MODE        = 0;
  pwm->COUNTERTOP  = 0x3FF;
  pwm->PRESCALER   = 0;
  pwm->DECODER     = 0;
  pwm->LOOP        = 0;
  pwm->SEQ[0].PTR  = 0;
  pwm->SEQ[0].CNT  = 0;
}

void led_pwm_disable(uint32_t led_pin)
{
  NRF_PWM_Type* pwm = (led_pin == LED_RED) ? NRF_PWM0 : NRF_PWM1;

  pwm->TASKS_SEQSTART[0] = 0;
  pwm->ENABLE = 0;
}

void led_pwm_enable(uint32_t led_pin)
{
  NRF_PWM_Type* pwm = (led_pin == LED_RED) ? NRF_PWM0 : NRF_PWM1;

  pwm->ENABLE = 1;
  pwm->TASKS_SEQSTART[0] = 1;
}


void led_blink_fast(bool enable)
{
  if ( enable )
  {
    NRF_PWM0->MODE = PWM_MODE_UPDOWN_Up;
  }else
  {
    NRF_PWM0->MODE = PWM_MODE_UPDOWN_UpAndDown;
  }
}

