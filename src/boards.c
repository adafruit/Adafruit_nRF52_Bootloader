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
#include "nrf_pwm.h"
#include "app_scheduler.h"
#include "app_timer.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
#define SCHED_MAX_EVENT_DATA_SIZE           sizeof(app_timer_event_t)        /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                    30                               /**< Maximum number of events in the scheduler queue. */

#if defined(LED_NEOPIXEL) || defined(LED_RGB_RED_PIN)
  void neopixel_init(void);
  void neopixel_write(uint8_t *pixels);
  void neopixel_teardown(void);
#endif

//------------- IMPLEMENTATION -------------//
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

void board_init(void)
{
  // stop LF clock just in case we jump from application without reset
  NRF_CLOCK->TASKS_LFCLKSTOP = 1UL;

  // Use Internal OSC to compatible with all boards
  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_RC;
  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;

  button_init(BUTTON_DFU);
  button_init(BUTTON_FRESET);
  NRFX_DELAY_US(100); // wait for the pin state is stable

  // use PMW0 for LED RED
  led_pwm_init(LED_PRIMARY, LED_PRIMARY_PIN);
  #if LEDS_NUMBER > 1
  led_pwm_init(LED_SECONDARY, LED_SECONDARY_PIN);
  #endif

  // use neopixel for use enumeration
#if defined(LED_NEOPIXEL) || defined(LED_RGB_RED_PIN)
  neopixel_init();
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

  // Disable and reset PWM for LEDs
  led_pwm_teardown();

#if defined(LED_NEOPIXEL) || defined(LED_RGB_RED_PIN)
  neopixel_teardown();
#endif
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

static uint32_t _systick_count = 0;
void SysTick_Handler(void)
{
  _systick_count++;

  led_tick();
}


uint32_t tusb_hal_millis(void)
{
  return ( ( ((uint64_t)app_timer_cnt_get())*1000*(APP_TIMER_CONFIG_RTC_FREQUENCY+1)) / APP_TIMER_CLOCK_FREQ );
}

void pwm_teardown(NRF_PWM_Type* pwm )
{
  pwm->TASKS_SEQSTART[0] = 0;
  pwm->ENABLE            = 0;

  pwm->PSEL.OUT[0] = 0xFFFFFFFF;
  pwm->PSEL.OUT[1] = 0xFFFFFFFF;
  pwm->PSEL.OUT[2] = 0xFFFFFFFF;
  pwm->PSEL.OUT[3] = 0xFFFFFFFF;

  pwm->MODE        = 0;
  pwm->COUNTERTOP  = 0x3FF;
  pwm->PRESCALER   = 0;
  pwm->DECODER     = 0;
  pwm->LOOP        = 0;
  pwm->SEQ[0].PTR  = 0;
  pwm->SEQ[0].CNT  = 0;
}

static uint16_t led_duty_cycles[PWM0_CH_NUM] = { 0 };

#if LEDS_NUMBER > PWM0_CH_NUM
#error "Only " PWM0_CH_NUM " concurrent status LEDs are supported."
#endif

void led_pwm_init(uint32_t led_index, uint32_t led_pin)
{
  NRF_PWM_Type* pwm    = NRF_PWM0;

  pwm->ENABLE = 0;

  nrf_gpio_cfg_output(led_pin);
  nrf_gpio_pin_write(led_pin, 1 - LED_STATE_ON);

  pwm->PSEL.OUT[led_index] = led_pin;

  pwm->MODE            = PWM_MODE_UPDOWN_Up;
  pwm->COUNTERTOP      = 0xff;
  pwm->PRESCALER       = PWM_PRESCALER_PRESCALER_DIV_16;
  pwm->DECODER         = PWM_DECODER_LOAD_Individual;
  pwm->LOOP            = 0;

  pwm->SEQ[0].PTR      = (uint32_t) (led_duty_cycles);
  pwm->SEQ[0].CNT      = 4; // default mode is Individual --> count must be 4
  pwm->SEQ[0].REFRESH  = 0;
  pwm->SEQ[0].ENDDELAY = 0;

  pwm->ENABLE = 1;

  pwm->EVENTS_SEQEND[0] = 0;
//  pwm->TASKS_SEQSTART[0] = 1;
}

void led_pwm_teardown(void)
{
  pwm_teardown(NRF_PWM0);
}

void led_pwm_duty_cycle(uint32_t led_index, uint16_t duty_cycle)
{
  led_duty_cycles[led_index] = duty_cycle;
  nrf_pwm_event_clear(NRF_PWM0, NRF_PWM_EVENT_SEQEND0);
  nrf_pwm_task_trigger(NRF_PWM0, NRF_PWM_TASK_SEQSTART0);
}

static uint32_t primary_cycle_length;
#ifdef LED_SECONDARY_PIN
static uint32_t secondary_cycle_length;
#endif
void led_tick() {
    uint32_t millis = _systick_count;

    uint32_t cycle = millis % primary_cycle_length;
    uint32_t half_cycle = primary_cycle_length / 2;
    if (cycle > half_cycle) {
        cycle = primary_cycle_length - cycle;
    }
    uint16_t duty_cycle = 0x4f * cycle / half_cycle;
    #if LED_STATE_ON == 1
    duty_cycle = 0xff - duty_cycle;
    #endif
    led_pwm_duty_cycle(LED_PRIMARY, duty_cycle);

    #ifdef LED_SECONDARY_PIN
    cycle = millis % secondary_cycle_length;
    half_cycle = secondary_cycle_length / 2;
    if (cycle > half_cycle) {
        cycle = secondary_cycle_length - cycle;
    }
    duty_cycle = 0x8f * cycle / half_cycle;
    #if LED_STATE_ON == 1
    duty_cycle = 0xff - duty_cycle;
    #endif
    led_pwm_duty_cycle(LED_SECONDARY, duty_cycle);
    #endif
}

static uint32_t rgb_color;
static bool temp_color_active = false;
void led_state(uint32_t state)
{
    uint32_t new_rgb_color = rgb_color;
    uint32_t temp_color = 0;
    switch (state) {
        case STATE_USB_MOUNTED:
          new_rgb_color = 0x00ff00;
          primary_cycle_length = 3000;
          break;

        case STATE_BOOTLOADER_STARTED:
        case STATE_USB_UNMOUNTED:
          new_rgb_color = 0xff0000;
          primary_cycle_length = 300;
          break;

        case STATE_WRITING_STARTED:
          temp_color = 0xff0000;
          primary_cycle_length = 100;
          break;

        case STATE_WRITING_FINISHED:
          // Empty means to unset any temp colors.
          primary_cycle_length = 3000;
          break;

        case STATE_BLE_CONNECTED:
          new_rgb_color = 0x0000ff;
          #ifdef LED_SECONDARY_PIN
          secondary_cycle_length = 3000;
          #else
          primary_cycle_length = 3000;
          #endif
          break;

        case STATE_BLE_DISCONNECTED:
          new_rgb_color = 0xff00ff;
          #ifdef LED_SECONDARY_PIN
          secondary_cycle_length = 300;
          #else
          primary_cycle_length = 300;
          #endif
          break;

        default:
        break;
    }
    uint8_t* final_color = NULL;
    new_rgb_color &= BOARD_RGB_BRIGHTNESS;
    if (temp_color != 0){
        temp_color &= BOARD_RGB_BRIGHTNESS;
        final_color = (uint8_t*)&temp_color;
        temp_color_active = true;
    } else if (new_rgb_color != rgb_color) {
        final_color = (uint8_t*)&new_rgb_color;
        rgb_color = new_rgb_color;
    } else if (temp_color_active) {
        final_color = (uint8_t*)&rgb_color;
    }
    #if LED_NEOPIXEL || defined(LED_RGB_RED_PIN)
    if (final_color != NULL) {
        neopixel_write(final_color);
    }
    #else
    (void) final_color;
    #endif
}

#ifdef LED_NEOPIXEL

// WS2812B (rev B) timing is 0.4 and 0.8 us
#define MAGIC_T0H               6UL | (0x8000) // 0.375us
#define MAGIC_T1H              13UL | (0x8000) // 0.8125us
#define CTOPVAL                20UL            // 1.25us

#define BYTE_PER_PIXEL  3

static uint16_t pixels_pattern[NEOPIXELS_NUMBER*BYTE_PER_PIXEL * 8 + 2];

// use PWM1 for neopixel
void neopixel_init(void)
{
  // To support both the SoftDevice + Neopixels we use the EasyDMA
  // feature from the NRF25. However this technique implies to
  // generate a pattern and store it on the memory. The actual
  // memory used in bytes corresponds to the following formula:
  //              totalMem = numBytes*8*2+(2*2)
  // The two additional bytes at the end are needed to reset the
  // sequence.
  NRF_PWM_Type* pwm = NRF_PWM1;

  // Set the wave mode to count UP
  // Set the PWM to use the 16MHz clock
  // Setting of the maximum count
  // but keeping it on 16Mhz allows for more granularity just
  // in case someone wants to do more fine-tuning of the timing.
  nrf_pwm_configure(pwm, NRF_PWM_CLK_16MHz, NRF_PWM_MODE_UP, CTOPVAL);

  // Disable loops, we want the sequence to repeat only once
  nrf_pwm_loop_set(pwm, 0);

  // On the "Common" setting the PWM uses the same pattern for the
  // for supported sequences. The pattern is stored on half-word of 16bits
  nrf_pwm_decoder_set(pwm, PWM_DECODER_LOAD_Common, PWM_DECODER_MODE_RefreshCount);

  // The following settings are ignored with the current config.
  nrf_pwm_seq_refresh_set(pwm, 0, 0);
  nrf_pwm_seq_end_delay_set(pwm, 0, 0);

  // The Neopixel implementation is a blocking algorithm. DMA
  // allows for non-blocking operation. To "simulate" a blocking
  // operation we enable the interruption for the end of sequence
  // and block the execution thread until the event flag is set by
  // the peripheral.
  //    pwm->INTEN |= (PWM_INTEN_SEQEND0_Enabled<<PWM_INTEN_SEQEND0_Pos);

  // PSEL must be configured before enabling PWM
  nrf_pwm_pins_set(pwm, (uint32_t[] ) { LED_NEOPIXEL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL });

  // Enable the PWM
  nrf_pwm_enable(pwm);
}

void neopixel_teardown(void)
{
  uint8_t rgb[3] = { 0, 0, 0 };

  NRFX_DELAY_US(50);  // wait for previous write is complete

  neopixel_write(rgb);
  NRFX_DELAY_US(50);  // wait for this write

  pwm_teardown(NRF_PWM1);
}

// write 3 bytes color RGB to built-in neopixel
void neopixel_write (uint8_t *pixels)
{
  // convert RGB to GRB
  uint8_t grb[BYTE_PER_PIXEL] = {pixels[1], pixels[2], pixels[0]};
  uint16_t pos = 0;    // bit position

  // Set all neopixel to same value
  for (uint16_t n = 0; n < NEOPIXELS_NUMBER; n++ )
  {
    for(uint8_t c = 0; c < BYTE_PER_PIXEL; c++)
    {
      uint8_t const pix = grb[c];

      for ( uint8_t mask = 0x80; mask > 0; mask >>= 1 )
      {
        pixels_pattern[pos] = (pix & mask) ? MAGIC_T1H : MAGIC_T0H;
        pos++;
      }
    }
  }

  // Zero padding to indicate the end of sequence
  pixels_pattern[pos++] = 0 | (0x8000);    // Seq end
  pixels_pattern[pos++] = 0 | (0x8000);    // Seq end

  NRF_PWM_Type* pwm = NRF_PWM1;

  nrf_pwm_seq_ptr_set(pwm, 0, pixels_pattern);
  nrf_pwm_seq_cnt_set(pwm, 0, sizeof(pixels_pattern)/2);
  nrf_pwm_event_clear(pwm, NRF_PWM_EVENT_SEQEND0);
  nrf_pwm_task_trigger(pwm, NRF_PWM_TASK_SEQSTART0);

  // blocking wait for sequence complete
  while( !nrf_pwm_event_check(pwm, NRF_PWM_EVENT_SEQEND0) ) {}
  nrf_pwm_event_clear(pwm, NRF_PWM_EVENT_SEQEND0);
}
#endif

#if defined(LED_RGB_RED_PIN) && defined(LED_RGB_GREEN_PIN) && defined(LED_RGB_BLUE_PIN)

#ifdef LED_SECONDARY_PIN
#error "Cannot use secondary LED at the same time as an RGB status LED."
#endif

#define LED_RGB_RED   1
#define LED_RGB_BLUE  2
#define LED_RGB_GREEN 3

void neopixel_init(void)
{
  led_pwm_init(LED_RGB_RED, LED_RGB_RED_PIN);
  led_pwm_init(LED_RGB_GREEN, LED_RGB_GREEN_PIN);
  led_pwm_init(LED_RGB_BLUE, LED_RGB_BLUE_PIN);
}

void neopixel_teardown(void)
{
  uint8_t rgb[3] = { 0, 0, 0 };
  neopixel_write(rgb);
  nrf_gpio_cfg_default(LED_RGB_RED_PIN);
  nrf_gpio_cfg_default(LED_RGB_GREEN_PIN);
  nrf_gpio_cfg_default(LED_RGB_BLUE_PIN);
}

// write 3 bytes color to a built-in neopixel
void neopixel_write (uint8_t *pixels)
{
  led_pwm_duty_cycle(LED_RGB_RED, pixels[2]);
  led_pwm_duty_cycle(LED_RGB_GREEN, pixels[1]);
  led_pwm_duty_cycle(LED_RGB_BLUE, pixels[0]);
}
#endif
