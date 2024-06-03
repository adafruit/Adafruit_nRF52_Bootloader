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
#ifndef NRF52820_XXAA 
#include "nrf_pwm.h"
#endif
#include "app_scheduler.h"
#include "app_timer.h"

#if defined(LED_APA102) || defined(LED_SHIFTER)
#include "nrf_spim.h"
#endif

#define SCHED_MAX_EVENT_DATA_SIZE           sizeof(app_timer_event_t)        /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                    30                               /**< Maximum number of events in the scheduler queue. */

#if defined(LED_NEOPIXEL) || defined(LED_RGB_RED_PIN) || defined(LED_APA102) || defined(LED_SHIFTER)
void neopixel_init(void);
void neopixel_write(uint8_t* pixels);
void neopixel_teardown(void);
#endif

//--------------------------------------------------------------------+
// IMPLEMENTATION
//--------------------------------------------------------------------+

static uint32_t _systick_count = 0;
void SysTick_Handler(void) {
  _systick_count++;
  led_tick();
}

void button_init(uint32_t pin) {
  if (BUTTON_PULL == NRF_GPIO_PIN_PULLDOWN) {
    nrf_gpio_cfg_sense_input(pin, BUTTON_PULL, NRF_GPIO_PIN_SENSE_HIGH);
  } else {
    nrf_gpio_cfg_sense_input(pin, BUTTON_PULL, NRF_GPIO_PIN_SENSE_LOW);
  }
}

bool button_pressed(uint32_t pin) {
  uint32_t const active_state = (BUTTON_PULL == NRF_GPIO_PIN_PULLDOWN ? 1 : 0);
  return nrf_gpio_pin_read(pin) == active_state;
}

// This is declared so that a board specific init can be called from here.
void __attribute__((weak)) board_init2(void) {}

void board_init(void) {
  // stop LF clock just in case we jump from application without reset
  NRF_CLOCK->TASKS_LFCLKSTOP = 1UL;

  // Use Internal OSC to compatible with all boards
  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_RC;
  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;

  button_init(BUTTON_DFU);
  button_init(BUTTON_FRESET);
  NRFX_DELAY_US(100); // wait for the pin state is stable

#if LEDS_NUMBER > 0
  // use PMW0 for LED RED
  led_pwm_init(LED_PRIMARY, LED_PRIMARY_PIN);
  #if LEDS_NUMBER > 1
  led_pwm_init(LED_SECONDARY, LED_SECONDARY_PIN);
  #endif
#endif

#if defined(LED_NEOPIXEL) || defined(LED_RGB_RED_PIN) || defined(LED_APA102) || defined(LED_SHIFTER)
  // use neopixel for use enumeration
  #ifdef NEOPIXEL_POWER_PIN
  nrf_gpio_cfg_output(NEOPIXEL_POWER_PIN);
  nrf_gpio_pin_write(NEOPIXEL_POWER_PIN, 1);
  #endif

  neopixel_init();
#endif

#if ENABLE_DCDC_0 == 1
  NRF_POWER->DCDCEN0 = 1;
#endif
#if ENABLE_DCDC_1 == 1
  NRF_POWER->DCDCEN = 1;
#endif

  // Make sure any custom inits are performed
  board_init2();

  // When board is supplied on VDDH (and not VDD), this specifies what voltage the GPIO should run at
  // and what voltage is output at VDD. The default (0xffffffff) is 1.8V; typically you'll want
  //     #define UICR_REGOUT0_VALUE UICR_REGOUT0_VOUT_3V3
  // in board.h when using that power configuration.
#ifdef UICR_REGOUT0_VALUE
  if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) != (UICR_REGOUT0_VALUE << UICR_REGOUT0_VOUT_Pos)){
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
  SysTick_Config(SystemCoreClock / 1000);
}

// Actions at the end of board_teardown.
void __attribute__((weak)) board_teardown2(void) {}

void board_teardown(void) {
  // Disable systick, turn off LEDs
  SysTick->CTRL = 0;

  // Disable and reset PWM for LEDs
#if LEDS_NUMBER > 0
  led_pwm_teardown();
#endif

#if defined(LED_NEOPIXEL) || defined(LED_RGB_RED_PIN) || defined(LED_APA102) || defined(LED_SHIFTER)
  neopixel_teardown();
#endif

#ifdef DISPLAY_PIN_SCK
  board_display_teardown();
#endif

  // Stop RTC1 used by app_timer
  NVIC_DisableIRQ(RTC1_IRQn);
  NRF_RTC1->EVTENCLR = RTC_EVTEN_COMPARE0_Msk;
  NRF_RTC1->INTENCLR = RTC_INTENSET_COMPARE0_Msk;
  NRF_RTC1->TASKS_STOP = 1;
  NRF_RTC1->TASKS_CLEAR = 1;

  // Stop LF clock
  NRF_CLOCK->TASKS_LFCLKSTOP = 1UL;

  // make sure all pins are back in reset state
  // NUMBER_OF_PINS is defined in nrf_gpio.h
  for (int i = 0; i < NUMBER_OF_PINS; ++i) {
    nrf_gpio_cfg_default(i);
  }

  // board specific teardown actions
  board_teardown2();
}

//--------------------------------------------------------------------+
// Display
//--------------------------------------------------------------------+
#ifdef DISPLAY_PIN_SCK
#include "nrf_spim.h"

#define TFT_MADCTL_MY  0x80  ///< Page addr order: Bottom to top
#define TFT_MADCTL_MX  0x40  ///< Column addr order: Right to left
#define TFT_MADCTL_MV  0x20  ///< Page/Column order: Reverse Mode ( X <-> Y )
#define TFT_MADCTL_ML  0x10  ///< LCD refresh Bottom to top
#define TFT_MADCTL_MH  0x04  ///< LCD refresh right to left
#define TFT_MADCTL_RGB 0x00  ///< Red-Green-Blue pixel order
#define TFT_MADCTL_BGR 0x08  ///< Blue-Green-Red pixel order

// Note don't use SPIM3 since it has lots of errata
NRF_SPIM_Type* _spim = NRF_SPIM0;

static void spi_write(NRF_SPIM_Type *p_spim, uint8_t const *tx_buf, size_t tx_len) {
  nrf_spim_tx_buffer_set(p_spim, tx_buf, tx_len);
  nrf_spim_rx_buffer_set(p_spim, NULL, 0);

  nrf_spim_event_clear(p_spim, NRF_SPIM_EVENT_ENDTX);
  nrf_spim_event_clear(p_spim, NRF_SPIM_EVENT_END);
  nrf_spim_task_trigger(p_spim, NRF_SPIM_TASK_START);

  // blocking wait until xfer complete
  while (!nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_END)){}
}

static void tft_controller_init(void);

static inline void tft_cs(bool state) {
  nrf_gpio_pin_write(DISPLAY_PIN_CS, state);
}

static inline void tft_dc(bool state) {
  nrf_gpio_pin_write(DISPLAY_PIN_DC, state);
}

static void tft_cmd(uint8_t cmd, uint8_t const* data, size_t narg) {
  tft_cs(false);

  // send command
  tft_dc(false);
  spi_write(_spim, &cmd, 1);

  // send data
  if (narg > 0) {
    tft_dc(true);
    spi_write(_spim, data, narg);
  }

  tft_cs(true);
}

void board_display_init(void) {
  //------------- SPI init -------------//
  // highspeed SPIM should set SCK and MOSI to high drive
  nrf_gpio_cfg(DISPLAY_PIN_SCK, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT,
               NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg(DISPLAY_PIN_MOSI, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
               NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg_output(DISPLAY_PIN_CS);
  nrf_gpio_pin_set(DISPLAY_PIN_CS);

  nrf_spim_pins_set(_spim, DISPLAY_PIN_SCK, DISPLAY_PIN_MOSI, NRF_SPIM_PIN_NOT_CONNECTED);
  nrf_spim_frequency_set(_spim, NRF_SPIM_FREQ_4M);
  nrf_spim_configure(_spim, NRF_SPIM_MODE_0, NRF_SPIM_BIT_ORDER_MSB_FIRST);
  nrf_spim_orc_set(_spim, 0xFF);

  nrf_spim_enable(_spim);

  //------------- Display Init -------------//
  nrf_gpio_cfg_output(DISPLAY_PIN_DC);

#if defined(DISPLAY_VSENSOR_PIN) && DISPLAY_VSENSOR_PIN >= 0
  nrf_gpio_cfg_output(DISPLAY_VSENSOR_PIN);
  nrf_gpio_pin_write(DISPLAY_VSENSOR_PIN, DISPLAY_VSENSOR_ON);
#endif

#if defined(DISPLAY_PIN_RST) && DISPLAY_PIN_RST >= 0
  nrf_gpio_cfg_output(DISPLAY_PIN_RST);
  nrf_gpio_pin_clear(DISPLAY_PIN_RST);
  NRFX_DELAY_MS(10);
  nrf_gpio_pin_set(DISPLAY_PIN_RST);
  NRFX_DELAY_MS(20);
#endif

#if defined(DISPLAY_PIN_BL) && DISPLAY_PIN_BL >= 0
  nrf_gpio_cfg_output(DISPLAY_PIN_BL);
  nrf_gpio_pin_write(DISPLAY_PIN_BL, DISPLAY_BL_ON);
#endif

  tft_controller_init();
}

void board_display_teardown(void) {
  nrf_spim_disable(_spim);
}

void board_display_draw_line(uint16_t y, uint8_t const* buf, size_t nbytes) {
  // column and row address set
  uint32_t xa32 = DISPLAY_COL_OFFSET << 16 | DISPLAY_WIDTH;
  xa32 = __builtin_bswap32(xa32);

  y += DISPLAY_ROW_OFFSET;
  uint32_t ya32 = (y << 16) | (y + 1);
  ya32 = __builtin_bswap32(ya32);

  tft_cmd(0x2A, (uint8_t*) &xa32, 4);
  tft_cmd(0x2B, (uint8_t*) &ya32, 4);

  // command: memory write
  tft_cmd(0x2C, buf, nbytes);
}

#endif


#ifdef EPD_PIN_SCK
#include "nrf_spim.h"
// Note don't use SPIM3 since it has lots of errata
NRF_SPIM_Type* _spim = NRF_SPIM0;

static void spi_write(NRF_SPIM_Type *p_spim, uint8_t const *tx_buf, size_t tx_len) {
  nrf_spim_tx_buffer_set(p_spim, tx_buf, tx_len);
  nrf_spim_rx_buffer_set(p_spim, NULL, 0);

  nrf_spim_event_clear(p_spim, NRF_SPIM_EVENT_ENDTX);
  nrf_spim_event_clear(p_spim, NRF_SPIM_EVENT_END);
  nrf_spim_task_trigger(p_spim, NRF_SPIM_TASK_START);

  // blocking wait until xfer complete
  while (!nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_END)){}
}

static void epd_controller_init(void);

static inline void epd_cs(bool state) {
  nrf_gpio_pin_write(EPD_PIN_CS, state);
}

static inline void epd_dc(bool state) {
  nrf_gpio_pin_write(EPD_PIN_DC, state);
}

static void epd_bsy_wait(void) {
  while(!nrf_gpio_pin_read(EPD_PIN_BSY)) 
    NRFX_DELAY_MS(1);
}

static void epd_cmd(uint8_t cmd, uint8_t const* data, size_t narg) {
  epd_cs(false);

  // send command
  epd_dc(false);
  spi_write(_spim, &cmd, 1);

  // send data
  if (narg > 0) {
    epd_dc(true);
    spi_write(_spim, data, narg);
  }

  epd_cs(true);
}

void board_epd_init(void) {
  //------------- SPI init -------------//
  // highspeed SPIM should set SCK and MOSI to high drive
  nrf_gpio_cfg(EPD_PIN_SCK, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT,
               NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg(EPD_PIN_MOSI, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
               NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg_output(EPD_PIN_CS);
  nrf_gpio_pin_set(EPD_PIN_CS);

  nrf_spim_pins_set(_spim, EPD_PIN_SCK, EPD_PIN_MOSI, NRF_SPIM_PIN_NOT_CONNECTED);
  nrf_spim_frequency_set(_spim, NRF_SPIM_FREQ_4M);
  nrf_spim_configure(_spim, NRF_SPIM_MODE_0, NRF_SPIM_BIT_ORDER_MSB_FIRST);
  nrf_spim_orc_set(_spim, 0xFF);

  nrf_spim_enable(_spim);

  //------------- Display Init -------------//
  nrf_gpio_cfg_output(EPD_PIN_DC);

#if defined(EPD_PWR_PIN) && EPD_PWR_PIN >= 0
  nrf_gpio_cfg_output(EPD_PWR_PIN);
  nrf_gpio_pin_write(EPD_PWR_PIN, EPD_PWR_ON);
#endif

#if defined(EPD_PIN_BSY) && EPD_PIN_BSY >= 0
  nrf_gpio_cfg_sense_input(EPD_PIN_BSY, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);
#endif

#if defined(EPD_PIN_RST) && EPD_PIN_RST >= 0
  nrf_gpio_cfg_output(EPD_PIN_RST);
  nrf_gpio_pin_clear(EPD_PIN_RST);
  NRFX_DELAY_MS(10);
  nrf_gpio_pin_set(EPD_PIN_RST);
  NRFX_DELAY_MS(20);
  epd_bsy_wait();
#endif

  epd_controller_init();
}

void board_epd_teardown(void) {
  nrf_spim_disable(_spim);
}

//Line drawing in screen specific code

#endif

//--------------------------------------------------------------------+
// LED Indicator
//--------------------------------------------------------------------+
#ifndef NRF52820_XXAA 

void pwm_teardown(NRF_PWM_Type* pwm) {
  pwm->TASKS_SEQSTART[0] = 0;
  pwm->ENABLE = 0;

  pwm->PSEL.OUT[0] = 0xFFFFFFFF;
  pwm->PSEL.OUT[1] = 0xFFFFFFFF;
  pwm->PSEL.OUT[2] = 0xFFFFFFFF;
  pwm->PSEL.OUT[3] = 0xFFFFFFFF;

  pwm->MODE = 0;
  pwm->COUNTERTOP = 0x3FF;
  pwm->PRESCALER = 0;
  pwm->DECODER = 0;
  pwm->LOOP = 0;
  pwm->SEQ[0].PTR = 0;
  pwm->SEQ[0].CNT = 0;
}

static uint16_t led_duty_cycles[PWM0_CH_NUM] = {0};

#if LEDS_NUMBER > PWM0_CH_NUM
#error "Only " PWM0_CH_NUM " concurrent status LEDs are supported."
#endif

void led_pwm_init(uint32_t led_index, uint32_t led_pin) {
  NRF_PWM_Type* pwm = NRF_PWM0;

  pwm->ENABLE = 0;

  nrf_gpio_cfg_output(led_pin);
  nrf_gpio_pin_write(led_pin, 1 - LED_STATE_ON);

  pwm->PSEL.OUT[led_index] = led_pin;

  pwm->MODE = PWM_MODE_UPDOWN_Up;
  pwm->COUNTERTOP = 0xff;
  pwm->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_16;
  pwm->DECODER = PWM_DECODER_LOAD_Individual;
  pwm->LOOP = 0;

  pwm->SEQ[0].PTR = (uint32_t) (led_duty_cycles);
  pwm->SEQ[0].CNT = 4; // default mode is Individual --> count must be 4
  pwm->SEQ[0].REFRESH = 0;
  pwm->SEQ[0].ENDDELAY = 0;

  pwm->ENABLE = 1;

  pwm->EVENTS_SEQEND[0] = 0;
//  pwm->TASKS_SEQSTART[0] = 1;
}

void led_pwm_teardown(void) {
  pwm_teardown(NRF_PWM0);
}

void led_pwm_duty_cycle(uint32_t led_index, uint16_t duty_cycle) {
  led_duty_cycles[led_index] = duty_cycle;
  nrf_pwm_event_clear(NRF_PWM0, NRF_PWM_EVENT_SEQEND0);
  nrf_pwm_task_trigger(NRF_PWM0, NRF_PWM_TASK_SEQSTART0);
}
#endif

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
#ifndef NRF52820_XXAA 
  uint16_t duty_cycle = 0x4f * cycle / half_cycle;
  #if LED_STATE_ON == 1
  duty_cycle = 0xff - duty_cycle;
  #endif
  led_pwm_duty_cycle(LED_PRIMARY, duty_cycle);
#endif

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

void led_state(uint32_t state) {
  uint32_t new_rgb_color = rgb_color;
  uint32_t temp_color = 0;
  switch (state) {
    case STATE_USB_MOUNTED:
      new_rgb_color = 0x00ff00;
      primary_cycle_length = 3000;
      break;

    case STATE_BOOTLOADER_STARTED:
    case STATE_USB_UNMOUNTED:
      new_rgb_color = 0xffffff;
      primary_cycle_length = 300;
      break;

    case STATE_WRITING_STARTED:
      temp_color = 0x0000ff;
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
  if (temp_color != 0) {
    temp_color &= BOARD_RGB_BRIGHTNESS;
    final_color = (uint8_t*) &temp_color;
    temp_color_active = true;
  } else if (new_rgb_color != rgb_color) {
    final_color = (uint8_t*) &new_rgb_color;
    rgb_color = new_rgb_color;
  } else if (temp_color_active) {
    final_color = (uint8_t*) &rgb_color;
  }

#if defined(LED_NEOPIXEL) || defined(LED_RGB_RED_PIN) || defined(LED_APA102) || defined(LED_SHIFTER)
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

static uint16_t pixels_pattern[NEOPIXELS_NUMBER * BYTE_PER_PIXEL * 8 + 2];

// use PWM1 for neopixel
void neopixel_init(void) {
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
  nrf_pwm_pins_set(pwm, (uint32_t[]) {LED_NEOPIXEL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL});

  // Enable the PWM
  nrf_pwm_enable(pwm);
}

void neopixel_teardown(void) {
  uint8_t rgb[3] = {0, 0, 0};

  NRFX_DELAY_US(50);  // wait for previous write is complete
  neopixel_write(rgb);
  NRFX_DELAY_US(50);  // wait for this write
  pwm_teardown(NRF_PWM1);
}

// write 3 bytes color RGB to built-in neopixel
void neopixel_write(uint8_t* pixels) {
  // convert RGB to GRB
  uint8_t grb[BYTE_PER_PIXEL] = {pixels[1], pixels[2], pixels[0]};
  uint16_t pos = 0;    // bit position

  // Set all neopixel to same value
  for (uint16_t n = 0; n < NEOPIXELS_NUMBER; n++) {
    for (uint8_t c = 0; c < BYTE_PER_PIXEL; c++) {
      uint8_t const pix = grb[c];

      for (uint8_t mask = 0x80; mask > 0; mask >>= 1) {
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
  nrf_pwm_seq_cnt_set(pwm, 0, sizeof(pixels_pattern) / 2);
  nrf_pwm_event_clear(pwm, NRF_PWM_EVENT_SEQEND0);
  nrf_pwm_task_trigger(pwm, NRF_PWM_TASK_SEQSTART0);

  // blocking wait for sequence complete
  while (!nrf_pwm_event_check(pwm, NRF_PWM_EVENT_SEQEND0)) {}
  nrf_pwm_event_clear(pwm, NRF_PWM_EVENT_SEQEND0);
}

#endif

#ifdef LED_APA102
#define BYTE_PER_PIXEL  4

// 4 zero bytes are required to initiate update
#define PATTERN_SIZE() ((APA102_NUMBER*BYTE_PER_PIXEL) + 4)
// N/2 * 1 bits are required at the end
static uint8_t pixels_pattern[PATTERN_SIZE() + 4];

// use SPIM1 for dotstar
void neopixel_init(void) {
  NRF_SPIM_Type* spi = NRF_SPIM1;

  nrf_spim_disable(spi);

  nrf_gpio_pin_set(LED_APA102_CLK);

  nrf_gpio_cfg(LED_APA102_CLK,
                NRF_GPIO_PIN_DIR_OUTPUT,
                NRF_GPIO_PIN_INPUT_CONNECT,
                NRF_GPIO_PIN_NOPULL,
                NRF_GPIO_PIN_S0S1,
                NRF_GPIO_PIN_NOSENSE);

  nrf_gpio_pin_clear(LED_APA102_DATA);
  nrf_gpio_cfg_output(LED_APA102_DATA);

  nrf_spim_pins_set(spi, LED_APA102_CLK, LED_APA102_DATA, 0xFFFFFFFF);
  nrf_spim_frequency_set(spi, NRF_SPIM_FREQ_4M);
  nrf_spim_configure(spi, NRF_SPIM_MODE_3, NRF_SPIM_BIT_ORDER_MSB_FIRST);

  nrf_spim_orc_set(spi, 0);
  nrf_spim_tx_list_disable(spi);

  // Enable the spi
  nrf_spim_enable(spi);

  uint8_t rgb[3] = {0, 0, 0 };
  neopixel_write(rgb);
}

void neopixel_teardown(void) {
  uint8_t rgb[3] = {0, 0, 0 };
  neopixel_write(rgb);

  NRF_SPIM_Type* spi = NRF_SPIM1;
  nrf_spim_disable(spi);
}

// write 3 bytes color RGB to built-in neopixel
void neopixel_write (uint8_t *pixels) {
  NRF_SPIM_Type*  spi = NRF_SPIM1;

  //brightness, blue, green, red
  uint8_t bbgr[BYTE_PER_PIXEL] = {0xE0 | LED_APA102_BRIGHTNESS, pixels[0], pixels[1], pixels[2]};
  pixels_pattern[0] = 0;
  pixels_pattern[1] = 0;
  pixels_pattern[2] = 0;
  pixels_pattern[3] = 0;

  for (uint8_t i = 4; i < PATTERN_SIZE(); i+=4) {
    pixels_pattern[i] = bbgr[0];
    pixels_pattern[i+1] = bbgr[1];
    pixels_pattern[i+2] = bbgr[2];
    pixels_pattern[i+3] = bbgr[3];
  }

  pixels_pattern[PATTERN_SIZE()] = 0xff;
  pixels_pattern[PATTERN_SIZE()+1] = 0xff;
  pixels_pattern[PATTERN_SIZE()+2] = 0xff;
  pixels_pattern[PATTERN_SIZE()+3] = 0xff;

  nrf_spim_tx_buffer_set(spi, pixels_pattern, PATTERN_SIZE() + 4);
  nrf_spim_event_clear(spi, NRF_SPIM_EVENT_ENDTX);

  nrf_spim_task_trigger(spi, NRF_SPIM_TASK_START);

  while(!nrf_spim_event_check(spi, NRF_SPIM_EVENT_ENDTX));
}
#endif


#if defined(LED_RGB_RED_PIN) && defined(LED_RGB_GREEN_PIN) && defined(LED_RGB_BLUE_PIN)

#ifdef LED_SECONDARY_PIN
#error "Cannot use secondary LED at the same time as an RGB status LED."
#endif

#define LED_RGB_RED   1
#define LED_RGB_BLUE  2
#define LED_RGB_GREEN 3

void neopixel_init(void) {
  led_pwm_init(LED_RGB_RED, LED_RGB_RED_PIN);
  led_pwm_init(LED_RGB_GREEN, LED_RGB_GREEN_PIN);
  led_pwm_init(LED_RGB_BLUE, LED_RGB_BLUE_PIN);
}

void neopixel_teardown(void) {
  uint8_t rgb[3] = { 0, 0, 0 };
  neopixel_write(rgb);
  nrf_gpio_cfg_default(LED_RGB_RED_PIN);
  nrf_gpio_cfg_default(LED_RGB_GREEN_PIN);
  nrf_gpio_cfg_default(LED_RGB_BLUE_PIN);
}

// write 3 bytes color to a built-in neopixel
void neopixel_write (uint8_t *pixels) {
  led_pwm_duty_cycle(LED_RGB_RED, pixels[2]);
  led_pwm_duty_cycle(LED_RGB_GREEN, pixels[1]);
  led_pwm_duty_cycle(LED_RGB_BLUE, pixels[0]);
}
#endif

#if defined(LED_SHIFTER) && defined(SHIFTER_PIN_CS) && defined(SHIFTER_PIN_MOSI) && defined(SHIFTER_PIN_SCK)  && defined(SHIFTER_INDEX)

static void spi_write(NRF_SPIM_Type *p_spim, uint8_t const *tx_buf, size_t tx_len) {
  nrf_spim_tx_buffer_set(p_spim, tx_buf, tx_len);
  nrf_spim_rx_buffer_set(p_spim, NULL, 0);

  nrf_spim_event_clear(p_spim, NRF_SPIM_EVENT_ENDTX);
  nrf_spim_event_clear(p_spim, NRF_SPIM_EVENT_END);
  nrf_spim_task_trigger(p_spim, NRF_SPIM_TASK_START);

  // blocking wait until xfer complete
  while (!nrf_spim_event_check(p_spim, NRF_SPIM_EVENT_END)){}
}
void neopixel_init(void) {
  NRF_SPIM_Type* spi = NRF_SPIM1;
  //------------- SPI init -------------//
  // highspeed SPIM should set SCK and MOSI to high drive
  nrf_gpio_cfg(SHIFTER_PIN_SCK, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT,
               NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg(SHIFTER_PIN_MOSI, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
               NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg_output(SHIFTER_PIN_CS);
  nrf_gpio_pin_set(SHIFTER_PIN_CS);

  nrf_spim_pins_set(spi, SHIFTER_PIN_SCK, SHIFTER_PIN_MOSI, NRF_SPIM_PIN_NOT_CONNECTED);
  nrf_spim_frequency_set(spi, NRF_SPIM_FREQ_4M);
  nrf_spim_configure(spi, NRF_SPIM_MODE_0, NRF_SPIM_BIT_ORDER_MSB_FIRST);
  nrf_spim_orc_set(spi, 0xFF);

  nrf_spim_enable(spi);
}

void neopixel_teardown(void) {
  //uint8_t rgb[3] = { 0, 0, 0 };
  //neopixel_write(rgb);
NRF_SPIM_Type* spi = NRF_SPIM1;
    nrf_spim_disable(spi);

}

// write 3 bytes color to a built-in neopixel
void neopixel_write (uint8_t *pixels) {
  nrf_gpio_pin_write(SHIFTER_PIN_CS, false);
  NRF_SPIM_Type* spi = NRF_SPIM1;
  //djfhdskjss
  uint8_t data = 0;
  // send data
 #if defined(SHIFTER_COM_ANODE)
  data = (pixels[0] ? 0 : 1) << SHIFTER_CH_R | (pixels[1] ? 0 : 1) << SHIFTER_CH_G | (pixels[2] ? 0 : 1) << SHIFTER_CH_B;
  #if defined(SHIFTER_2_LEDS)
  data |= (pixels[0] ? 0 : 1) << (3+SHIFTER_CH_R) | (pixels[1] ? 0 : 1) << (3+SHIFTER_CH_G) | (pixels[2] ? 0 : 1) << (3+SHIFTER_CH_B);
  #endif
  #else
  data = (pixels[0] ? 1 : 0)  << SHIFTER_CH_R | (pixels[1] ? 1 : 0) << SHIFTER_CH_G | (pixels[2] ? 1 : 0) << SHIFTER_CH_B;
  #if defined(SHIFTER_2_LEDS)
  data |= (pixels[0] ? 1 : 0) << (3+SHIFTER_CH_R) | (pixels[1] ? 1 : 0) << (3+SHIFTER_CH_G) | (pixels[2] ? 1 : 0) << (3+SHIFTER_CH_B);
  #endif
  #endif
  spi_write(spi, &data, 1);
  
  #if SHIFTER_INDEX > 0
  data = 0;
    for(uint8_t i=0; i<SHIFTER_INDEX; i++)
      spi_write(spi, &data, 1);
  #endif
  nrf_gpio_pin_write(SHIFTER_PIN_CS, true);

}

#endif

//--------------------------------------------------------------------+
// Display controller
//--------------------------------------------------------------------+

#ifdef DISPLAY_CONTROLLER_ST7789

#define ST_CMD_DELAY 0x80 // special signifier for command lists

#define ST77XX_NOP 0x00
#define ST77XX_SWRESET 0x01
#define ST77XX_RDDID 0x04
#define ST77XX_RDDST 0x09

#define ST77XX_SLPIN 0x10
#define ST77XX_SLPOUT 0x11
#define ST77XX_PTLON 0x12
#define ST77XX_NORON 0x13

#define ST77XX_INVOFF 0x20
#define ST77XX_INVON 0x21
#define ST77XX_DISPOFF 0x28
#define ST77XX_DISPON 0x29
#define ST77XX_CASET 0x2A
#define ST77XX_RASET 0x2B
#define ST77XX_RAMWR 0x2C
#define ST77XX_RAMRD 0x2E

#define ST77XX_PTLAR 0x30
#define ST77XX_TEOFF 0x34
#define ST77XX_TEON 0x35
#define ST77XX_MADCTL 0x36
#define ST77XX_VSCSAD 0x37
#define ST77XX_COLMOD 0x3A

#define ST77XX_MADCTL_MY 0x80
#define ST77XX_MADCTL_MX 0x40
#define ST77XX_MADCTL_MV 0x20
#define ST77XX_MADCTL_ML 0x10
#define ST77XX_MADCTL_RGB 0x00

#define ST77XX_RDID1 0xDA
#define ST77XX_RDID2 0xDB
#define ST77XX_RDID3 0xDC
#define ST77XX_RDID4 0xDD

// Some ready-made 16-bit ('565') color settings:
#define ST77XX_BLACK 0x0000
#define ST77XX_WHITE 0xFFFF
#define ST77XX_RED 0xF800
#define ST77XX_GREEN 0x07E0
#define ST77XX_BLUE 0x001F
#define ST77XX_CYAN 0x07FF
#define ST77XX_MAGENTA 0xF81F
#define ST77XX_YELLOW 0xFFE0
#define ST77XX_ORANGE 0xFC00

static void tft_controller_init(void) {
  // Init commands for 7789 screens
  uint8_t cmdinit_st7789[] = {
      #if !defined(DISPLAY_PIN_RST) || (DISPLAY_PIN_RST < 0)
      // Software reset if rst pin not available, no args, w/delay ~150 ms delay
      ST77XX_SWRESET, ST_CMD_DELAY, 150,
      #endif
      // Out of sleep mode, no args, w/delay 10 ms delay
      ST77XX_SLPOUT, ST_CMD_DELAY, 10,
      // Set color mode, 1 arg + delay: 16-bit color, 10 ms delay
      ST77XX_COLMOD, 1 + ST_CMD_DELAY, 0x55, 10,
      // Mem access ctrl (directions), 1 arg: Row/col addr, bottom-top refresh
      ST77XX_MADCTL, 1, DISPLAY_MADCTL,
      // Vertical Scroll Start Address of RAM
      // ST77XX_VSCSAD, 2, DISPLAY_VSCSAD >> 8, DISPLAY_VSCSAD & 0xFF,
      // Column addr set, 4 args, no delay: XSTART = 0, XEND = 240
      ST77XX_CASET, 4, 0x00, 0, 0, 240,
      // Row addr set, 4 args, no delay: YSTART = 0 YEND = 320
      ST77XX_RASET, 4, 0x00, 0, 320 >> 8, 320 & 0xFF,
      // Inversion on
      ST77XX_INVON, ST_CMD_DELAY, 10,
      // Normal display on, no args, w/delay 10 ms delay
      ST77XX_NORON, ST_CMD_DELAY, 10,
      // Main screen turn on, no args, delay 10 ms delay
      ST77XX_DISPON, ST_CMD_DELAY, 10
  };

  size_t count = 0;
  while (count < sizeof(cmdinit_st7789)) {
    uint8_t const cmd = cmdinit_st7789[count++];
    uint8_t const cmd_arg = cmdinit_st7789[count++];
    uint8_t const has_delay = cmd_arg & ST_CMD_DELAY;
    uint8_t const narg = cmd_arg & ~ST_CMD_DELAY;

    tft_cmd(cmd, cmdinit_st7789 + count, narg);
    count += narg;

    if (has_delay) {
      uint16_t delay = (uint16_t) cmdinit_st7789[count++];
      if (delay == 255) {
        delay = 500; // If 255, delay for 500 ms
      }
      NRFX_DELAY_MS(delay);
    }
  }
}

#endif

#ifdef EPD_CONTROLLER_IL0323

#define IL0323_CMD_PSR 0x00
#define IL0323_CMD_PWR 0x01
#define IL0323_CMD_POF 0x02
#define IL0323_CMD_PFS 0x03
#define IL0323_CMD_PON 0x04
#define IL0323_CMD_PMES 0x05
#define IL0323_CMD_CPSET 0x06
#define IL0323_CMD_DSLP 0x07
#define IL0323_CMD_DTM1 0x10
#define IL0323_CMD_DSP 0x11
#define IL0323_CMD_DRF 0x12
#define IL0323_CMD_DTM2 0x13
#define IL0323_CMD_AUTO 0x17
#define IL0323_CMD_LUTOPT 0x2A
#define IL0323_CMD_PLL 0x30
#define IL0323_CMD_TSC 0x40
#define IL0323_CMD_TSE 0x41
#define IL0323_CMD_PBC 0x44
#define IL0323_CMD_CDI 0x50
#define IL0323_CMD_LPD 0x51
#define IL0323_CMD_TCON 0x60
#define IL0323_CMD_TRES 0x61
#define IL0323_CMD_GSST 0x65
#define IL0323_CMD_REV 0x70
#define IL0323_CMD_FLG 0x71
#define IL0323_CMD_CRC 0x72
#define IL0323_CMD_AMV 0x80
#define IL0323_CMD_VV 0x81
#define IL0323_CMD_VDCS 0x82
#define IL0323_CMD_PTL 0x90
#define IL0323_CMD_PIN 0x91
#define IL0323_CMD_POUT 0x92
#define IL0323_CMD_PGM 0xA0
#define IL0323_CMD_APG 0xA1
#define IL0323_CMD_ROTP 0xA2
#define IL0323_CMD_CCSET 0xE0
#define IL0323_CMD_PWS 0xE3
#define IL0323_CMD_LVSEL 0xE4
#define IL0323_CMD_TSSET 0xE5
#define IL0323_CMD_LUTW 0x23
#define IL0323_CMD_LUTB 0x24

#define BIT(n)  (1UL << (n))
#define IL0323_PSR_RES_WIDTH BIT(7)
#define IL0323_PSR_RES_HEIGHT BIT(6)
#define IL0323_PSR_LUT_REG BIT(5)
#define IL0323_PSR_LUT_OTP BIT(4)
#define IL0323_PSR_UD BIT(3)
#define IL0323_PSR_SHL BIT(2)
#define IL0323_PSR_SHD BIT(1)
#define IL0323_PSR_RST BIT(0)

#define IL0323_AUTO_PON_DRF_POF 0xA5
#define IL0323_AUTO_PON_DRF_POF_DSLP 0xA7

#define IL0323_CDI_REG_LENGTH 1U
#define IL0323_CDI_CDI_IDX 0

#define IL0323_TRES_REG_LENGTH 2U
#define IL0323_TRES_HRES_IDX 0
#define IL0323_TRES_VRES_IDX 1

#define IL0323_PTL_REG_LENGTH 5U
#define IL0323_PTL_HRST_IDX 0
#define IL0323_PTL_HRED_IDX 1
#define IL0323_PTL_VRST_IDX 2
#define IL0323_PTL_VRED_IDX 3
#define IL0323_PTL_PT_SCAN BIT(0)

/* Time constants in ms */
#define IL0323_RESET_DELAY 10U
#define IL0323_PON_DELAY 100U
#define IL0323_BUSY_DELAY 1U


const uint8_t lut_w[] =
{
0x60	,0x01	,0x01	,0x00	,0x00	,0x01	,
0x80	,0x0f	,0x00	,0x00	,0x00	,0x01	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,

};	
const uint8_t lut_b[] =
{
0x90	,0x01	,0x01	,0x00	,0x00	,0x01	,
0x40	,0x0f	,0x00	,0x00	,0x00	,0x01	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,
0x00	,0x00	,0x00	,0x00	,0x00	,0x00	,

};

static void epd_controller_init(void) {
  
  uint8_t tmp[IL0323_TRES_REG_LENGTH];
  uint8_t pwr[] = {0x03, 0x00, 0x2b, 0x2b};


  tmp[0] = 0x6F;
  epd_cmd(IL0323_CMD_PSR, tmp, 1);
  epd_cmd(IL0323_CMD_PWR, pwr, sizeof(pwr));
  

  tmp[0] = IL0323_PSR_UD | IL0323_PSR_SHL | IL0323_PSR_SHD | IL0323_PSR_RST | IL0323_PSR_RES_HEIGHT;


  epd_cmd(IL0323_CMD_PSR, tmp, 1);
/* Set panel resolution */
  tmp[IL0323_TRES_HRES_IDX] = EPD_WIDTH;
  tmp[IL0323_TRES_VRES_IDX] = EPD_HEIGHT;
  epd_cmd(IL0323_CMD_TRES, tmp, IL0323_TRES_REG_LENGTH);

  tmp[0] = 0x00;
  tmp[1] = 0x00;

  epd_cmd(IL0323_CMD_LUTOPT, tmp, 2);

  tmp[0] = 0xF7;
  epd_cmd(IL0323_CMD_CDI, tmp, 1);

  tmp[0] = 0x22;
  epd_cmd(IL0323_CMD_TCON, tmp, 1);
  /* Enable Auto Sequence */
  tmp[0] = IL0323_AUTO_PON_DRF_POF;
  epd_cmd(IL0323_CMD_AUTO, tmp, 1);

  epd_cmd(IL0323_CMD_LUTW, lut_w, 42);
  epd_cmd(IL0323_CMD_LUTB, lut_b, 42);

  epd_cmd(IL0323_CMD_PON, NULL, 0);
  NRFX_DELAY_MS(IL0323_PON_DELAY);
  epd_bsy_wait();

}

static uint8_t old_buffer[1280];

#define IL0323_PIXELS_PER_BYTE 8U

/* Horizontally aligned page! */
#define IL0323_NUMOF_PAGES (EPD_PANEL_WIDTH / IL0323_PIXELS_PER_BYTE)
#define IL0323_PANEL_FIRST_GATE 0U
#define IL0323_PANEL_LAST_GATE (EPD_PANEL_HEIGHT - 1)
#define IL0323_PANEL_FIRST_PAGE 0U
#define IL0323_PANEL_LAST_PAGE (IL0323_NUMOF_PAGES - 1)

void board_epd_draw(uint8_t start_x,  uint8_t start_y, uint8_t end_x, uint8_t end_y, uint8_t const* buf, size_t nbytes) {
  uint8_t ptl[IL0323_PTL_REG_LENGTH] = {0};
  /* Setup Partial Window and enable Partial Mode */
    ptl[IL0323_PTL_HRST_IDX] = start_x;
    ptl[IL0323_PTL_HRED_IDX] = end_x;
    ptl[IL0323_PTL_VRST_IDX] = start_y;
    ptl[IL0323_PTL_VRED_IDX] = end_y;
    ptl[sizeof(ptl) - 1] = IL0323_PTL_PT_SCAN;

    epd_bsy_wait();
    epd_cmd(IL0323_CMD_PIN, NULL, 0);
    epd_cmd(IL0323_CMD_PTL, ptl, sizeof(ptl));
    epd_cmd(IL0323_CMD_DTM1, old_buffer, 1280);
    epd_cmd(IL0323_CMD_DTM2, buf, nbytes);

    memcpy(old_buffer, buf, 1280);

    epd_cmd(IL0323_CMD_DRF, NULL, 0);
    NRFX_DELAY_MS(1);
    epd_cmd(IL0323_CMD_POUT, NULL, 0);
    NRFX_DELAY_MS(500);
}

#endif