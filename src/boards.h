/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef BOARDS_H
#define BOARDS_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"

#if defined BOARD_FEATHER_NRF52840_EXPRESS
  #include "boards/feather_nrf52840_express.h"
#elif defined BOARD_FEATHER_NRF52832
  #include "boards/feather_nrf52832.h"
#elif defined BOARD_PCA10056
  #include "boards/pca10056.h"
#elif defined BOARD_PCA10059
  #include "boards/pca10059.h"
#else
  #error No boards defined
#endif

#define BUTTON_DFU      BUTTON_1
#define BUTTON_FRESET   BUTTON_2

#define LED_RED         LED_1
#define LED_BLUE        LED_2

// Helper function
#define memclr(buffer, size)                memset(buffer, 0, size)
#define varclr(_var)                        memclr(_var, sizeof(*(_var)))
#define arrclr(_arr)                        memclr(_arr, sizeof(_arr))
#define arrcount(_arr)                      ( sizeof(_arr) / sizeof(_arr[0]) )

void board_init(void);
void board_teardown(void);

//--------------------------------------------------------------------+
// LED
//--------------------------------------------------------------------+

#define bit(b) (1UL << (b))

static inline void led_control(uint32_t pin, bool state)
{
  nrf_gpio_pin_write(pin, state ? LED_STATE_ON : (1-LED_STATE_ON));
}

static inline void led_on(uint32_t pin)
{
  led_control(pin, true);
}

static inline void led_off(uint32_t pin)
{
  led_control(pin, false);
}

void led_pwm_init(uint32_t led_pin);
void led_pwm_teardown(uint32_t led_pin);
void led_pwm_disable(uint32_t led_pin);
void led_pwm_enable(uint32_t led_pin);

void led_red_blink_fast(bool enable);

#ifdef LED_NEOPIXEL
  void neopixel_write(uint8_t *pixels);
#endif

//--------------------------------------------------------------------+
// BUTTONS
//--------------------------------------------------------------------+
// Make sure we have at least two buttons (DFU + FRESET since DFU+FRST=OTA)
#if BUTTONS_NUMBER < 2
#error "At least two buttons required in the BSP (see 'BUTTONS_NUMBER')"
#endif

static inline void button_init(uint32_t pin)
{
  nrf_gpio_cfg_sense_input(pin, BUTTON_PULL, NRF_GPIO_PIN_SENSE_LOW);
}

static inline bool button_pressed(uint32_t pin)
{
  return (nrf_gpio_pin_read(pin) == 0) ? true : false;
}



bool is_ota(void);

#endif
