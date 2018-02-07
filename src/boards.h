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
#include "nrf_gpio.h"

#if defined BOARD_FEATHER52
#include "feather52.h"
#elif defined BOARD_METRO52
#include "metro52.h"
#else
#error No boards defined
#endif

// Make sure we have at least two buttons (DFU + FRESET since DFU+FRST=OTA)
#if BUTTONS_NUMBER < 2
#error "At least two buttons required in the BSP (see 'BUTTONS_NUMBER')"
#endif

#define LED_RED     LED_1
#define LED_BLUE    LED_2

#define bit(b) (1UL << (b))

static inline uint32_t bit_set(uint32_t value, uint8_t n)
{
  return value | bit(n);
}

static inline uint32_t bit_clear(uint32_t value, uint8_t n)
{
  return value & (~bit(n));
}

static inline bool bit_test(uint32_t value, uint8_t n)
{
  return (value & bit(n)) ? true : false;
}


void led_control(uint32_t pin, bool state);

static inline void led_on(uint32_t pin)
{
  led_control(pin, true);
}

static inline void led_off(uint32_t pin)
{
  led_control(pin, false);
}

#endif
