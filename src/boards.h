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

#ifndef BOARDS_H
#define BOARDS_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "nrf_gpio.h"

#include "board.h"

#ifndef UF2_VOLUME_LABEL
#define UF2_VOLUME_LABEL   "NRF52BOOT  "
#endif

#ifndef BUTTON_DFU
#define BUTTON_DFU      BUTTON_1
#endif

#ifndef BUTTON_FRESET
#define BUTTON_FRESET   BUTTON_2
#endif

// The primary LED is usually Red but not in all cases.
#define LED_PRIMARY 0
// The secondary LED, when available, is usually blue.
#define LED_SECONDARY 1

// Its more common for LEDs to be sinking to the MCU pin. Setting is only for individual LEDs, not
// RGB LEDs.
#ifndef LED_STATE_ON
#define LED_STATE_ON   0
#endif

// Internal status colors are masked by this brightness setting.
#ifndef BOARD_RGB_BRIGHTNESS
#define BOARD_RGB_BRIGHTNESS 0x101010
#endif

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

enum {
  STATE_BOOTLOADER_STARTED = 0,
  STATE_USB_MOUNTED,
  STATE_USB_UNMOUNTED,
  STATE_FACTORY_RESET_STARTED,
  STATE_FACTORY_RESET_FINISHED,
  STATE_WRITING_STARTED,
  STATE_WRITING_FINISHED,
  STATE_BLE_CONNECTED,
  STATE_BLE_DISCONNECTED
};

void led_pwm_init(uint32_t led_index, uint32_t led_pin);
void led_pwm_teardown(void);
void led_pwm_disable(uint32_t led_index);
void led_pwm_enable(uint32_t led_index);
void led_state(uint32_t state);
void led_tick(void);

//--------------------------------------------------------------------+
// BUTTONS
//--------------------------------------------------------------------+
// Make sure we have at least two buttons (DFU + FRESET since DFU+FRST=OTA)
#if BUTTONS_NUMBER < 2
#error "At least two buttons required in the BSP (see 'BUTTONS_NUMBER')"
#endif

void button_init(uint32_t pin);
bool button_pressed(uint32_t pin);

bool is_ota(void);

//--------------------------------------------------------------------+
// DEBUG
//--------------------------------------------------------------------+

#ifdef CFG_DEBUG

#include <stdio.h>

#define PRINTF                printf
#define PRINT_LOCATION()      printf("%s: %d:\n", __PRETTY_FUNCTION__, __LINE__)
#define PRINT_MESS(x)         printf("%s: %d: %s \n"   , __FUNCTION__, __LINE__, (char*)(x))
#define PRINT_STR(x)          printf("%s: %d: " #x " = %s\n"   , __FUNCTION__, __LINE__, (char*)(x) )
#define PRINT_INT(x)          printf("%s: %d: " #x " = %ld\n"  , __FUNCTION__, __LINE__, (uint32_t) (x) )
#define PRINT_HEX(x)          printf("%s: %d: " #x " = 0x%lX\n"  , __FUNCTION__, __LINE__, (uint32_t) (x) )

#define PRINT_BUFFER(buf, n) \
  do {\
    uint8_t const* p8 = (uint8_t const*) (buf);\
    printf(#buf ": ");\
    for(uint32_t i=0; i<(n); i++) printf("%x ", p8[i]);\
    printf("\n");\
  }while(0)

#else

#define PRINTF(...)
#define PRINT_LOCATION()
#define PRINT_MESS(x)
#define PRINT_STR(x)
#define PRINT_INT(x)
#define PRINT_HEX(x)
#define PRINT_BUFFER(buf, n)

#endif


#endif
