/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Scott Shawcroft for Adafruit Industries
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

#ifndef _PARTICLE_BORON_H
#define _PARTICLE_BORON_H

#define _PINNUM(port, pin)    ((port)*32 + (pin))

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER    1
#define LED_PRIMARY_PIN           _PINNUM(1, 12)
#define LED_STATE_ON   1

#define LED_RGB_RED_PIN           _PINNUM(0, 13)
#define LED_RGB_GREEN_PIN         _PINNUM(0, 14)
#define LED_RGB_BLUE_PIN          _PINNUM(0, 15)
#define BOARD_RGB_BRIGHTNESS 0x202020

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER 2
#define BUTTON_DFU          _PINNUM(0, 11)
#define BUTTON_FRESET       _PINNUM(0, 03) // A0
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

/*------------------------------------------------------------------*/
/* UART (only used by nRF52832)
 *------------------------------------------------------------------*/
#define RX_PIN_NUMBER  8
#define TX_PIN_NUMBER  6
#define CTS_PIN_NUMBER 0
#define RTS_PIN_NUMBER 0
#define HWFC           false

// Used as model string in OTA mode
#define BLEDIS_MANUFACTURER   "Particle Industries"
#define BLEDIS_MODEL          "Boron"

#define UF2_PRODUCT_NAME   "Particle Boron"
#define UF2_VOLUME_LABEL   "BORONBOOT  "
#define UF2_BOARD_ID       "nRF52840-Boron-v1"
#define UF2_INDEX_URL      "https://www.particle.io/mesh/"

#endif // _PARTICLE_BORON_H
