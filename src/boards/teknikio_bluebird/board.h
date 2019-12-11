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

#ifndef TEKNIKIO_BLUEBIRD_H
#define TEKNIKIO_BLUEBIRD_H

#define _PINNUM(port, pin)    ((port)*32 + (pin))

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER       0 
#define LED_PRIMARY_PIN   _PINNUM(0, 2)
#define LED_SECONDARY_PIN 12 // Blue
#define LED_STATE_ON      1
/*------------------------------------------------------------------*/
/* NEOPIXEL
 *------------------------------------------------------------------*/
#define NEOPIXELS_NUMBER  1
#define LED_NEOPIXEL      _PINNUM(0, 26)

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER  2
#define BUTTON_1       _PINNUM(0, 4)
#define BUTTON_2       _PINNUM(0, 6)
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

/*------------------------------------------------------------------*/
/* UART
 *------------------------------------------------------------------*/
#define RX_PIN_NUMBER  7
#define TX_PIN_NUMBER  8
#define HWFC           false

// Used as model string in OTA mode
#define BLEDIS_MANUFACTURER  "Teknikio"
#define BLEDIS_MODEL         "Bleubird"

#define UF2_PRODUCT_NAME  "Teknikio Bluebird"
#define UF2_BOARD_ID      "nRF52840-bluebird-v1"
#define UF2_INDEX_URL     "https://www.teknikio.com/pages/bluebird"

#endif