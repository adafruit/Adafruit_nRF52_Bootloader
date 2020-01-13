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

#ifndef _FEATHER_NRF52832_H
#define _FEATHER_NRF52832_H

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER        2
#define LED_PRIMARY_PIN    17 // Red
#define LED_SECONDARY_PIN  19 // Blue
#define LED_STATE_ON       1

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER     2
#define BUTTON_1           20
#define BUTTON_2           22
#define BUTTON_PULL        NRF_GPIO_PIN_PULLUP

/*------------------------------------------------------------------*/
/* UART (only used by nRF52832)
 *------------------------------------------------------------------*/
#define RX_PIN_NUMBER      8
#define TX_PIN_NUMBER      6
#define CTS_PIN_NUMBER     0
#define RTS_PIN_NUMBER     0
#define HWFC               false

// Used as model string in OTA mode
#define BLEDIS_MANUFACTURER "Adafruit Industries"
#define BLEDIS_MODEL        "Feather nRF52832"

#define UF2_PRODUCT_NAME    "Adafruit Feather nRF52832"
#define UF2_INDEX_URL       "https://www.adafruit.com/product/3406"

#endif // _FEATHER_NRF52832_H
