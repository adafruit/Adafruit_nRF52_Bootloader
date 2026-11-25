/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Yihui Xiong for Makerdiary
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

#ifndef _ME25LS01_H
#define _ME25LS01_H

#define _PINNUM(port, pin)    ((port)*32 + (pin))

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER           3
#define LED_PRIMARY_PIN       _PINNUM(1, 07)  // Blue
#define LED_SECONDARY_PIN     _PINNUM(1, 05)  // Red
#define LED_TERTIARY_PIN      _PINNUM(0, 22)  // Blue
#define LED_STATE_ON          0

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER        3
#define BUTTON_1              _PINNUM(0, 27) // Primary Button
#define BUTTON_2              _PINNUM(0, 18) // unusable: RESET
#define BUTTON_3              _PINNUM(0, 11) // Secondary Button
#define BUTTON_PULL           NRF_GPIO_PIN_PULLUP

//--------------------------------------------------------------------+
// BLE OTA
//--------------------------------------------------------------------+
#define BLEDIS_MANUFACTURER   "Minewsemi"
#define BLEDIS_MODEL          "ME25LS01"

//--------------------------------------------------------------------+
// USB
//--------------------------------------------------------------------+
#define USB_DESC_VID            0x1209
#define USB_DESC_UF2_PID        0x5285
#define USB_DESC_CDC_ONLY_PID   0x5285

//--------------------------------------------------------------------+
// UF2
//--------------------------------------------------------------------+
#define UF2_PRODUCT_NAME        "Minewsemi ME25LS01 for Meshtastic"
#define UF2_VOLUME_LABEL        "ME25LS01"
#define UF2_BOARD_ID            "nRF52840-ME25LS01-v1.0"
#define UF2_INDEX_URL           "https://store.minewsemi.com/product/me25ls01-bluetooth-module-mx25le01-development-kit/"


#endif /* _ME25LS01_H */
