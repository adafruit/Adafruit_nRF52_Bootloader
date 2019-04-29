/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 gpshead (krypto.org) for Adafruit Industries
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

#ifndef _MDK_NRF52840_DONGLE_H_
#define _MDK_NRF52840_DONGLE_H_

#define _PINNUM(port, pin)    ((port)*32 + (pin))

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER           2  // TODO(gpshead): support 0.
#define LED_PRIMARY_PIN       _PINNUM(0, 23)  // Red
#define LED_SECONDARY_PIN     _PINNUM(0, 24)  // Blue
#define LED_STATE_ON          0

//#define LED_RGB_RED_PIN       _PINNUM(0, 23)
//#define LED_RGB_GREEN_PIN     _PINNUM(0, 22)
//#define LED_RGB_BLUE_PIN      _PINNUM(0, 24)
#define BOARD_RGB_BRIGHTNESS  0x404040
/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
// TODO(gpshead): simplify, have code support 0.  double reset only.
#define BUTTONS_NUMBER        2  // none connected at all
#define BUTTON_1              _PINNUM(0, 18)  // unusable: RESET
#define BUTTON_2              _PINNUM(0, 19)  // no connection
#define BUTTON_PULL           NRF_GPIO_PIN_PULLUP

// Used as model string in OTA mode
#define BLEDIS_MANUFACTURER   "MakerDiary"
#define BLEDIS_MODEL          "nRF52840 Micro Dev Kit USB Dongle"

#define UF2_PRODUCT_NAME   "MDK nRF52840 USB Dongle"
#define UF2_VOLUME_LABEL   "MDK840DONGL"
#define UF2_BOARD_ID       "nRF52840-Dongle-v1"
#define UF2_INDEX_URL      "https://wiki.makerdiary.com/nrf52840-mdk-usb-dongle/"

#endif /* _MDK_NRF52840_DONGLE_H_ */
