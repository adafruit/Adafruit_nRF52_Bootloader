/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Benjamin Meisels
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

#ifndef _ARAMCON2_BADGE_H
#define _ARAMCON2_BADGE_H

#define _PINNUM(port, pin)    ((port)*32 + (pin))

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER           1
#define LED_PRIMARY_PIN       _PINNUM(1, 11) // Red

#define LED_NEOPIXEL          _PINNUM(0, 8)
#define NEOPIXELS_NUMBER      2
#define BOARD_RGB_BRIGHTNESS  0x040404

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER        2
#define BUTTON_1              _PINNUM(0, 2) // left Button
#define BUTTON_2              _PINNUM(0, 29) // middle button
#define BUTTON_PULL           NRF_GPIO_PIN_PULLUP

// Used as model string in OTA mode
#define BLEDIS_MANUFACTURER   "ARAMCON Badge Team"
#define BLEDIS_MODEL          "ARAMCON2 Badge"

//--------------------------------------------------------------------+
// USB
//--------------------------------------------------------------------+
#define USB_DESC_VID           0x239A
#define USB_DESC_UF2_PID       0x007B
#define USB_DESC_CDC_ONLY_PID  0x007B

//------------- UF2 -------------//
#define UF2_PRODUCT_NAME   "ARAMCON2 Badge"
#define UF2_VOLUME_LABEL   "ARAMBOOT"
#define UF2_BOARD_ID       "nrf52840-ARAMCON2-Badge"
#define UF2_INDEX_URL      "https://github.com/aramcon-badge"

#endif // _ARAMCON2_BADGE_H
