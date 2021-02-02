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

#ifndef _MAKERDIARY_PITAYA_GO_H_
#define _MAKERDIARY_PITAYA_GO_H_

#define _PINNUM(port, pin)    ((port)*32 + (pin))

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER           1
#define LED_PRIMARY_PIN       _PINNUM(1, 10)  // Red
#define LED_STATE_ON          0

#define LED_RGB_RED_PIN       _PINNUM(1, 10)
#define LED_RGB_GREEN_PIN     _PINNUM(1, 11)
#define LED_RGB_BLUE_PIN      _PINNUM(1, 12)
#define BOARD_RGB_BRIGHTNESS  0x404040
/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER        2
#define BUTTON_1              _PINNUM(1, 0)  // P1.00: User Button
#define BUTTON_2              _PINNUM(1, 7)  // P1.07: NC
#define BUTTON_PULL           NRF_GPIO_PIN_PULLUP

//--------------------------------------------------------------------+
// BLE OTA
//--------------------------------------------------------------------+
#define BLEDIS_MANUFACTURER   "Makerdiary"
#define BLEDIS_MODEL          "Pitaya Go"

//--------------------------------------------------------------------+
// USB
//--------------------------------------------------------------------+
#define USB_DESC_VID            0x2886
#define USB_DESC_UF2_PID        0xF00E
#define USB_DESC_CDC_ONLY_PID   0xF00E

//--------------------------------------------------------------------+
// UF2
//--------------------------------------------------------------------+
#define UF2_PRODUCT_NAME        "Pitaya Go"
#define UF2_VOLUME_LABEL        "PITAYAGO"
#define UF2_BOARD_ID            "PITAYAGO"
#define UF2_INDEX_URL           "https://wiki.makerdiary.com/pitaya-go"


#endif /* _MAKERDIARY_PITAYA_GO_H_ */
