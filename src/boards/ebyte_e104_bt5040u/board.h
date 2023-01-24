/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 amenekowo, gpshead
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

#ifndef _EBYTE_E104_BT5040U_H
#define _EBYTE_E104_BT5040U_H

#define _PINNUM(port, pin)    ((port)*32 + (pin))

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER           1
#define LED_PRIMARY_PIN       _PINNUM(0, 6)  // Power LED (Near SW button one)
//#define LED_SECONDARY_PIN     _PINNUM(0, 24)  // RGB LED (Near RST button one)
#define LED_STATE_ON          0

//Datasheet: R (red): P0.08; G (green): P1.09; B (blue): P0.12;
#define LED_RGB_RED_PIN       _PINNUM(0, 8)
#define LED_RGB_GREEN_PIN     _PINNUM(1, 9)
#define LED_RGB_BLUE_PIN      _PINNUM(0, 12)
#define BOARD_RGB_BRIGHTNESS  0x404040

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER        2
#define BUTTON_1              _PINNUM(0, 18)  // Silk mark: RST according to nRF52840 pinout
#define BUTTON_2              _PINNUM(1, 6)  // Silk mark: SW according to datasheet
#define BUTTON_PULL           NRF_GPIO_PIN_PULLUP

//--------------------------------------------------------------------+
// BLE OTA
//--------------------------------------------------------------------+
#define BLEDIS_MANUFACTURER   "CDEBYTE"
#define BLEDIS_MODEL          "E104-BT5040U"

//--------------------------------------------------------------------+
// USB
//--------------------------------------------------------------------+

// Shared VID/PID with Feather nRF52840, will be disabled for building in the future
// Dunno it's appropriate, but left here for a while.
#define USB_DESC_VID           0x239A
#define USB_DESC_UF2_PID       0x0029
#define USB_DESC_CDC_ONLY_PID  0x0029

#define UF2_PRODUCT_NAME    "Ebyte E104-BT5040U"
#define UF2_VOLUME_LABEL   "EBT5040U"
#define UF2_BOARD_ID       "10290-v1.0" // Silk on the PCB
#define UF2_INDEX_URL       "https://www.ebyte.com/en/product-view-news.html?id=1185"

#endif // _EBYTE_E104_BT5040U_H
