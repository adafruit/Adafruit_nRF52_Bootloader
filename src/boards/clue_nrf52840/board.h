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

#ifndef _CLUE_NRF52840_H
#define _CLUE_NRF52840_H

#define _PINNUM(port, pin)    ((port)*32 + (pin))

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER           1
#define LED_PRIMARY_PIN       _PINNUM(1, 01)
#define LED_STATE_ON          1

#define LED_NEOPIXEL          _PINNUM(0, 16)
#define NEOPIXELS_NUMBER      1
#define BOARD_RGB_BRIGHTNESS  0x040404

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER        2
#define BUTTON_1              _PINNUM(1, 02)     // left button
#define BUTTON_2              _PINNUM(1, 10)     // right button
#define BUTTON_PULL           NRF_GPIO_PIN_PULLUP

//--------------------------------------------------------------------+
// Display
//--------------------------------------------------------------------+
#define DISPLAY_CONTROLLER_ST7789

#define DISPLAY_PIN_SCK       _PINNUM(0, 14)
#define DISPLAY_PIN_MOSI      _PINNUM(0, 15)

#define DISPLAY_PIN_CS        _PINNUM(0, 12)
#define DISPLAY_PIN_DC        _PINNUM(0, 13)
#define DISPLAY_PIN_RST       _PINNUM(1,  3)
#define DISPLAY_PIN_BL        _PINNUM(1,  5)
#define DISPLAY_BL_ON         1  // GPIO state to enable back light

#define DISPLAY_WIDTH         240
#define DISPLAY_HEIGHT        240

#define DISPLAY_COL_OFFSET    0
#define DISPLAY_ROW_OFFSET    80

// Memory Data Access Control & // Vertical Scroll Start Address
#define DISPLAY_MADCTL        (TFT_MADCTL_MY)
#define DISPLAY_VSCSAD        0

#define DISPLAY_TITLE         "CLUE"

//--------------------------------------------------------------------+
// BLE OTA
//--------------------------------------------------------------------+
#define BLEDIS_MANUFACTURER   "Adafruit Industries"
#define BLEDIS_MODEL          "CLUE nRF52840"

//--------------------------------------------------------------------+
// USB
//--------------------------------------------------------------------+
#define USB_DESC_VID           0x239A
#define USB_DESC_UF2_PID       0x0071
#define USB_DESC_CDC_ONLY_PID  0x0071

//------------- UF2 -------------//
#define UF2_PRODUCT_NAME   "Adafruit CLUE nRF52840"
#define UF2_VOLUME_LABEL   "CLUEBOOT"
#define UF2_BOARD_ID       "nRF52840-CLUE-revA"
#define UF2_INDEX_URL      "https://www.adafruit.com/product/4500"

#endif // _CLUE_NRF52840_H
