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

#ifndef _FEATHER_NRF52840_SENSE_TFT_H
#define _FEATHER_NRF52840_SENSE_TFT_H

#define _PINNUM(port, pin)    ((port)*32 + (pin))

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER           1
#define LED_PRIMARY_PIN       _PINNUM(0, 11)
#define LED_STATE_ON          1

#define LED_NEOPIXEL          _PINNUM(1, 8)
#define NEOPIXEL_POWER_PIN    _PINNUM(1, 2)
#define NEOPIXELS_NUMBER      1
#define BOARD_RGB_BRIGHTNESS  0x040404

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER        2
#define BUTTON_1              _PINNUM(1, 6)
#define BUTTON_2              _PINNUM(1, 10) // DFU pin
#define BUTTON_PULL           NRF_GPIO_PIN_PULLUP

//--------------------------------------------------------------------+
// Display
//--------------------------------------------------------------------+

// VSensor required to power the display
#define DISPLAY_VSENSOR_PIN   _PINNUM(1, 7)
#define DISPLAY_VSENSOR_ON    1

#define DISPLAY_CONTROLLER_ST7789

#define DISPLAY_PIN_SCK       _PINNUM(0, 26)
#define DISPLAY_PIN_MOSI      _PINNUM(0,  5)

#define DISPLAY_PIN_CS        _PINNUM(1,  5)
#define DISPLAY_PIN_DC        _PINNUM(1,  1)
#define DISPLAY_PIN_RST       _PINNUM(1,  3)
#define DISPLAY_PIN_BL        _PINNUM(0, 27)
#define DISPLAY_BL_ON         1  // GPIO state to enable back light

#define DISPLAY_WIDTH         240
#define DISPLAY_HEIGHT        135

#define DISPLAY_COL_OFFSET    53
#define DISPLAY_ROW_OFFSET    40

// Memory Data Access Control & // Vertical Scroll Start Address
#define DISPLAY_MADCTL        (TFT_MADCTL_MX)
#define DISPLAY_VSCSAD        0

#define DISPLAY_TITLE         "Sense TFT"

//--------------------------------------------------------------------+
// BLE OTA
//--------------------------------------------------------------------+
#define BLEDIS_MANUFACTURER   "Adafruit Industries"
#define BLEDIS_MODEL          "Feather nRF52840 Sense TFT"

//--------------------------------------------------------------------+
// USB
//--------------------------------------------------------------------+
#define USB_DESC_VID           0x239A
#define USB_DESC_UF2_PID       0x0087
#define USB_DESC_CDC_ONLY_PID  0x0088

//------------- UF2 -------------//
#define UF2_PRODUCT_NAME      "Adafruit Feather nRF52840 Sense TFT"
#define UF2_VOLUME_LABEL      "SENSTFTBOOT"
#define UF2_BOARD_ID          "nRF52840-FeatherSenseTFT-revA"
#define UF2_INDEX_URL         "https://www.adafruit.com/product/"

#endif
