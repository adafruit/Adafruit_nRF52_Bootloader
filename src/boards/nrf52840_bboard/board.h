/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 ddB0515(Dario Budimir)
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

#ifndef _NRF52840_BBOARD_H
#define _NRF52840_BBOARD_H

#define _PINNUM(port, pin)    ((port)*32 + (pin))

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER           1
#define LED_PRIMARY_PIN       _PINNUM(0, 7)
#define LED_STATE_ON          1

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
// NOTE: USB UF2 bootloader can be triggered by double pressing RESET
// App can trigger OTA bootloader by writing DFU_MAGIC_OTA_APPJUM to
// GPREGRET register if softdevice is not initialized; or by
// writing DFU_MAGIC_OTA_RESET in case softdevice is initialized.
#define BUTTONS_NUMBER      2
#define BUTTON_1            18 // RESET also by default
#define BUTTON_2            1  // P0.1 not exposed anywhere, FRST n/a
#define BUTTON_PULL         NRF_GPIO_PIN_PULLUP

//--------------------------------------------------------------------+
// BLE OTA
//--------------------------------------------------------------------+
#define BLEDIS_MANUFACTURER   "ddB0515"
#define BLEDIS_MODEL          "nRF52840 BBoard"

//--------------------------------------------------------------------+
// USB
//--------------------------------------------------------------------+
#define USB_DESC_VID           0x1d50
#define USB_DESC_UF2_PID       0x6157
#define USB_DESC_CDC_ONLY_PID  0x6157

//------------- UF2 -------------//
#define UF2_PRODUCT_NAME   "nRF52840 BBoard"
#define UF2_VOLUME_LABEL   "BBOARDBOOT"
#define UF2_BOARD_ID       "nRF52840-BBoard-rev2"
#define UF2_INDEX_URL      "https://github.com/ddB0515/nRF52840-BBoard"

#endif // _NRF52840_BBOARD_H
