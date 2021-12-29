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

#ifndef RAYTAC_MDBT50Q_DB_40_H
#define RAYTAC_MDBT50Q_DB_40_H

#define _PINNUM(port, pin)      ((port)*32 + (pin))

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER             3
/* This is the Green led marked D1 on the board */
#define LED_PRIMARY_PIN         _PINNUM(1, 13)
/* This is the Red led marked D2 on the board */
#define LED_SECONDARY_PIN  	_PINNUM(1, 14)
/* This is the Blue led marked D3 on the board */
#define LED_TERTIARY_PIN  	_PINNUM(1, 15)
/* There are one more unpopulated LEDs, but for the purposes of bootloader that
 * doesn't matter. */
#define LED_STATE_ON            0

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER          4

#define BUTTON_1                _PINNUM(0, 11)
#define BUTTON_2                _PINNUM(0, 12)
#define BUTTON_3                _PINNUM(0, 24)
#define BUTTON_4                _PINNUM(0, 25)
#define BUTTON_PULL             NRF_GPIO_PIN_PULLUP

//--------------------------------------------------------------------+
// BLE OTA
//--------------------------------------------------------------------+
#define BLEDIS_MANUFACTURER     "Raytac"
#define BLEDIS_MODEL            "MDBT50Q-DB-40"

//--------------------------------------------------------------------+
// USB
//--------------------------------------------------------------------+

#define USB_DESC_VID           0x239A
#define USB_DESC_UF2_PID       0x00BB
#define USB_DESC_CDC_ONLY_PID  0x00BB

//------------- UF2 -------------//
#define UF2_PRODUCT_NAME        "Raytac MDBT50Q Demo Board 40"
#define UF2_VOLUME_LABEL        "MDBT50QBOOT"
#define UF2_BOARD_ID            "nRF52840-MDBT50Q-1MV2"
#define UF2_INDEX_URL           "https://www.raytac.com/product/ins.php?index_id=81"

#endif // RAYTAC_MDBT50Q_DB_40_H
