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

#ifndef OTHERNET_PPR1_H
#define OTHERNET_PPR1_H

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER 2
#define LED_PRIMARY_PIN 25  // green
#define LED_SECONDARY_PIN 11 // red
#define LED_STATE_ON 1

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER 5
#define BUTTON_1 (2)
#define BUTTON_2 (3) 
#define BUTTON_3 (4)
#define BUTTON_4 (5)
#define BUTTON_5 (6)
#define BUTTON_PULL NRF_GPIO_PIN_PULLUP 

/// This is VUSB_EN, we need it forced high to power VBUS_nRF (hw bug)
#define FORCE_HIGH 21

//--------------------------------------------------------------------+
// BLE OTA
//--------------------------------------------------------------------+
#define BLEDIS_MANUFACTURER "othernet"
#define BLEDIS_MODEL "ppr1"

//--------------------------------------------------------------------+
// USB
//--------------------------------------------------------------------+

#define USB_DESC_VID 0x239A
#define USB_DESC_UF2_PID 0x0029
#define USB_DESC_CDC_ONLY_PID 0x0029

#define UF2_PRODUCT_NAME "PPR1"
#define UF2_VOLUME_LABEL "PPR1Boot   "
#define UF2_BOARD_ID "othernet-ppr1"
#define UF2_INDEX_URL "https://othernet.is/ppr1"

#endif
