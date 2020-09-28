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

#ifndef TTGO_EINK_H
#define TTGO_EINK_H

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER 2
#define LED_PRIMARY_PIN 13   // red
#define LED_SECONDARY_PIN 15 // blue
#define LED_STATE_ON 0

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER 2
#define BUTTON_1 (32 + 10)
#define BUTTON_2 (0 + 18) // We repurpose the RESET button as button 2
#define BUTTON_PULL NRF_GPIO_PIN_NOPULL 
#define BUTTON_SENSE NRF_GPIO_PIN_PULLUP

//--------------------------------------------------------------------+
// BLE OTA
//--------------------------------------------------------------------+
#define BLEDIS_MANUFACTURER "TTGO"
#define BLEDIS_MODEL "eink"

//--------------------------------------------------------------------+
// USB
//--------------------------------------------------------------------+

#define USB_DESC_VID 0x239A
#define USB_DESC_UF2_PID 0x0029
#define USB_DESC_CDC_ONLY_PID 0x0029

#define UF2_PRODUCT_NAME "TTGO"
#define UF2_VOLUME_LABEL "EinkBoot   "
#define UF2_BOARD_ID "ttgo-eink"
#define UF2_INDEX_URL "https://www.meshtastic.org"

#endif
