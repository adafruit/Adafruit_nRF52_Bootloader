/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2026 Ambraglow
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

#ifndef _SYLDRA_H
#define _SYLDRA_H

//------------------------------------------------------------------//
// LED
//------------------------------------------------------------------//
#define LEDS_NUMBER             1
#define LED_PRIMARY_PIN         PINNUM(0, 17) // AD10 LED
#define LED_STATE_ON            1

//------------------------------------------------------------------//
// BUTTON
//------------------------------------------------------------------//
#define BUTTON_DFU     PINNUM(0, 24) // AD20 -- DFU
#define BUTTON_DFU_OTA PINNUM(1, 00) // AD22 -- OTA

#define BUTTON_PULL             NRF_GPIO_PIN_PULLUP

//-------------------------------------------------------------------//
// BLE OTA
//-------------------------------------------------------------------//
#define BLEDIS_MANUFACTURER     "Ambraglow-&-localcc"
#define BLEDIS_MODEL            "Syldra_dev_r1"

//--------------------------------------------------------------------+
// USB - using test PIDs
//--------------------------------------------------------------------+
#define USB_DESC_VID           0x1209
#define USB_DESC_UF2_PID       0x0001
#define USB_DESC_CDC_ONLY_PID  0x7A01

#define UF2_PRODUCT_NAME        "Syldra nrf52840 Development Board"
#define UF2_VOLUME_LABEL        "Syldra-boot"
#define UF2_BOARD_ID            "nRF52840-Syldra-dev-v1"
#define UF2_INDEX_URL           "https://github.com/ambraglow/nonorf"

#endif