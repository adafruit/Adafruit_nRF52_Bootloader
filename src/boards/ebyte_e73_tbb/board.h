/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Pierre Constantineau
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

#ifndef _EBYTE_E73_TBB_H
#define _EBYTE_E73_TBB_H

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER        2
#define LED_PRIMARY_PIN    17 // Red  - named "LED1"
#define LED_SECONDARY_PIN  18 // Blue - named "LED2" (also a red one on the Test Board)
#define LED_STATE_ON       0

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER     2
#define BUTTON_1           14 // BUTTON_1 is DFU.  SW1 on E73-TBB Test Board. 
                              // This Test board uses the E73-2G4M04S1B module which contains a nrf52832. 
                              // The module does not have reset circuitry. 
                              // The Test Board from Ebyte (E73-TBB) also does not have reset circuitry
                              // To be able to upload using the Arduino IDE, (To enter DFU mode)
                              // Press "SW1", momentarily ground "RST/P0.21", then release "SW1"
#define BUTTON_2           13 // SW2 on board
#define BUTTON_PULL        NRF_GPIO_PIN_PULLUP

/*------------------------------------------------------------------*/
/* UART (only used by nRF52832)
 *------------------------------------------------------------------*/
#define RX_PIN_NUMBER      8
#define TX_PIN_NUMBER      6
#define CTS_PIN_NUMBER     5
#define RTS_PIN_NUMBER     7
#define HWFC               false  // leaving it false to make GPIO available

//--------------------------------------------------------------------+
// BLE OTA
//--------------------------------------------------------------------+
#define BLEDIS_MANUFACTURER "CDEBYTE"
#define BLEDIS_MODEL        "E73-TBB"

#define UF2_PRODUCT_NAME    "Ebyte E73-TBB nRF52832"
#define UF2_INDEX_URL       "https://www.ebyte.com/en/product-view-news.aspx?id=889"

#endif // _EBYTE_E73_TBB_H
