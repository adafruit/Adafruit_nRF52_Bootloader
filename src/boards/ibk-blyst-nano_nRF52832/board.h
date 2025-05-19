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

#ifndef _IBK_BLYST_NANO_H
#define _IBK_BLYST_NANO_H

#define _PINNUM(port, pin)    ((port)*32 + (pin))

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER				2  // LED3
#define LED_PRIMARY_PIN       	_PINNUM(0, 28)	// LED3 Red
#define LED_SECONDARY_PIN		_PINNUM(0, 29)	// LED2 Green
#define LED_STATE_ON          	0

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER        	2
#define BUTTON_1              	_PINNUM(0, 2)	// S1 switch
#define BUTTON_2              	_PINNUM(0, 13)	// S2 switch
#define BUTTON_PULL           	NRF_GPIO_PIN_PULLUP

/*------------------------------------------------------------------*/
/* UART (only used by nRF52832)
 *------------------------------------------------------------------*/
#define RX_PIN_NUMBER      8
#define TX_PIN_NUMBER      7
#define CTS_PIN_NUMBER     12
#define RTS_PIN_NUMBER     11
#define HWFC               false  // leaving it false to make GPIO available

//--------------------------------------------------------------------+
// BLE OTA
//--------------------------------------------------------------------+
#define BLEDIS_MANUFACTURER   	"I-SYST INC"
#define BLEDIS_MODEL          	"IBK-BLYST-NANO"

//--------------------------------------------------------------------+
// USB
//--------------------------------------------------------------------+
#define USB_DESC_VID           	0x239A
#define USB_DESC_UF2_PID       	0x0063
#define USB_DESC_CDC_ONLY_PID  	0x0064

//------------- UF2 -------------//
#define UF2_PRODUCT_NAME   	"BLYST-NANO Breakout board"
#define UF2_VOLUME_LABEL   	"IBK832_UF2"
#define UF2_BOARD_ID       	"nRF52832-IBK-BLYST-NANO-V0"
#define UF2_INDEX_URL      	"https://www.i-syst.com/products/blyst-nano"

#endif // _IBK_BLYST_NANO_H
