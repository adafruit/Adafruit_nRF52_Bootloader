/*
 * The MIT License (MIT)
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

#ifndef _WAVESHARE_NRF52840_H
#define _WAVESHARE_NRF52840_H

#define _PINNUM(port, pin)    ((port)*32 + (pin))

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER           2
/* This is led marked LED1 on the board */
#define LED_PRIMARY_PIN       _PINNUM(0, 13)
/* This is led marked LED2 on the board */
#define LED_SECONDARY_PIN     _PINNUM(0, 14)
/* There are two more LEDs, but for the purposes of bootloader that
 * doesn't matter. */
#define LED_STATE_ON          0

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER        2
/* This is the 'KEY1' in top left of the board under RESET key.
 * It is used to remain in bootloader mode after reset. */
#define BUTTON_1              _PINNUM(0, 11)
/* Pin below is in the bottom left of PCB, and can be shorted to GND
 * with a regular 2.54mm jumper or tweezers. This pin is used to
 * clear user firmware and remain in bootloader mode. */
#define BUTTON_2              _PINNUM(0, 29)
#define BUTTON_PULL           NRF_GPIO_PIN_PULLUP

//--------------------------------------------------------------------+
// BLE OTA
//--------------------------------------------------------------------+
#define BLEDIS_MANUFACTURER   "Waveshare"
#define BLEDIS_MODEL          "nRF52840 Eval"

//--------------------------------------------------------------------+
// USB
//--------------------------------------------------------------------+
#define USB_DESC_VID           0x239A
#define USB_DESC_UF2_PID       0x0029
#define USB_DESC_CDC_ONLY_PID  0x002A

//------------- UF2 -------------//
#define UF2_PRODUCT_NAME      "Waveshare nRF52840 Eval"
#define UF2_VOLUME_LABEL      "WS52840EVK"
#define UF2_BOARD_ID          "WS-nRF52840-EVK"
#define UF2_INDEX_URL         "https://www.waveshare.com/wiki/NRF52840_Eval_Kit"

#endif // _WAVESHARE_NRF52840_H
