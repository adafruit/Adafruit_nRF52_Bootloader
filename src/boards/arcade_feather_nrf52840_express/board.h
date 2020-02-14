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

#ifndef _FEATHER_NRF52840_H
#define _FEATHER_NRF52840_H

#define _PINNUM(port, pin)    ((port)*32 + (pin))

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER           2
#define LED_PRIMARY_PIN       _PINNUM(1, 15)
#define LED_SECONDARY_PIN     _PINNUM(1, 10)
#define LED_STATE_ON          1

#define LED_NEOPIXEL           _PINNUM(0, 16)
#define NEOPIXELS_NUMBER      1
#define BOARD_RGB_BRIGHTNESS  0x040404

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER        2
#define BUTTON_1              _PINNUM(1, 02)
#define BUTTON_2              _PINNUM(0, 10)
#define BUTTON_PULL           NRF_GPIO_PIN_PULLUP

/*------------------------------------------------------------------*/
/* UART (only used by nRF52832)
 *------------------------------------------------------------------*/
#define RX_PIN_NUMBER         8
#define TX_PIN_NUMBER         6
#define CTS_PIN_NUMBER        0
#define RTS_PIN_NUMBER        0
#define HWFC                  false

//--------------------------------------------------------------------+
// BLE OTA
//--------------------------------------------------------------------+
#define BLEDIS_MANUFACTURER   "Adafruit Industries"
#define BLEDIS_MODEL          "Feather nRF52840 Express"

//--------------------------------------------------------------------+
// USB
//--------------------------------------------------------------------+
#define USB_DESC_VID           0x239A
#define USB_DESC_UF2_PID       0x0029
#define USB_DESC_CDC_ONLY_PID  0x002A

//------------- UF2 -------------//
#define UF2_PRODUCT_NAME      "Adafruit Feather nRF52840 Express"
#define UF2_VOLUME_LABEL      "ARCADE-N4"
#define UF2_BOARD_ID          "nRF52840-Feather-revD"
#define UF2_INDEX_URL         "https://www.adafruit.com/product/4062"

#ifdef DEFINE_BOOTLOADER_CONFIG
#define BOOTLOADER_CONFIG_PRESENT 1
__attribute__((used, section(".bootloaderConfig")))
const uint32_t bootloaderConfig[] = {
    /* CF2 START */
    513675505, 539130489, // magic
    49, 100,  // used entries, total entries
    4, 0x18, // PIN_BTN_A = PIN_RX
    5, 0xa, // PIN_BTN_B = PIN_D2
    13, 0x29, // PIN_LED = PIN_D13
    18, 0xf, // PIN_MISO = PA15
    19, 0xd, // PIN_MOSI = PA13
    20, 0x23, // PIN_NEOPIXEL = PB03
    21, 0x18, // PIN_RX = PA24
    23, 0xe, // PIN_SCK = PA14
    24, 0xb, // PIN_SCL = PA11
    25, 0xc, // PIN_SDA = PA12
    28, 0x19, // PIN_TX = PA25
    32, 0xe, // PIN_DISPLAY_SCK = PIN_SCK
    34, 0xd, // PIN_DISPLAY_MOSI = PIN_MOSI
    35, 0x1e, // PIN_DISPLAY_CS = PIN_A2
    36, 0x1c, // PIN_DISPLAY_DC = PIN_A3
    37, 0xa0, // DISPLAY_WIDTH = 160
    38, 0x80, // DISPLAY_HEIGHT = 128
    39, 0x80, // DISPLAY_CFG0 = 0x80
    40, 0x603, // DISPLAY_CFG1 = 0x603
    41, 0x20, // DISPLAY_CFG2 = 0x20
    43, 0x2, // PIN_DISPLAY_RST = PIN_A4
    44, 0x3, // PIN_DISPLAY_BL = PIN_A5
    47, 0x6, // PIN_BTN_LEFT = PIN_D11
    48, 0x1a, // PIN_BTN_RIGHT = PIN_D9
    49, 0x1b, // PIN_BTN_UP = PIN_D10
    50, 0x7, // PIN_BTN_DOWN = PIN_D6
    51, 0x8, // PIN_BTN_MENU = PIN_D12
    56, 0x2a, // PIN_LED2 = PB10
    60, 0x19, // PIN_JACK_TX = PIN_TX
    100, 0x4, // PIN_A0 = PA04
    101, 0x5, // PIN_A1 = PA05
    102, 0x1e, // PIN_A2 = PA30
    103, 0x1c, // PIN_A3 = PA28
    104, 0x2, // PIN_A4 = PA02
    105, 0x3, // PIN_A5 = PA03
    152, 0xa, // PIN_D2 = PA10
    155, 0x28, // PIN_D5 = PB08
    156, 0x7, // PIN_D6 = PA07
    159, 0x1a, // PIN_D9 = PA26
    160, 0x1b, // PIN_D10 = PA27
    161, 0x6, // PIN_D11 = PA06
    162, 0x8, // PIN_D12 = PA08
    163, 0x29, // PIN_D13 = PB09
    200, 0x1, // NUM_NEOPIXELS = 1
    204, 0x100000, // FLASH_BYTES = 0x100000
    205, 0x40000, // RAM_BYTES = 0x40000
    208, 0xd7688ea1, // BOOTLOADER_BOARD_ID = 0xd7688ea1
    209, 0xada52840, // UF2_FAMILY = 0xada52840
    210, 0x20, // PINS_PORT_SIZE = PA_32
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    /* CF2 END */
};
#endif

#endif // _FEATHER_NRF52840_H

