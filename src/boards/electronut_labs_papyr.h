/**************************************************************************/
/*!
    @file     elelctronutlabs_papyr.h
    @author   Tavish Naruka <tavish@electronut.in>

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2017, Adafruit Industries (adafruit.com)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#ifndef PAPYR_H
#define PAPYR_H

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER         2
#define LED_PRIMARY_PIN     13
#define LED_SECONDARY_PIN   14
#define LED_STATE_ON        0

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

/*------------------------------------------------------------------*/
/* UART
 *------------------------------------------------------------------*/
#define RX_PIN_NUMBER       7
#define TX_PIN_NUMBER       8
#define CTS_PIN_NUMBER      0
#define RTS_PIN_NUMBER      0
#define HWFC                false

// Used as model string in OTA mode
#define DIS_MANUFACTURER    "Electronut Labs"
#define DIS_MODEL           "Papyr"

#define BOARD_ID            "Electronut Labs Papyr"
#define INDEX_URL           "https://docs.electronut.in/papyr"

#define USB_DESC_VID        0x239A
#define USB_DESC_UF2_PID    0x003B

#define USB_STRING_DESCRIPTORS {                                                                                    \
    /* 0: is supported language = English */                                                                        \
    TUD_DESC_STRCONV(0x0409),                                                                                       \
                                                                                                                    \
    /* 1: Manufacturer */                                                                                           \
    TUD_DESC_STRCONV('E','l','e','c','t','r','o','n','u','t',' ','L','a','b','s'),                                  \
                                                                                                                    \
    /* 2: Product */                                                                                                \
    TUD_DESC_STRCONV('P','a','p','y','r', ' ', 'D','F','U'),                                                        \
                                                                                                                    \
    /* 3: Serials TODO use chip ID */                                                                               \
    usb_desc_str_serial,                                                                                            \
                                                                                                                    \
    /* 4: CDC Interface */                                                                                          \
    TUD_DESC_STRCONV('P','a','p','y','r',' ','S','e','r','i','a','l'),                                              \
                                                                                                                    \
    /* 5: MSC Interface */                                                                                          \
    TUD_DESC_STRCONV('P','a','p','y','r',' ','U','F','2'),                                                          \
}

#endif // PPAPYR_H
