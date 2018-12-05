/**************************************************************************/
/*!
    @file     particle_boron.h
    @author   Scott Shawcroft

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2018, Scott Shawcroft for Adafruit Industries (adafruit.com)
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

#ifndef _PARTICLE_XENON_H
#define _PARTICLE_XENON_H

#define _PINNUM(port, pin)    ((port)*32 + (pin))

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
 #define LEDS_NUMBER    1
 #define LED_PRIMARY_PIN           _PINNUM(1, 12)
 #define LED_STATE_ON   1

 #define LED_RGB_RED_PIN           _PINNUM(0, 13)
 #define LED_RGB_GREEN_PIN         _PINNUM(0, 14)
 #define LED_RGB_BLUE_PIN          _PINNUM(0, 15)
 #define BOARD_RGB_BRIGHTNESS 0x202020

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER 2
#define BUTTON_DFU          _PINNUM(0, 11)
#define BUTTON_FRESET       _PINNUM(0, 03) // A0
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

/*------------------------------------------------------------------*/
/* UART
 *------------------------------------------------------------------*/
#define RX_PIN_NUMBER  8
#define TX_PIN_NUMBER  6
#define CTS_PIN_NUMBER 0
#define RTS_PIN_NUMBER 0
#define HWFC           false

// Used as model string in OTA mode
#define DIS_MANUFACTURER  "Particle Industries"
#define DIS_MODEL         "Xenon"

#define VOLUME_LABEL      "XENONBOOT  "

#define BOARD_ID "Particle-Xenon-v1"

#define INDEX_URL "https://www.particle.io/mesh/"

#endif // _PARTICLE_XENON_H
