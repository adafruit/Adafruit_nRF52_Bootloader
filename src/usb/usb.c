/**************************************************************************/
/*!
    @file     usb.c
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2018, Adafruit Industries (adafruit.com)
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

#include "nrfx.h"
#include "nrfx_power.h"

#ifdef SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#include "nrf_soc.h"
#endif

#include "nrf_usbd.h"
#include "tusb.h"
#include "usb_desc.h"

#include "boards.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

// from usb_desc.c for dynamic descriptor
extern tusb_desc_device_t usb_desc_dev;
extern usb_desc_cfg_t     usb_desc_cfg;

// Serial string using unique Device ID
extern uint16_t           usb_desc_str_serial[1+16];

/* tinyusb function that handles power event (detected, ready, removed)
 * We must call it within SD's SOC event handler, or set it as power event handler if SD is not enabled. */
extern void tusb_hal_nrf_power_event(uint32_t event);


//------------- IMPLEMENTATION -------------//
void usb_init(bool cdc_only)
{
  // USB power may already be ready at this time -> no event generated
  // We need to invoke the handler based on the status initially
  uint32_t usb_reg;

#ifdef SOFTDEVICE_PRESENT
  uint8_t sd_en = false;
  (void) sd_softdevice_is_enabled(&sd_en);

  if ( sd_en ) {
    sd_power_usbdetected_enable(true);
    sd_power_usbpwrrdy_enable(true);
    sd_power_usbremoved_enable(true);

    sd_power_usbregstatus_get(&usb_reg);
  }else
#endif
  {
    // Power module init
    const nrfx_power_config_t pwr_cfg = { 0 };
    nrfx_power_init(&pwr_cfg);

    // Register tusb function as USB power handler
    const nrfx_power_usbevt_config_t config = { .handler = (nrfx_power_usb_event_handler_t) tusb_hal_nrf_power_event };
    nrfx_power_usbevt_init(&config);

    nrfx_power_usbevt_enable();

    usb_reg = NRF_POWER->USBREGSTATUS;
  }

  if ( usb_reg & POWER_USBREGSTATUS_VBUSDETECT_Msk ) {
    tusb_hal_nrf_power_event(NRFX_POWER_USB_EVT_DETECTED);
  }

  if ( usb_reg & POWER_USBREGSTATUS_OUTPUTRDY_Msk ) {
    tusb_hal_nrf_power_event(NRFX_POWER_USB_EVT_READY);
  }

  if ( cdc_only )
  {
    // Change PID to CDC only
    usb_desc_dev.idProduct = USB_DESC_SERIAL_ONLY_PID;

    // Remove MSC interface = reduce total interface + adjust config desc length
    usb_desc_cfg.config.bNumInterfaces--;
    usb_desc_cfg.config.wTotalLength -= sizeof(usb_desc_cfg.msc);
  }

  // Create Serial string descriptor
  char tmp_serial[17];
  sprintf(tmp_serial, "%08lX%08lX", NRF_FICR->DEVICEID[1], NRF_FICR->DEVICEID[0]);

  for(uint8_t i=0; i<16; i++)
  {
    usb_desc_str_serial[1+i] = tmp_serial[i];
  }

  // Init tusb stack
  tusb_init();
}

void usb_teardown(void)
{
  if ( NRF_USBD->ENABLE )
  {
    // Abort all transfers

    // Disable pull up
    nrf_usbd_pullup_disable();

    // Disable Interrupt
    NVIC_DisableIRQ(USBD_IRQn);

    // disable all interrupt
    NRF_USBD->INTENCLR = NRF_USBD->INTEN;

    nrf_usbd_disable();
    sd_clock_hfclk_release();

    sd_power_usbdetected_enable(false);
    sd_power_usbpwrrdy_enable(false);
    sd_power_usbremoved_enable(false);
  }
}

//--------------------------------------------------------------------+
// tinyusb callbacks
//--------------------------------------------------------------------+
void tud_mount_cb(void)
{
  led_state(STATE_USB_MOUNTED);
}

void tud_umount_cb(void)
{
  led_state(STATE_USB_UNMOUNTED);
}
