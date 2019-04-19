/**************************************************************************/
/*!
    @file     usb_desc.h
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, hathach (tinyusb.org)
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

#ifndef USB_DESC_H_
#define USB_DESC_H_

#include "tusb.h"
#include "boards.h"

#ifdef __cplusplus
 extern "C" {
#endif

#ifndef USB_DESC_VID
#define USB_DESC_VID                0x239A
#endif

#ifndef USB_DESC_UF2_PID
#define USB_DESC_UF2_PID            0x0029
#endif

#ifndef USB_DESC_SERIAL_ONLY_PID
#define USB_DESC_SERIAL_ONLY_PID    0x002A
#endif


/*------------- Configuration Descriptor -------------*/
typedef struct ATTR_PACKED
{
  tusb_desc_configuration_t           config;

  //------------- CDC -------------//
  struct ATTR_PACKED
  {
    tusb_desc_interface_assoc_t       iad;

    //CDC Control Interface
    tusb_desc_interface_t             comm_itf;
    cdc_desc_func_header_t            header;
    cdc_desc_func_call_management_t   call;
    cdc_desc_func_acm_t               acm;
    cdc_desc_func_union_t             union_func;
    tusb_desc_endpoint_t              ep_notif;

    //CDC Data Interface
    tusb_desc_interface_t             data_itf;
    tusb_desc_endpoint_t              ep_out;
    tusb_desc_endpoint_t              ep_in;
  }cdc;

  //------------- Mass Storage -------------//
  struct ATTR_PACKED
  {
    tusb_desc_interface_t             itf;
    tusb_desc_endpoint_t              ep_out;
    tusb_desc_endpoint_t              ep_in;
  } msc;
} usb_desc_cfg_t;


#ifdef __cplusplus
 }
#endif

#endif /* USB_DESC_H_ */
