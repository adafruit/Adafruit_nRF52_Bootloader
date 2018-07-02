/**************************************************************************/
/*!
    @file     tusb_descriptors.h
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, hathach (tinyusb.org)
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
    INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    This file is part of the tinyusb stack.
*/
/**************************************************************************/

#ifndef _TUSB_DESCRIPTORS_H_
#define _TUSB_DESCRIPTORS_H_

#include "tusb.h"

//--------------------------------------------------------------------+
// Descriptors Value (calculated by enabled Classes)
//--------------------------------------------------------------------+
#define CFG_VENDORID            0x239A
#define CFG_PRODUCTID           0x0029


#define ITF_NUM_CDC    0
#define ITF_NUM_MSC    2
#define ITF_NUM_CUS    3

// total number of interfaces
#define ITF_TOTAL      (CFG_TUD_CDC*2 + CFG_TUD_MSC + CFG_TUD_CUSTOM_CLASS)

//--------------------------------------------------------------------+
// Endpoints Address & Max Packet Size
//--------------------------------------------------------------------+
#define EDPT_IN(x)    (0x80 | (x))
#define EDPT_OUT(x)   (x)

#define CDC_EDPT_NOTIF       EDPT_IN (1)
#define CDC_EDPT_NOTIF_SIZE  8

#define CDC_EDPT_OUT         EDPT_OUT(2)
#define CDC_EDPT_IN          EDPT_IN (2)
#define CDC_EDPT_SIZE        64

#define MSC_EDPT_OUT         EDPT_OUT(3)
#define MSC_EDPT_IN          EDPT_IN(3)
#define MSC_EDPT_SIZE        64

#define CUS_EDPT_OUT         EDPT_OUT(4)
#define CUS_EDPT_IN          EDPT_IN(4)
#define CUS_EDPT_SIZE        64


//--------------------------------------------------------------------+
// CONFIGURATION DESCRIPTOR
//--------------------------------------------------------------------+
typedef struct ATTR_PACKED
{
  tusb_desc_configuration_t           configuration;

#if CFG_TUD_CDC
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
#endif

#if CFG_TUD_MSC
  struct ATTR_PACKED
  {
    tusb_desc_interface_t             itf;
    tusb_desc_endpoint_t              ep_out;
    tusb_desc_endpoint_t              ep_in;
  }msc;
#endif

#if CFG_TUD_CUSTOM_CLASS
  struct ATTR_PACKED
  {
    tusb_desc_interface_t             itf;
    tusb_desc_endpoint_t              ep_out;
    tusb_desc_endpoint_t              ep_in;
  }cus;
#endif

} app_descriptor_configuration_t;


extern tud_desc_set_t tud_desc_set;

#endif
