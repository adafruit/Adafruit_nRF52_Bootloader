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

#ifndef USB_DESC_H_
#define USB_DESC_H_

#include "tusb.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define USB_DESC_VID                0x239A

#define USB_DESC_UF2_PID            0x0029
#define USB_DESC_SERIAL_ONLY_PID    0x002A


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
