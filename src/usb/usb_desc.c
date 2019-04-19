/**************************************************************************/
/*!
    @file     usb_desc.c
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
    INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    This file is part of the tinyusb stack.
*/
/**************************************************************************/

#include "usb_desc.h"


/*------------- Interface Numbering -------------*/
enum {
    ITF_NUM_CDC = 0  ,
    ITF_NUM_CDC_DATA ,
    ITF_NUM_MSC      ,
    ITF_NUM_TOTAL
};

enum {
    ITF_STR_LANGUAGE = 0 ,
    ITF_STR_MANUFACTURER ,
    ITF_STR_PRODUCT      ,
    ITF_STR_SERIAL       ,
    ITF_STR_CDC          ,
    ITF_STR_MSC
};

/*------------- Endpoint Numbering & Size -------------*/
#define _EP_IN(x)          (0x80 | (x))
#define _EP_OUT(x)         (x)

// CDC
#define EP_CDC_NOTIF       _EP_IN ( ITF_NUM_CDC+1 )
#define EP_CDC_NOTIF_SIZE  8

#define EP_CDC_OUT         _EP_OUT( ITF_NUM_CDC+2 )
#define EP_CDC_IN          _EP_IN ( ITF_NUM_CDC+2 )

// Mass Storage
#define EP_MSC_OUT         _EP_OUT( ITF_NUM_MSC+1 )
#define EP_MSC_IN          _EP_IN ( ITF_NUM_MSC+1 )

#define EP_MSC_SIZE        64

//--------------------------------------------------------------------+
// STRING DESCRIPTORS
//--------------------------------------------------------------------+

#ifndef USB_STRING_DESCRIPTORS
#define USB_STRING_DESCRIPTORS {                                                                                 \
    /* 0: is supported language = English */                                                                        \
    TUD_DESC_STRCONV(0x0409),                                                                                       \
                                                                                                                    \
    /* 1: Manufacturer */                                                                                           \
    TUD_DESC_STRCONV('A','d','a','f','r','u','i','t',' ','I','n','d','u','s','t','r','i','e','s'),                  \
                                                                                                                    \
    /* 2: Product */                                                                                                \
    TUD_DESC_STRCONV('B','l','u','e','f','r','u','i','t',' ','n','R','F','5','2','8','4','0', ' ', 'D','F','U'),    \
                                                                                                                    \
    /* 3: Serials TODO use chip ID */                                                                               \
    usb_desc_str_serial,                                                                                            \
                                                                                                                    \
    /* 4: CDC Interface */                                                                                          \
    TUD_DESC_STRCONV('B','l','u','e','f','r','u','i','t',' ','S','e','r','i','a','l'),                              \
                                                                                                                    \
    /* 5: MSC Interface */                                                                                          \
    TUD_DESC_STRCONV('B','l','u','e','f','r','u','i','t',' ','U','F','2'),                                          \
}
#endif

// Serial is 64-bit DeviceID -> 16 chars len
uint16_t usb_desc_str_serial[1+16] = { TUD_DESC_STR_HEADER(16) };

// array of pointer to string descriptors
uint16_t const * const string_desc_arr [] = USB_STRING_DESCRIPTORS;

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+
tusb_desc_device_t usb_desc_dev =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // Use Interface Association Descriptor (IAD) for CDC
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDOINT0_SIZE,

    .idVendor           = USB_DESC_VID,
    .idProduct          = USB_DESC_UF2_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
usb_desc_cfg_t usb_desc_cfg =
{
    .config =
    {
        .bLength             = sizeof(tusb_desc_configuration_t),
        .bDescriptorType     = TUSB_DESC_CONFIGURATION,
        .wTotalLength        = sizeof(usb_desc_cfg_t),
        .bNumInterfaces      = ITF_NUM_TOTAL,
        .bConfigurationValue = 1,
        .iConfiguration      = 0x00,
        .bmAttributes        = TUSB_DESC_CONFIG_ATT_BUS_POWER,
        .bMaxPower           = TUSB_DESC_CONFIG_POWER_MA(100)
    },

    // IAD points to CDC Interfaces
    .cdc =
    {
      .iad =
      {
          .bLength           = sizeof(tusb_desc_interface_assoc_t),
          .bDescriptorType   = TUSB_DESC_INTERFACE_ASSOCIATION,

          .bFirstInterface   = ITF_NUM_CDC,
          .bInterfaceCount   = 2,

          .bFunctionClass    = TUSB_CLASS_CDC,
          .bFunctionSubClass = CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL,
          .bFunctionProtocol = CDC_COMM_PROTOCOL_ATCOMMAND,
          .iFunction         = 0
      },

      //------------- CDC Communication Interface -------------//
      .comm_itf =
      {
          .bLength            = sizeof(tusb_desc_interface_t),
          .bDescriptorType    = TUSB_DESC_INTERFACE,
          .bInterfaceNumber   = ITF_NUM_CDC,
          .bAlternateSetting  = 0,
          .bNumEndpoints      = 1,
          .bInterfaceClass    = TUSB_CLASS_CDC,
          .bInterfaceSubClass = CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL,
          .bInterfaceProtocol = CDC_COMM_PROTOCOL_ATCOMMAND,
          .iInterface         = ITF_STR_CDC
      },

      .header =
      {
          .bLength            = sizeof(cdc_desc_func_header_t),
          .bDescriptorType    = TUSB_DESC_CLASS_SPECIFIC,
          .bDescriptorSubType = CDC_FUNC_DESC_HEADER,
          .bcdCDC             = 0x0120
      },

      .call =
      {
          .bLength            = sizeof(cdc_desc_func_call_management_t),
          .bDescriptorType    = TUSB_DESC_CLASS_SPECIFIC,
          .bDescriptorSubType = CDC_FUNC_DESC_CALL_MANAGEMENT,
          .bmCapabilities     = { 0 },
          .bDataInterface     = ITF_NUM_CDC+1,
      },

      .acm =
      {
          .bLength            = sizeof(cdc_desc_func_acm_t),
          .bDescriptorType    = TUSB_DESC_CLASS_SPECIFIC,
          .bDescriptorSubType = CDC_FUNC_DESC_ABSTRACT_CONTROL_MANAGEMENT,
          .bmCapabilities     = { // 0x02
              .support_line_request = 1,
          }
      },

      .union_func =
      {
          .bLength                  = sizeof(cdc_desc_func_union_t), // plus number of
          .bDescriptorType          = TUSB_DESC_CLASS_SPECIFIC,
          .bDescriptorSubType       = CDC_FUNC_DESC_UNION,
          .bControlInterface        = ITF_NUM_CDC,
          .bSubordinateInterface    = ITF_NUM_CDC+1,
      },

      .ep_notif =
      {
          .bLength          = sizeof(tusb_desc_endpoint_t),
          .bDescriptorType  = TUSB_DESC_ENDPOINT,
          .bEndpointAddress = EP_CDC_NOTIF,
          .bmAttributes     = { .xfer = TUSB_XFER_INTERRUPT },
          .wMaxPacketSize   = { .size = EP_CDC_NOTIF_SIZE },
          .bInterval        = 0x10
      },

      //------------- CDC Data Interface -------------//
      .data_itf =
      {
          .bLength            = sizeof(tusb_desc_interface_t),
          .bDescriptorType    = TUSB_DESC_INTERFACE,
          .bInterfaceNumber   = ITF_NUM_CDC+1,
          .bAlternateSetting  = 0x00,
          .bNumEndpoints      = 2,
          .bInterfaceClass    = TUSB_CLASS_CDC_DATA,
          .bInterfaceSubClass = 0,
          .bInterfaceProtocol = 0,
          .iInterface         = 0x00
      },

      .ep_out =
      {
          .bLength          = sizeof(tusb_desc_endpoint_t),
          .bDescriptorType  = TUSB_DESC_ENDPOINT,
          .bEndpointAddress = EP_CDC_OUT,
          .bmAttributes     = { .xfer = TUSB_XFER_BULK },
          .wMaxPacketSize   = { .size = CFG_TUD_CDC_EPSIZE },
          .bInterval        = 0
      },

      .ep_in =
      {
          .bLength          = sizeof(tusb_desc_endpoint_t),
          .bDescriptorType  = TUSB_DESC_ENDPOINT,
          .bEndpointAddress = EP_CDC_IN,
          .bmAttributes     = { .xfer = TUSB_XFER_BULK },
          .wMaxPacketSize   = { .size = CFG_TUD_CDC_EPSIZE },
          .bInterval        = 0
      },
    },

    //------------- Mass Storage-------------//
    .msc =
    {
      .itf =
      {
          .bLength            = sizeof(tusb_desc_interface_t),
          .bDescriptorType    = TUSB_DESC_INTERFACE,
          .bInterfaceNumber   = ITF_NUM_MSC,
          .bAlternateSetting  = 0x00,
          .bNumEndpoints      = 2,
          .bInterfaceClass    = TUSB_CLASS_MSC,
          .bInterfaceSubClass = MSC_SUBCLASS_SCSI,
          .bInterfaceProtocol = MSC_PROTOCOL_BOT,
          .iInterface         = ITF_STR_MSC
      },

      .ep_out =
      {
          .bLength          = sizeof(tusb_desc_endpoint_t),
          .bDescriptorType  = TUSB_DESC_ENDPOINT,
          .bEndpointAddress = EP_MSC_OUT,
          .bmAttributes     = { .xfer = TUSB_XFER_BULK },
          .wMaxPacketSize   = { .size = EP_MSC_SIZE},
          .bInterval        = 1
      },

      .ep_in =
      {
          .bLength          = sizeof(tusb_desc_endpoint_t),
          .bDescriptorType  = TUSB_DESC_ENDPOINT,
          .bEndpointAddress = EP_MSC_IN,
          .bmAttributes     = { .xfer = TUSB_XFER_BULK },
          .wMaxPacketSize   = { .size = EP_MSC_SIZE },
          .bInterval        = 1
      }
    }
};


// tud_desc_set is required by tinyusb stack
// since CFG_TUD_DESC_AUTO is enabled, we only need to set string_arr 
tud_desc_set_t tud_desc_set =
{
    .device       = &usb_desc_dev,
    .config       = &usb_desc_cfg,
    .string_arr   = (uint8_t const **) string_desc_arr,
    .string_count = sizeof(string_desc_arr)/sizeof(string_desc_arr[0]),

    .hid_report =
    {
        .generic       = NULL,
        .boot_keyboard = NULL,
        .boot_mouse    = NULL
    }
};
