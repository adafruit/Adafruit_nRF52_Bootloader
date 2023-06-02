#ifndef _NRF5340_DK_H
#define _NRF5340_DK_H

#define _PINNUM(port, pin)    ((port)*32 + (pin))

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LED_STATE_ON          0
#define LED_STATE_OFF         1

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTON_1              _PINNUM(0, 23)
#define BUTTON_PULL           NRF_GPIO_PIN_PULLUP

#define LED_1              _PINNUM(0, 28)
#define LED_2              _PINNUM(0, 29)

//--------------------------------------------------------------------+
// BLE OTA
//--------------------------------------------------------------------+
#define BLEDIS_MANUFACTURER   "HID"
#define BLEDIS_MODEL          "HID"

//--------------------------------------------------------------------+
// USB
//--------------------------------------------------------------------+
#define USB_DESC_VID           0x239A
#define USB_DESC_UF2_PID       0x0029
#define USB_DESC_CDC_ONLY_PID  0x002A

//------------- UF2 -------------//
#define UF2_PRODUCT_NAME      "nRF5340_DK"
#define UF2_VOLUME_LABEL      "FTHR840BOOT"
#define UF2_BOARD_ID          "nRF5340-DK"
#define UF2_INDEX_URL         "https://www.nordicsemi.com/Products/Development-hardware/nRF5340-DK"

#endif // _NRF5340_DK_H
