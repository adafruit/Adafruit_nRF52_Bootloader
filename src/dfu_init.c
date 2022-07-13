/**
 * Copyright (c) 2017 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**@file
 *
 * @defgroup nrf_dfu_init_template Template file with an DFU init packet handling example.
 * @{
 *
 * @ingroup nrf_dfu
 *
 * @brief This file contains a template on how to implement DFU init packet handling.
 *
 * @details The template shows how device type and revision can be used for a safety check of the 
 *          received image. It shows how validation can be performed in two stages:
 *          - Stage 1: Pre-check of firmware image before transfer to ensure the firmware matches:
 *                     - Device Type.
 *                     - Device Revision.
 *                     Installed SoftDevice.
 *                     This template can be extended with additional checks according to needs.
 *                     For example, such a check could be the origin of the image (trusted source) 
 *                     based on a signature scheme.
 *          - Stage 2: Post-check of the image after image transfer but before installing firmware.
 *                     For example, such a check could be an integrity check in form of hashing or 
 *                     verification of a signature.
 *                     In this template, a simple CRC check is carried out.
 *                     The CRC check can be replaced with other mechanisms, like signing.
 *
 * @note This module does not support security features such as image signing, but the 
 *       implementation allows for such extension.
 *       If the init packet is signed by a trusted source, it must be decrypted before it can be
 *       processed.
 */

#include "dfu_init.h"
#include <stdint.h>
#include <string.h>
#include <dfu_types.h>
#include "nrf_error.h"
#include "crc16.h"

/* ADAFRUIT
 * - All firmware init data must has Device Type ADAFRUIT_DEVICE_TYPE (nrf52832 and nrf52840)
 * - SD + Bootloader upgrade must have correct Device Revision to make sure bootloader is not flashed
 * on the wrong device (e.g flah nRF52832's bootloader on nRF52840 board and vice versa)
 *   - nrf52832 dev-rev is 0xADAF
 *   - nrf52840 dev-rev is  52840
 */
#define ADAFRUIT_DEVICE_TYPE         0x0052

#if defined(NRF52840_XXAA)
  #define ADAFRUIT_DEV_REV           52840
#elif defined(NRF52833_XXAA)
  #define ADAFRUIT_DEV_REV           52833
#elif defined NRF52832_XXAA
  #define ADAFRUIT_DEV_REV           0xADAF
#else
  #error Unknown MCU
#endif



#define DFU_INIT_PACKET_EXT_LENGTH_MIN      2                       //< Minimum length of the extended init packet. The extended init packet may contain a CRC, a HASH, or other data. This value must be changed according to the requirements of the system. The template uses a minimum value of two in order to hold a CRC. */
#define DFU_INIT_PACKET_EXT_LENGTH_MAX      10                      //< Maximum length of the extended init packet. The extended init packet may contain a CRC, a HASH, or other data. This value must be changed according to the requirements of the system. The template uses a maximum value of 10 in order to hold a CRC and any padded data on transport layer without overflow. */

static uint8_t m_extended_packet[DFU_INIT_PACKET_EXT_LENGTH_MAX];   //< Data array for storage of the extended data received. The extended data follows the normal init data of type \ref dfu_init_packet_t. Extended data can be used for a CRC, hash, signature, or other data. */
static uint8_t m_extended_packet_length;                            //< Length of the extended data received with init packet. */


uint32_t dfu_init_prevalidate(uint8_t * p_init_data, uint32_t init_data_len, uint8_t image_type)
{
    uint32_t i = 0;
    
    // In order to support signing or encryption then any init packet decryption function / library
    // should be called from here or implemented at this location.

    // Length check to ensure valid data are parsed.
    if (init_data_len < sizeof(dfu_init_packet_t))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    // Current template uses clear text data so they can be casted for pre-check.
    dfu_init_packet_t * p_init_packet = (dfu_init_packet_t *)p_init_data;

    m_extended_packet_length = ((uint32_t)p_init_data + init_data_len) -
                               (uint32_t)&p_init_packet->softdevice[p_init_packet->softdevice_len];
    if (m_extended_packet_length < DFU_INIT_PACKET_EXT_LENGTH_MIN)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    if (((uint32_t)p_init_data + init_data_len) < 
        (uint32_t)&p_init_packet->softdevice[p_init_packet->softdevice_len])
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    memcpy(m_extended_packet,
           &p_init_packet->softdevice[p_init_packet->softdevice_len],
           m_extended_packet_length);

    /** [DFU init application version] */
    // To support application versioning, this check should be updated.
    // This template allows for any application to be installed. However, 
    // customers can place a revision number at the bottom of the application 
    // to be verified by the bootloader. This can be done at a location 
    // relative to the application, for example the application start 
    // address + 0x0100.
    /** [DFU init application version] */
    
    // First check to verify the image to be transfered matches the device type.
    // If no Device type is present in DFU_DEVICE_INFO then any image will be accepted.
//    if ((DFU_DEVICE_INFO->device_type != DFU_DEVICE_TYPE_EMPTY) &&
//        (p_init_packet->device_type != DFU_DEVICE_INFO->device_type))
//    {
//        return NRF_ERROR_INVALID_DATA;
//    }
    
    // Second check to verify the image to be transfered matches the device revision.
    // If no Device revision is present in DFU_DEVICE_INFO then any image will be accepted.
    // if ((DFU_DEVICE_INFO->device_rev != DFU_DEVICE_REVISION_EMPTY) &&
    //    (p_init_packet->device_rev != DFU_DEVICE_INFO->device_rev))

    if ( p_init_packet->device_type != ADAFRUIT_DEVICE_TYPE )
    {
        return NRF_ERROR_FORBIDDEN;
    }

    // Adafruit unlock code must match to upgrade SoftDevice and/or Bootloader
    if ( image_type & (DFU_UPDATE_SD | DFU_UPDATE_BL) )
    {
      if (p_init_packet->device_rev != ADAFRUIT_DEV_REV)
      {
        return NRF_ERROR_FORBIDDEN;
      }
    }

    // Third check: Check the array of supported SoftDevices by this application.
    //              If the installed SoftDevice does not match any SoftDevice in the list then an
    //              error is returned.
    while (i < p_init_packet->softdevice_len)
    {
        if (p_init_packet->softdevice[i]   == DFU_SOFTDEVICE_ANY ||
            p_init_packet->softdevice[i++] == SD_FWID_GET(MBR_SIZE))
        {
            return NRF_SUCCESS;
        }
    }
    
    // No matching SoftDevice found - Return NRF_ERROR_INVALID_DATA.
    return NRF_ERROR_INVALID_DATA;
}


uint32_t dfu_init_postvalidate(uint8_t * p_image, uint32_t image_len)
{
    uint16_t image_crc;
    uint16_t received_crc;
    
    // In order to support hashing (and signing) then the (decrypted) hash should be fetched and
    // the corresponding hash should be calculated over the image at this location.
    // If hashing (or signing) is added to the system then the CRC validation should be removed.

    // calculate CRC from active block.
    image_crc = crc16_compute(p_image, image_len, NULL);

    // Decode the received CRC from extended data.    
    received_crc = uint16_decode((uint8_t *)&m_extended_packet[0]);

    // Compare the received and calculated CRC.
    if (image_crc != received_crc)
    {
        return NRF_ERROR_INVALID_DATA;
    }

    return NRF_SUCCESS;
}

