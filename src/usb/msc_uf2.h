/**************************************************************************/
/*!
    @file     msc_uf2.h
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
#ifndef MSC_FLASH_H_
#define MSC_FLASH_H_

#include "tusb.h"

#ifdef __cplusplus
 extern "C" {
#endif

// for checking flash size
#include "dfu_types.h"
#include "uf2/uf2.h"

/*------------------------------------------------------------------*/
/* FLASH Configuration
 *------------------------------------------------------------------*/
#define MSC_UF2_FLASH_ADDR_START  0xAD000
#define MSC_UF2_FLASH_SIZE        (256*1024)

#define MSC_UF2_BLOCK_SIZE        512
#define MSC_UF2_BLOCK_NUM         UF2_NUM_BLOCKS

VERIFY_STATIC( MSC_UF2_FLASH_ADDR_START+MSC_UF2_FLASH_SIZE == BOOTLOADER_REGION_START-DFU_APP_DATA_RESERVED, );

/*------------------------------------------------------------------*/
/* Note ATTR_WEAK is used when CFG_TUD_MSC = 0
 *------------------------------------------------------------------*/

ATTR_WEAK void msc_uf2_init(void);
ATTR_WEAK void msc_uf2_mount(void);
ATTR_WEAK void msc_uf2_umount(void);


#ifdef __cplusplus
 }
#endif

#endif /* MSC_FLASH_H_ */
