/**************************************************************************/
/*!
 @file     flash_nrf5x.c
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

#include <string.h>
#include "flash_nrf5x.h"
#include "boards.h"

#include "nrf_sdm.h"

#define FLASH_PAGE_SIZE    4096

#define FLASH_CACHE_INVALID_ADDR 0xffffffff

static uint32_t _fl_addr = FLASH_CACHE_INVALID_ADDR;
static uint8_t _fl_buf[FLASH_PAGE_SIZE] __attribute__((aligned(4)));

void flash_flush(void)
{
  if ( _fl_addr == FLASH_CACHE_INVALID_ADDR ) return;

  if ( memcmp(_fl_buf, (void *) _fl_addr, FLASH_PAGE_SIZE) != 0 )
  {
    // nrf_nvmc_page_erase(_fl_addr);
    //nrf_nvmc_write_words(_fl_addr, (uint32_t *) _fl_buf, FLASH_PAGE_SIZE/4);

    sd_flash_page_erase(_fl_addr/FLASH_PAGE_SIZE);
    sd_flash_write((uint32_t*) _fl_addr, (uint32_t *) _fl_buf, FLASH_PAGE_SIZE/4);

  }

  _fl_addr = FLASH_CACHE_INVALID_ADDR;
}

void flash_write (uint32_t dst, void const *src, int len)
{
  uint32_t newAddr = dst & ~(FLASH_PAGE_SIZE - 1);

  if ( newAddr != _fl_addr )
  {
    flash_flush();
    _fl_addr = newAddr;
    memcpy(_fl_buf, (void *) newAddr, FLASH_PAGE_SIZE);
  }
  memcpy(_fl_buf + (dst & (FLASH_PAGE_SIZE - 1)), src, len);
}

void flash_erase(uint32_t addr, uint32_t bytes)
{
  uint32_t page_count = bytes/FLASH_PAGE_SIZE;
  if ( bytes%FLASH_PAGE_SIZE ) page_count++;

  for(uint32_t i=0; i<page_count; i++)
  {
    nrf_nvmc_page_erase(addr + i*FLASH_PAGE_SIZE);
  }
}
