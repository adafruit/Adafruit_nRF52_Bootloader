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

#include "tusb.h"
#include "uf2/uf2.h"

#if CFG_TUD_MSC

#include "bootloader.h"

/*------------------------------------------------------------------*/
/* MACRO TYPEDEF CONSTANT ENUM
 *------------------------------------------------------------------*/

/*------------------------------------------------------------------*/
/* UF2
 *------------------------------------------------------------------*/
static WriteState _wr_state = { 0 };

void read_block(uint32_t block_no, uint8_t *data);
int write_block(uint32_t block_no, uint8_t *data, bool quiet, WriteState *state);

//--------------------------------------------------------------------+
// tinyusb callbacks
//--------------------------------------------------------------------+

// Callback invoked when received an SCSI command not in built-in list below
// - READ_CAPACITY10, READ_FORMAT_CAPACITY, INQUIRY, MODE_SENSE6, REQUEST_SENSE
// - READ10 and WRITE10 has their own callbacks
int32_t tud_msc_scsi_cb (uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize)
{
  void const* response = NULL;
  int32_t resplen = 0;
  memset(buffer, 0, bufsize);

  switch ( scsi_cmd[0] )
  {
    case SCSI_CMD_TEST_UNIT_READY:
      // Command that host uses to check our readiness before sending other commands
      resplen = 0;
    break;

    case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
      // Host is about to read/write etc ... better not to disconnect disk
      resplen = 0;
    break;

    case SCSI_CMD_START_STOP_UNIT:
      // Host try to eject/safe remove/poweroff us. We could safely disconnect with disk storage, or go into lower power
      /* scsi_start_stop_unit_t const * start_stop = (scsi_start_stop_unit_t const *) scsi_cmd;
       // Start bit = 0 : low power mode, if load_eject = 1 : unmount disk storage as well
       // Start bit = 1 : Ready mode, if load_eject = 1 : mount disk storage
       start_stop->start;
       start_stop->load_eject;
       */
      resplen = 0;
    break;

    default:
      // Set Sense = Invalid Command Operation
      tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);

      // negative means error -> tinyusb could stall and/or response with failed status
      resplen = -1;
    break;
  }

  // return len must not larger than bufsize
  if ( resplen > (int32_t)bufsize ) resplen = bufsize;

  // copy response to stack's buffer if any
  if ( response && resplen )
  {
    memcpy(buffer, response, resplen);
  }

  return resplen;
}

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and return number of copied bytes.
int32_t tud_msc_read10_cb (uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize)
{
  (void) lun;
  memset(buffer, 0, bufsize);

  // since we return block size each, offset should always be zero
  TU_ASSERT(offset == 0, -1);

  uint32_t count = 0;

  while ( count < bufsize )
  {
    read_block(lba, buffer);

    lba++;
    buffer += 512;
    count  += 512;
  }

  return count;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and return number of written bytes
int32_t tud_msc_write10_cb (uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize)
{
  (void) lun;

  uint32_t count = 0;
  int wr_ret;

  while ( (count < bufsize) && ((wr_ret = write_block(lba, buffer, false, &_wr_state)) > 0) )
  {
    lba++;
    buffer += 512;
    count  += 512;
  }

  // Consider non-uf2 block write as successful
  return  (wr_ret < 0) ? bufsize : count;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
void tud_msc_write10_complete_cb(uint8_t lun)
{
  // uf2 file writing is complete --> complete DFU process
  if ( _wr_state.numBlocks && (_wr_state.numWritten >= _wr_state.numBlocks) )
  {
    led_state(STATE_WRITING_FINISHED);

    dfu_update_status_t update_status;

    memset(&update_status, 0, sizeof(dfu_update_status_t ));
    update_status.status_code = DFU_UPDATE_APP_COMPLETE;
    update_status.app_crc     = 0; // skip CRC checking with uf2 upgrade
    update_status.app_size    = _wr_state.numBlocks*256;

    bootloader_dfu_update_process(update_status);
  }
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size)
{
  (void) lun;

  *block_count = UF2_NUM_BLOCKS;
  *block_size  = 512;
}

#endif
