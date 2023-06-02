/*
 */

#include <string.h>
#include <dfu_types.h>
#include <nrfx_config.h>
#include "uf2_block_flip.h"
#include "uf2/uf2.h"

#include "boards.h"

extern int  write_block(uint32_t block_no, uint8_t *data, WriteState *state);

void set_net_bootloader_state(net_bootloader_state_t net_bootloader_state)
{
    uf2_message_item.net_bootloader_state = net_bootloader_state;
}

net_bootloader_state_t get_net_bootloader_state(void)
{
    return uf2_message_item.net_bootloader_state;
}

void store_data_in_flip_section(uint8_t * data)
{
    bool loop = true;

    while (loop)
    {
        INTERCORE_MUTEX_LOCK;

        if( (uf2_message_item.net_bootloader_state == NET_APPL_FW_FLASH_DONE) || (uf2_message_item.net_bootloader_state == NET_IDLE) )
        {
            /* New block is expected by net core */
            memcpy(uf2_message_item.uf2_block, data, UF2_BLOCK_SIZE);
            uf2_message_item.net_bootloader_state = NET_APPL_FW_FLASH_REQ;
            loop = false;
        }
        NRF_MUTEX->MUTEX[0] = 0;  /* Release the mutex */
    }
}

void get_data_from_flip_section(uint8_t * data)
{
    static WriteState _wr_state = { 0 };
   
    if(uf2_message_item.net_bootloader_state == NET_APPL_FW_FLASH_REQ)
    {
        UF2_Block *bl = (void*)uf2_message_item.uf2_block;

        if(0 != write_block(0, uf2_message_item.uf2_block, &_wr_state))
        {
            uf2_message_item.net_bootloader_state = NET_APPL_FW_FLASH_DONE;
        }
    }
}