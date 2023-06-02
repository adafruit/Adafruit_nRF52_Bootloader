/*
 */

#ifndef UF2_BLOCK_FLIP_H_
#define UF2_BLOCK_FLIP_H_

#include <stdint.h>
#include <stdbool.h>

#define UF2_BLOCK_SIZE 512

/* Read MUTEX[0]
*  If a 0 is read, the mutex will lock
*  If a 1 is read, polling will continue
*/
#define INTERCORE_MUTEX_LOCK  while (NRF_MUTEX->MUTEX[0]){ }

/* Release the mutex */
#define INTERCORE_MUTEX_UNLOCK  NRF_MUTEX->MUTEX[0] = 0

typedef enum
{
   NET_INIT, 
   NET_IDLE_REQ,
   NET_IDLE,
   NET_APPL_START_REQ,
   NET_APPL_START_DONE,
   NET_APPL_FW_FLASH_REQ,
   NET_APPL_FW_FLASH_DONE,
   NET_APPL_FW_FLASH_FLUSH_REQ,
   NET_APPL_FW_FLASH_FLUSH_DONE
} net_bootloader_state_t;

typedef struct
{
    net_bootloader_state_t net_bootloader_state;
    uint8_t uf2_block[UF2_BLOCK_SIZE];
} uf2_message_item_t;

#ifdef __cplusplus
 extern "C" {
#endif

void set_net_bootloader_state(net_bootloader_state_t net_bootloader_state);
net_bootloader_state_t get_net_bootloader_state(void);
void store_data_in_flip_section(uint8_t *data);
void get_data_from_flip_section(uint8_t *data);
extern uf2_message_item_t uf2_message_item __attribute__((aligned(4)));

#ifdef __cplusplus
 }
#endif

#endif /* UF2_BLOCK_FLIP_H_ */
