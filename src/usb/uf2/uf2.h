/**

Microsoft UF2

The MIT License (MIT)

Copyright (c) Microsoft Corporation

All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
#ifndef UF2FORMAT_H
#define UF2FORMAT_H 1

#include "uf2cfg.h"

#include <stdint.h>
#include <stdbool.h>

#include "app_util.h"

#include "dfu_types.h"

#define SD_MAGIC_NUMBER 0x51b1e5db
#define SD_MAGIC_OK() (*((uint32_t*)(SOFTDEVICE_INFO_STRUCT_ADDRESS+4)) == 0x51b1e5db)
extern bool sdRunning;

// All entries are little endian.

#define UF2_MAGIC_START0 0x0A324655UL // "UF2\n"
#define UF2_MAGIC_START1 0x9E5D5157UL // Randomly selected
#define UF2_MAGIC_END 0x0AB16F30UL    // Ditto

// If set, the block is "comment" and should not be flashed to the device
#define UF2_FLAG_NOFLASH 0x00000001
#define UF2_FLAG_FAMILYID 0x00002000

#define MAX_BLOCKS (FLASH_SIZE / 256 + 100)
typedef struct {
    uint32_t numBlocks;
    uint32_t numWritten;
    uint8_t writtenMask[MAX_BLOCKS / 8 + 1];
} WriteState;

typedef struct {
    // 32 byte header
    uint32_t magicStart0;
    uint32_t magicStart1;
    uint32_t flags;
    uint32_t targetAddr;
    uint32_t payloadSize;
    uint32_t blockNo;
    uint32_t numBlocks;
    uint32_t familyID;

    // raw data;
    uint8_t data[476];

    // store magic also at the end to limit damage from partial block reads
    uint32_t magicEnd;
} UF2_Block;

typedef struct {
    uint8_t version;
    uint8_t ep_in;
    uint8_t ep_out;
    uint8_t reserved0;
    uint32_t cbw_tag;
    uint32_t blocks_remaining;
    uint8_t *buffer;
} UF2_HandoverArgs;

typedef void (*UF2_MSC_Handover_Handler)(UF2_HandoverArgs *handover);
typedef void (*UF2_HID_Handover_Handler)(int ep);

// this is required to be exactly 16 bytes long by the linker script
typedef struct {
    void *reserved0;
    UF2_HID_Handover_Handler handoverHID;
    UF2_MSC_Handover_Handler handoverMSC;
    const char *info_uf2;
} UF2_BInfo;

#define UF2_BINFO ((UF2_BInfo *)(APP_START_ADDRESS - sizeof(UF2_BInfo)))

static inline bool is_uf2_block(void *data) {
    UF2_Block *bl = (UF2_Block *)data;
    return bl->magicStart0 == UF2_MAGIC_START0 && bl->magicStart1 == UF2_MAGIC_START1 &&
           bl->magicEnd == UF2_MAGIC_END;
}

static inline bool in_uf2_bootloader_space(const void *addr) {
    return USER_FLASH_END <= (uint32_t)addr && (uint32_t)addr < FLASH_SIZE;
}


#ifdef UF2_DEFINE_HANDOVER
static inline const char *uf2_info(void) {
    if (in_uf2_bootloader_space(UF2_BINFO->info_uf2))
        return UF2_BINFO->info_uf2;
    return "N/A";
}

static inline void hf2_handover(uint8_t ep) {
    const char *board_info = UF2_BINFO->info_uf2;
    UF2_HID_Handover_Handler fn = UF2_BINFO->handoverHID;

    if (in_uf2_bootloader_space(board_info) && in_uf2_bootloader_space((const void *)fn) &&
        ((uint32_t)fn & 1)) {
        // Pass control to bootloader; never returns
        fn(ep & 0xf);
    }
}

// the ep_in/ep_out are without the 0x80 mask
// cbw_tag is in the same bit format as it came
static inline void check_uf2_handover(uint8_t *buffer, uint32_t blocks_remaining, uint8_t ep_in,
                                      uint8_t ep_out, uint32_t cbw_tag) {
    if (!is_uf2_block(buffer))
        return;

    const char *board_info = UF2_BINFO->info_uf2;
    UF2_MSC_Handover_Handler fn = UF2_BINFO->handoverMSC;

    if (in_uf2_bootloader_space(board_info) && in_uf2_bootloader_space((const void *)fn) &&
        ((uint32_t)fn & 1)) {
        UF2_HandoverArgs hand = {
            1, ep_in, ep_out, 0, cbw_tag, blocks_remaining, buffer,
        };
        // Pass control to bootloader; never returns
        fn(&hand);
    }
}
#endif

#endif
