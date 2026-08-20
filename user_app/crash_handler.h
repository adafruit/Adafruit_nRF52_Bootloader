/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2026 Alastair D'Silva
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

/**
 * @file crash_handler.h
 * @brief Application-side Crash Dump Capture and UF2 Bootloader Recovery Integration
 *
 * ============================================================================
 * ARCHITECTURAL OVERVIEW
 * ============================================================================
 * When an embedded application crashes (CPU HardFault, BusFault, MemManage fault,
 * Kernel Panic, Assert failure, or Hardware Watchdog timeout), standard bootloaders
 * simply reboot straight back into the faulty application because the flash image
 * header is still valid. This creates an unrecoverable, rapid reboot loop that
 * prevents reading serial logs and disrupts physical button timing windows.
 *
 * This header enables user applications to cooperatively capture the fault state
 * and command the Adafruit nRF52 UF2 Bootloader to halt autoboot:
 *
 * 1. Crash Context Capture:
 *    Fatal CPU registers (PC, LR, SP, xPSR, CFSR, HFSR, MMFAR, BFAR, R0-R3, R12)
 *    and the top 64 words of stack memory are saved into an uninitialized RAM
 *    structure (.noinit) that survives soft resets.
 *
 * 2. Bootloader Signal:
 *    The DFU reset command (0x57) is written to NRF_POWER->GPREGRET.
 *
 * 3. Warm Reset:
 *    A Cortex-M System Reset is triggered via SCB->AIRCR.
 *
 * 4. Bootloader Diagnostic File:
 *    The UF2 Bootloader detects the 'CRSH' magic signature in RAM, halts autoboot,
 *    mounts the USB Mass Storage volume, and serves a virtual 'CRASH.TXT' file
 *    containing the register dump and call stack addresses.
 *
 * ============================================================================
 * HOW TO INCORPORATE IN YOUR APPLICATION
 * ============================================================================
 *
 * ----------------------------------------------------------------------------
 * 1. ZEPHYR RTOS INTEGRATION
 * ----------------------------------------------------------------------------
 * In your Zephyr application (e.g. src/main.c or src/crash_handler.c):
 *
 *   #include "crash_handler.h"
 *
 *   // Option A (Simplest - zero boilerplate, uses CRASH_DUMP_RAM_ADDR directly):
 *   CRASH_HANDLER_REGISTER_DEFAULT_ZEPHYR()
 *
 *   // Option B (Explicit .noinit RAM variable):
 *   CRASH_HANDLER_RETAINED struct crash_dump_data g_crash_dump;
 *   CRASH_HANDLER_REGISTER_ZEPHYR(&g_crash_dump)
 *
 *   // Optional Watchdog timeout callback hook:
 *   static void watchdog_timeout_callback(const struct device *dev, int channel_id) {
 *       register uint32_t sp_val __asm__("sp");
 *       uint32_t lr_val = (uint32_t)__builtin_return_address(0);
 *       crash_handler_record_watchdog_timeout(CRASH_DUMP_GLOBAL, sp_val,
 *                                             (uint32_t)watchdog_timeout_callback,
 *                                             lr_val, k_uptime_get());
 *   }
 *
 * ----------------------------------------------------------------------------
 * 2. BARE-METAL / CMSIS / ARDUINO INTEGRATION
 * ----------------------------------------------------------------------------
 * In your application source:
 *
 *   #include "crash_handler.h"
 *
 *   // Option A (Simplest - uses CRASH_DUMP_RAM_ADDR directly):
 *   CRASH_HANDLER_REGISTER_DEFAULT_CMSIS_FAULT()
 *
 *   // Option B (Explicit .noinit RAM variable):
 *   CRASH_HANDLER_RETAINED struct crash_dump_data g_crash_dump;
 *   CRASH_HANDLER_REGISTER_CMSIS_FAULT(&g_crash_dump)
 *
 * ----------------------------------------------------------------------------
 * 3. CHECKING OR CLEARING CRASH DUMPS AT APPLICATION STARTUP
 * ----------------------------------------------------------------------------
 * On boot, your application can check if the previous run crashed:
 *
 *   int main(void) {
 *       if (crash_handler_is_valid(CRASH_DUMP_GLOBAL)) {
 *           printf("!!! PREVIOUS BOOT CRASH DETECTED !!!\n");
 *           printf("Reason: 0x%08lX, PC: 0x%08lX, LR: 0x%08lX\n",
 *                  (unsigned long)CRASH_DUMP_GLOBAL->reason,
 *                  (unsigned long)CRASH_DUMP_GLOBAL->pc,
 *                  (unsigned long)CRASH_DUMP_GLOBAL->lr);
 *
 *           // Clear the signature once processed/logged
 *           crash_handler_clear(CRASH_DUMP_GLOBAL);
 *       }
 *       // ... normal startup ...
 *   }
 *
 * ----------------------------------------------------------------------------
 * 4. DECODING THE CRASH STACK TRACE
 * ----------------------------------------------------------------------------
 * When the bootloader halts in DFU mode, open 'CRASH.TXT' on the USB drive.
 * Pass the PC, LR, and stack return addresses to addr2line to locate source lines:
 *
 *   arm-none-eabi-addr2line -e your_firmware.elf 0x00014a2b 0x00011b90 ...
 * ============================================================================
 */

#ifndef CRASH_HANDLER_H_
#define CRASH_HANDLER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
#define CRASH_HANDLER_EXTERN_C extern "C"
extern "C" {
#else
#define CRASH_HANDLER_EXTERN_C
#endif

/**
 * @brief Bootloader RAM Base Address
 *
 * The Adafruit nRF52 UF2 Bootloader starts its active execution RAM at 0x20008000.
 */
#ifndef BOOTLOADER_RAM_START
#define BOOTLOADER_RAM_START    0x20008000UL
#endif

/**
 * @brief Retained Crash Buffer Offset Below Bootloader RAM Base
 *
 * Placed 2048 bytes (0x800 / 2 KB) below BOOTLOADER_RAM_START to ensure generous
 * retention capacity while avoiding collisions with:
 *   - BOOTLOADER_RAM_START - 0x80 (0x20007F80): BLE DFU OTA Bond Data (128 bytes)
 *   - BOOTLOADER_RAM_START - 0x84 (0x20007F7C): Double-Reset Magic Word (4 bytes)
 */
#ifndef CRASH_DUMP_RAM_OFFSET
#define CRASH_DUMP_RAM_OFFSET   0x800UL
#endif

/* Standardized retained RAM address (0x20007800) */
#ifndef CRASH_DUMP_RAM_ADDR
#define CRASH_DUMP_RAM_ADDR     (BOOTLOADER_RAM_START - CRASH_DUMP_RAM_OFFSET)
#endif

/**
 * @brief Crash Dump Structure Version Identifier
 */
#define CRASH_DUMP_VERSION_1    1UL

/**
 * @brief Crash Dump Magic Identifier ('CRSH')
 *
 * When stored in the retained crash dump structure, this signature indicates
 * to the UF2 bootloader that a crash occurred.
 */
/* Standardized 32-bit Magic Signature: 'CRSH' in ASCII (0x43525348) */
#define CRASH_DUMP_MAGIC        0x43525348UL

/**
 * @brief Crash Reason Codes
 */
#define CRASH_REASON_HARDFAULT    0x00000001UL
#define CRASH_REASON_MEMMANAGE    0x00000002UL
#define CRASH_REASON_BUSFAULT     0x00000003UL
#define CRASH_REASON_USAGEFAULT   0x00000004UL
#define CRASH_REASON_KERNEL_PANIC 0x00000005UL
#define CRASH_REASON_ASSERT       0x00000006UL
#define CRASH_REASON_WATCHDOG     0x5744544FUL /* 'WDTO' */

/**
 * @brief Attribute macro for placing data into uninitialized RAM (.noinit)
 *
 * Variables marked with this attribute persist across soft resets and
 * watchdog reboots without being overwritten by C runtime zero-initialization.
 */
#define CRASH_HANDLER_RETAINED __attribute__((section(".noinit.crash_dump")))

/**
 * @brief Retained Crash Dump Context Structure (Version 1)
 *
 * Standard memory layout for exchanging crash context with the UF2 bootloader.
 */
struct crash_dump_data_v1 {
    uint32_t version;         /**< Structure version (CRASH_DUMP_VERSION_1 = 1) */
    uint32_t magic;           /**< CRASH_DUMP_MAGIC ('CRSH') */
    uint32_t reason;          /**< Exception / Crash reason code */
    uint32_t pc;              /**< Program Counter (PC / R15) */
    uint32_t lr;              /**< Link Register (LR / R14) */
    uint32_t r0;              /**< General Purpose Register R0 */
    uint32_t r1;              /**< General Purpose Register R1 */
    uint32_t r2;              /**< General Purpose Register R2 */
    uint32_t r3;              /**< General Purpose Register R3 */
    uint32_t r12;             /**< General Purpose Register R12 (IP) */
    uint32_t xpsr;            /**< Program Status Register (xPSR) */
    uint32_t sp;              /**< Stack Pointer at fault (MSP/PSP) */
    uint32_t cfsr;            /**< Configurable Fault Status Register (SCB->CFSR) */
    uint32_t hfsr;            /**< HardFault Status Register (SCB->HFSR) */
    uint32_t mmfar;           /**< MemManage Fault Address Register (SCB->MMFAR) */
    uint32_t bfar;            /**< BusFault Address Register (SCB->BFAR) */
    uint64_t uptime_ms;       /**< Milliseconds since boot */
    uint32_t stack_words[64]; /**< Top 64 words of stack memory at fault */
};

typedef struct crash_dump_data_v1 crash_dump_data_v1_t;
typedef struct crash_dump_data_v1 crash_dump_data_t;
#define crash_dump_data crash_dump_data_v1

/**
 * @brief Global pointer convenience macro referencing the standardized retained RAM location.
 */
#define CRASH_DUMP_GLOBAL ((struct crash_dump_data_v1 *)CRASH_DUMP_RAM_ADDR)

/**
 * @brief Check if a valid crash dump is currently retained in RAM.
 *
 * @param dump Pointer to crash_dump_data_v1 instance.
 * @return true if valid crash dump present, false otherwise.
 */
static inline bool crash_handler_is_valid(const struct crash_dump_data_v1 *dump) {
    return dump && (dump->magic == CRASH_DUMP_MAGIC) && (dump->version == CRASH_DUMP_VERSION_1);
}

/**
 * @brief Clear/invalidate the retained crash dump.
 *
 * @param dump Pointer to crash_dump_data_v1 instance.
 */
static inline void crash_handler_clear(struct crash_dump_data_v1 *dump) {
    if (dump) {
        dump->magic = 0;
        dump->version = 0;
    }
}

/**
 * @brief Set GPREGRET to stay in bootloader and trigger system reset.
 */
static inline void crash_handler_trigger_dfu(void) {
    /* Set NRF_POWER->GPREGRET = 0x57 (DFU_MAGIC_UF2_RESET) */
    *(volatile uint32_t *)0x4000051CUL = 0x57;

    /* Issue Cortex-M System Reset via SCB->AIRCR */
    *(volatile uint32_t *)0xE000ED0CUL = 0x05FA0004UL;

    while (1) {
        __asm__ volatile("nop");
    }
}

/**
 * @brief Capture hardware fault status registers into crash context.
 *
 * @param dump Pointer to crash_dump_data_v1 instance.
 */
static inline void crash_handler_read_scb(struct crash_dump_data_v1 *dump) {
    if (!dump) return;
    /* Cortex-M System Control Block Fault Registers */
    dump->cfsr  = *(volatile uint32_t *)0xE000ED28UL; /* SCB->CFSR */
    dump->hfsr  = *(volatile uint32_t *)0xE000ED2CUL; /* SCB->HFSR */
    dump->mmfar = *(volatile uint32_t *)0xE000ED34UL; /* SCB->MMFAR */
    dump->bfar  = *(volatile uint32_t *)0xE000ED38UL; /* SCB->BFAR */
}

/**
 * @brief Record a crash from standard Cortex-M hardware stacked exception frame.
 *
 * Standard exception stack frame layout pushed by Cortex-M hardware:
 *   [sp+0]  = R0
 *   [sp+4]  = R1
 *   [sp+8]  = R2
 *   [sp+12] = R3
 *   [sp+16] = R12
 *   [sp+20] = LR (return address before exception)
 *   [sp+24] = PC (address of instruction causing exception)
 *   [sp+28] = xPSR
 *
 * @param dump Pointer to crash_dump_data_v1 instance.
 * @param reason Reason code (e.g. CRASH_REASON_HARDFAULT).
 * @param stack_frame Pointer to top of hardware exception stack frame.
 * @param uptime_ms Milliseconds since system startup (0 if unavailable).
 */
static inline void crash_handler_record_exception_frame(struct crash_dump_data_v1 *dump,
                                                        uint32_t reason,
                                                        const uint32_t *stack_frame,
                                                        uint64_t uptime_ms) {
    if (!dump) return;

    dump->version = CRASH_DUMP_VERSION_1;
    dump->magic = CRASH_DUMP_MAGIC;
    dump->reason = reason;
    dump->uptime_ms = uptime_ms;
    crash_handler_read_scb(dump);

    if (stack_frame != NULL) {
        dump->r0   = stack_frame[0];
        dump->r1   = stack_frame[1];
        dump->r2   = stack_frame[2];
        dump->r3   = stack_frame[3];
        dump->r12  = stack_frame[4];
        dump->lr   = stack_frame[5];
        dump->pc   = stack_frame[6];
        dump->xpsr = stack_frame[7];
        dump->sp   = (uint32_t)stack_frame;

        /* Safely copy up to 64 stack words within SRAM address space */
        uint32_t sp_addr = (uint32_t)stack_frame;
        for (int i = 0; i < 64; i++) {
            if (sp_addr >= 0x20000000UL && sp_addr <= 0x2003FFFCUL) {
                dump->stack_words[i] = *(const uint32_t *)sp_addr;
                sp_addr += 4;
            } else {
                dump->stack_words[i] = 0;
            }
        }
    }

    crash_handler_trigger_dfu();
}

/**
 * @brief Record watchdog timeout crash dump and reset to DFU mode.
 *
 * @param dump Pointer to crash_dump_data_v1 instance.
 * @param sp_val Stack pointer at timeout.
 * @param pc_val PC address of caller / timeout handler.
 * @param lr_val Return address / Link Register.
 * @param uptime_ms Milliseconds since system startup (0 if unavailable).
 */
static inline void crash_handler_record_watchdog_timeout(struct crash_dump_data_v1 *dump,
                                                         uint32_t sp_val,
                                                         uint32_t pc_val,
                                                         uint32_t lr_val,
                                                         uint64_t uptime_ms) {
    if (!dump) return;

    dump->version = CRASH_DUMP_VERSION_1;
    dump->magic = CRASH_DUMP_MAGIC;
    dump->reason = CRASH_REASON_WATCHDOG;
    dump->uptime_ms = uptime_ms;
    dump->sp = sp_val;
    dump->pc = pc_val;
    dump->lr = lr_val;
    crash_handler_read_scb(dump);

    const uint32_t *sp_ptr = (const uint32_t *)sp_val;
    if (sp_ptr) {
        uint32_t sp_addr = sp_val;
        for (int i = 0; i < 64; i++) {
            if (sp_addr >= 0x20000000UL && sp_addr <= 0x2003FFFCUL) {
                dump->stack_words[i] = *(const uint32_t *)sp_addr;
                sp_addr += 4;
            } else {
                dump->stack_words[i] = 0;
            }
        }
    }

    crash_handler_trigger_dfu();
}

/* ========================================================================= */
/* Framework Registration Helpers                                            */
/* ========================================================================= */

/**
 * @brief Macro to define standard CMSIS Cortex-M HardFault_Handler.
 *
 * Extracts MSP/PSP stack pointer and routes into crash handler context.
 *
 * Usage:
 *   CRASH_HANDLER_RETAINED struct crash_dump_data_v1 g_crash_dump;
 *   CRASH_HANDLER_REGISTER_CMSIS_FAULT(&g_crash_dump)
 */
#define CRASH_HANDLER_REGISTER_CMSIS_FAULT(dump_ptr)                                            \
    CRASH_HANDLER_EXTERN_C void __attribute__((used)) _crash_handler_c_entry(const uint32_t *stack_frame) { \
        crash_handler_record_exception_frame((dump_ptr), CRASH_REASON_HARDFAULT, stack_frame, 0); \
    }                                                                                           \
    CRASH_HANDLER_EXTERN_C __attribute__((naked)) void HardFault_Handler(void) {                \
        __asm__ volatile(                                                                       \
            "tst lr, #4\n"                                                                      \
            "ite eq\n"                                                                          \
            "mrseq r0, msp\n"                                                                   \
            "mrsne r0, psp\n"                                                                   \
            "b _crash_handler_c_entry\n"                                                        \
        );                                                                                      \
    }

/**
 * @brief Zero-argument macro to register standard CMSIS HardFault handler using CRASH_DUMP_RAM_ADDR directly.
 *
 * Usage:
 *   CRASH_HANDLER_REGISTER_DEFAULT_CMSIS_FAULT()
 */
#define CRASH_HANDLER_REGISTER_DEFAULT_CMSIS_FAULT() \
    CRASH_HANDLER_REGISTER_CMSIS_FAULT(CRASH_DUMP_GLOBAL)

/**
 * @brief Macro to define Zephyr OS fatal error handler (k_sys_fatal_error_handler).
 *
 * Usage:
 *   CRASH_HANDLER_RETAINED struct crash_dump_data_v1 g_crash_dump;
 *   CRASH_HANDLER_REGISTER_ZEPHYR(&g_crash_dump)
 */
#define CRASH_HANDLER_REGISTER_ZEPHYR(dump_ptr)                                                 \
    CRASH_HANDLER_EXTERN_C void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf) { \
        uint64_t now_ms = 0;                                                                    \
        if (dump_ptr) {                                                                         \
            (dump_ptr)->version = CRASH_DUMP_VERSION_1;                                         \
            (dump_ptr)->magic = CRASH_DUMP_MAGIC;                                               \
            (dump_ptr)->reason = reason;                                                        \
            (dump_ptr)->uptime_ms = now_ms;                                                     \
            crash_handler_read_scb(dump_ptr);                                                   \
            if (esf != NULL) {                                                                  \
                const uint32_t *frame = (const uint32_t *)esf;                                 \
                (dump_ptr)->r0   = frame[0];                                                    \
                (dump_ptr)->r1   = frame[1];                                                    \
                (dump_ptr)->r2   = frame[2];                                                    \
                (dump_ptr)->r3   = frame[3];                                                    \
                (dump_ptr)->r12  = frame[4];                                                    \
                (dump_ptr)->lr   = frame[5];                                                    \
                (dump_ptr)->pc   = frame[6];                                                    \
                (dump_ptr)->xpsr = frame[7];                                                    \
                (dump_ptr)->sp   = (uint32_t)esf;                                               \
                uint32_t esf_addr = (uint32_t)esf;                                              \
                for (int i = 0; i < 64; i++) {                                                  \
                    if (esf_addr >= 0x20000000UL && esf_addr <= 0x2003FFFCUL) {                 \
                        (dump_ptr)->stack_words[i] = *(const uint32_t *)esf_addr;               \
                        esf_addr += 4;                                                          \
                    } else {                                                                    \
                        (dump_ptr)->stack_words[i] = 0;                                         \
                    }                                                                           \
                }                                                                               \
            }                                                                                   \
        }                                                                                       \
        crash_handler_trigger_dfu();                                                            \
    }

/**
 * @brief Zero-argument macro to register Zephyr fatal error handler using CRASH_DUMP_RAM_ADDR directly.
 *
 * Usage:
 *   CRASH_HANDLER_REGISTER_DEFAULT_ZEPHYR()
 */
#define CRASH_HANDLER_REGISTER_DEFAULT_ZEPHYR() \
    CRASH_HANDLER_REGISTER_ZEPHYR(CRASH_DUMP_GLOBAL)

#ifdef __cplusplus
}
#endif

#endif /* CRASH_HANDLER_H_ */
