# Application Crash Handler & Post-Mortem Diagnostics

This directory provides a lightweight, zero-overhead crash retention framework for user applications running on top of the **Adafruit nRF52 UF2 Bootloader**.

---

## The Problem: Embedded Reboot Loops

When an unhandled CPU exception (such as a `HardFault`, `BusFault`, `MemManage`, `UsageFault`, `Kernel Panic`, or `Watchdog Timeout`) occurs on a microcontroller, traditional bootloaders immediately attempt to restart the application because the application image in flash is valid.

This leads to:
1. **Rapid reboot loops** where serial logs are lost before USB CDC can enumerate.
2. **Difficulty entering DFU mode** without precise physical button double-tapping.
3. **Zero diagnostic visibility** when devices are deployed in enclosures without SWD/JTAG debuggers.

---

## The Solution: Cooperative Crash Interception

With this crash handler framework:
1. When a fatal exception occurs, the application captures the CPU registers (`PC`, `LR`, `SP`, `xPSR`, `CFSR`, `HFSR`, `MMFAR`, `BFAR`, `R0`–`R3`, `R12`), uptime, and the top 64 words of the stack into a designated retained RAM region (`0x20007800`, 2 KB below bootloader RAM).
2. The application sets `NRF_POWER->GPREGRET = 0x57` (`DFU_MAGIC_UF2_RESET`) and performs a system reset.
3. On reboot, the UF2 bootloader inspects the retained RAM. If `CRASH_DUMP_MAGIC` (`0x43525348`) and `CRASH_DUMP_VERSION_1` (`1`) are present:
   - **Autoboot is halted**, keeping the device safely in DFU mode.
   - The bootloader dynamically generates a virtual **`CRASH.TXT`** file on the USB Mass Storage volume alongside `CURRENT.UF2` and `INFO_UF2.TXT`.
4. Developers can inspect `CRASH.TXT` directly or run `decode_crash.py` to resolve raw hex addresses into exact source filenames and line numbers.

---

## Memory Layout & Safe Retention

- **Bootloader RAM Usage**: The bootloader executes in RAM starting at `0x20008000`.
- **Retained Crash Buffer (`CRASH_DUMP_RAM_ADDR`)**: Placed at `0x20007800` (`BOOTLOADER_RAM_START - 0x800`), reserving a generous 2 KB window below the bootloader's active workspace and avoiding the BLE bond sharing (`0x20007F80`) and double-reset (`0x20007F7C`) memory words.
- Variables placed in `.noinit` or at `0x20007800` persist across warm resets and watchdog resets without being overwritten by the C runtime startup.

---

## Virtual `CRASH.TXT` Format

When the USB drive mounts (e.g. `NICENANO` or `FEATHERBOOT`), opening `CRASH.TXT` displays:

```text
=================================================
 Application Fatal Crash Dump Report (v1)
=================================================
 Version: 1
 Reason:  0x0000001A
 PC:      0x00006476
 LR:      0x00010023
 SP:      0x2000FAF8
 xPSR:    0x61000000
 CFSR:    0x00000000
 HFSR:    0x00000000
 Stack (Top 8 Words):
   0x00000000 0x000003E8 0xDEADBEEF 0xFFFFF000
   0x200025B0 0x00010023 0x00006476 0x61000000
=================================================
```

---

## Automated Symbol Decoding (`decode_crash.py`)

The included [`decode_crash.py`](decode_crash.py) script automatically detects the mounted drive, decodes ARM Cortex-M fault bitfields, and invokes `addr2line` against the application ELF binary:

```bash
python3 decode_crash.py --elf path/to/your/application.elf
```

### Example Output:
```text
Auto-detected mounted crash file: /run/media/deece/NICENANO/CRASH.TXT
======================================================================
 Adafruit nRF52 UF2 Bootloader - Crash Dump Analysis
 Source:    /run/media/deece/NICENANO/CRASH.TXT
 Symbols:   build/supermini_nrf52840/nrf-intercom/zephyr/zephyr.elf
 Tool:      arm-none-eabi-addr2line
======================================================================

[Dump Version] v1
[Crash Reason] 0x0000001A -> Zephyr CPU Exception (K_ERR_CPU_EXCEPTION / 0x1A)
[Fault Status] CFSR: 0x00000000

[Crash Location (PC)] 0x00006476 -> cli_thread_fn at src/main.c:1008
[Caller Return (LR)]  0x00010023 -> sys_clock_elapsed at zephyr/drivers/timer/nrf_rtc_timer.c:695
======================================================================
```

---

## Application Integration Examples

### 1. Zephyr RTOS Integration

#### Option A: Zero-Boilerplate Registration (Recommended)
Simply include the header and invoke the default registration macro in any source file (e.g. `src/main.c`):

```c
#include "crash_handler.h"

/* Registers k_sys_fatal_error_handler using CRASH_DUMP_GLOBAL */
CRASH_HANDLER_REGISTER_DEFAULT_ZEPHYR()
```

#### Option B: Custom Implementation
Implement `k_sys_fatal_error_handler` manually to intercept all fatal CPU exceptions, asserts, and kernel panics:

```c
#include "crash_handler.h"
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>

void k_sys_fatal_error_handler(unsigned int reason, const z_arch_esf_t *esf)
{
    CRASH_DUMP_GLOBAL->magic     = CRASH_DUMP_MAGIC;
    CRASH_DUMP_GLOBAL->reason    = (uint32_t)reason;
    CRASH_DUMP_GLOBAL->uptime_ms = (uint64_t)k_uptime_get();

    crash_handler_read_scb(CRASH_DUMP_GLOBAL);

    if (esf != NULL) {
        CRASH_DUMP_GLOBAL->r0   = esf->basic.r0;
        CRASH_DUMP_GLOBAL->r1   = esf->basic.r1;
        CRASH_DUMP_GLOBAL->r2   = esf->basic.r2;
        CRASH_DUMP_GLOBAL->r3   = esf->basic.r3;
        CRASH_DUMP_GLOBAL->r12  = esf->basic.r12;
        CRASH_DUMP_GLOBAL->lr   = esf->basic.lr;
        CRASH_DUMP_GLOBAL->pc   = esf->basic.pc;
        CRASH_DUMP_GLOBAL->xpsr = esf->basic.xpsr;
        CRASH_DUMP_GLOBAL->sp   = (uint32_t)esf;

        const uint32_t *stack = (const uint32_t *)esf;
        for (int i = 0; i < 64; i++) {
            CRASH_DUMP_GLOBAL->stack_words[i] = stack[i];
        }
    }

    crash_handler_trigger_dfu();
}
```

---

### 2. Bare-Metal / FreeRTOS (Cortex-M Hardware Exception Stacking)

#### Option A: Zero-Boilerplate Registration (Recommended)
```c
#include "crash_handler.h"

/* Registers HardFault_Handler using CRASH_DUMP_GLOBAL */
CRASH_HANDLER_REGISTER_DEFAULT_CMSIS_FAULT()
```

#### Option B: Custom Implementation
Capture the hardware-stacked exception frame directly from Assembly/C:

```c
#include "crash_handler.h"

void HardFault_Handler_C(uint32_t *stack_frame)
{
    crash_handler_record_exception_frame(CRASH_DUMP_GLOBAL, CRASH_REASON_HARDFAULT, stack_frame, 0);
}

__attribute__((naked)) void HardFault_Handler(void)
{
    __asm__ volatile(
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "b HardFault_Handler_C\n"
    );
}
```

---

### 3. Software Bootloader Jump (No Button Required)

Applications can programmatically enter the UF2 bootloader at any time over a serial CLI, BLE characteristic, or HTTP API:

```c
#include "crash_handler.h"

void reboot_to_bootloader(void)
{
    crash_handler_trigger_dfu();
}
```

