# Agent Instructions

This file provides guidance to AI agents when working with code in this repository.

## Project Overview

CDC/DFU/UF2 bootloader for Nordic nRF52 microcontrollers (nRF52832, nRF52833, nRF52840). Supports DFU over Serial, BLE OTA, and USB mass storage (UF2). Runs on 60+ boards.

## Style

Follow the repo `.clang-format` when making changes.

## Build / Verify

Default board: `feather_nrf52840_express`. Always verify changes against both boards:
- `feather_nrf52840_express` (nRF52840)
- `feather_nrf52832` (nRF52832)

### CMake (preferred)
```bash
cmake -S . -B cmake-build-feather_nrf52840_express -DBOARD=feather_nrf52840_express
cmake --build cmake-build-feather_nrf52840_express

cmake -S . -B cmake-build-feather_nrf52832 -DBOARD=feather_nrf52832
cmake --build cmake-build-feather_nrf52832
```

### Make (alternate)
```bash
make BOARD=feather_nrf52840_express all
make BOARD=feather_nrf52832 all
```

### Flashing
```bash
make BOARD={board} flash       # Flash via JLink
make BOARD={board} flash-dfu   # Flash via Serial/CDC DFU
make BOARD={board} flash-sd    # Flash SoftDevice only
```

### Build all boards
```bash
python3 tools/build_all.py
```

## Architecture

### MCU Variants and SoftDevices
- **nrf52** (nRF52832): UART-only bootloader, default SoftDevice s132 v6.1.1
- **nrf52833**: USB support, default SoftDevice s140 v7.3.0
- **nrf52840**: Full USB + OTA, default SoftDevice s140 v6.1.1
- Boards may override `SD_NAME`/`SD_VERSION` in their `board.mk`

### Key Source Structure
- `src/main.c` — Bootloader entry point, DFU mode detection, LED/button init
- `src/dfu_init.c` — DFU packet validation, CRC/signature verification
- `src/dfu_ble_svc.c` — BLE DFU service
- `src/flash_nrf5x.c` — Flash memory operations
- `src/boards/boards.c` — Board abstraction (LED control, buttons, timing)
- `src/usb/` — USB stack (nRF52833/nRF52840): CDC serial, MSC storage, UF2 handler
- `src/usb/uf2/ghostfat.c` — Virtual FAT filesystem for UF2 drag-and-drop

### Board Definition System

Each board lives in `src/boards/{board_name}/` with:
- `board.h` — Pin definitions, LED/button assignments, USB VID/PID, UF2 metadata
- `board.mk` — Makefile variable `MCU_SUB_VARIANT` (nrf52, nrf52833, or nrf52840)
- `board.cmake` — CMake variable `MCU_VARIANT`
- `pinconfig.c` — CF2 bootloader configuration (flash/RAM size, UF2 family ID)

### Memory Layout (linker scripts in `linker/`)
- Bootloader occupies ~38KB near end of flash (e.g., 0xF4000–0xFD800 on nRF52840)
- No heap (`__HEAP_SIZE=0`), static allocation only
- Special sections: double-reset detection word, bond info for OTA, MBR params, bootloader settings

### Submodules (`lib/`)
- `tinyusb` — USB device stack
- `nrfx` — Nordic HAL drivers
- `uf2` — UF2 format tools
- `tinycrypt` — Crypto (only when `SIGNED_FW=1`)
- `sdk/`, `sdk11/` — Nordic SDK libraries
- `softdevice/` — Precompiled Bluetooth stack binaries

### Compile-Time Feature Flags
- `SIGNED_FW` — Require signed firmware (disables UF2 unless `FORCE_UF2=1`)
- `DUALBANK_FW` — Dual-bank updates
- `DEFAULT_TO_OTA_DFU` — Default to BLE OTA instead of serial DFU
- `DEBUG` — Enable RTT debugging, larger bootloader region

## CI

GitHub Actions (`.github/workflows/githubci.yml`) builds all boards in parallel using a matrix generated from `src/boards/` directory names. On release, artifacts (zip, hex, uf2) are uploaded as release assets.

## Required Toolchain

- `arm-none-eabi-gcc` (tested with 12.3.1)
- Python 3 with: `adafruit-nrfutil`, `intelhex`
- `nrfjprog` (for JLink flashing)
