# Adafruit nRF52 Bootloader Agent Instructions

UF2, CDC/serial and BLE OTA DFU bootloader for Nordic nRF52832, nRF52833 and nRF52840, running beside Nordic's MBR and
SoftDevice, on the boards under `src/boards/`.

## Working Rules

- Worktrees: `git submodule update --init` (`lib/tinyusb`, `lib/nrfx`, `lib/uf2`, `lib/tinycrypt`); `--reference
  <primary checkout>/<path>` borrows the primary checkout's objects.
- Code references: boot decisions and DFU entry (GPREGRET magic, double reset, app validity, SoftDevice handoff)
  `src/main.c`; UF2 drive `src/usb/uf2/ghostfat.c`, `src/usb/msc_uf2.c`, layout `src/usb/uf2/uf2cfg.h`; USB
  descriptors `src/usb/usb_desc.c`; flash writes `src/flash_nrf5x.c`; DFU packet validation and signed firmware
  `src/dfu_init.c`; serial and BLE DFU transports, settings and banks `lib/sdk11/components/libraries/bootloader_dfu/`;
  boards `src/boards/<board>/` (`board.h`, `board.cmake`, `pinconfig.c`) and `src/boards/boards.c`; memory map
  `linker/`; MBR and SoftDevices `lib/softdevice/`; build `CMakeLists.txt`, `cmake/`, and `Makefile`.

## Code Rules

- Formatting follows `.clang-format`. Line endings are mixed (CRLF in `Makefile`, some `board.cmake`, the workflows);
  keep each file's own.
- CMake is the build; the `Makefile` is a thin wrapper that configures and builds through it. A board override in
  `board.cmake` is guarded with `if (NOT DEFINED <VAR>)`, so `-D` wins.
- The bootloader region is fixed by `linker/<mcu>.ld`; check `Memory region` usage in the build log after a change
  that grows code.
- USB VID/PID: a third-party board uses its own allocated VID/PID. Adafruit's VID `0x239A` is for Adafruit-allocated
  boards only, whose bootloader PID is their row in `Production/usb_vid_pid.csv` of MBAdafruitBoards
  (`~/code/adafruit/MBAdafruitBoards`); a board without a row needs a new allocation: ask the human first. Boards
  marked `// Shared VID/PID with ...` are legacy, not a template.

## Skills

- Agents, skills and workflows beyond this repo's `.claude/` come from [agentrc](https://github.com/hathach/agentrc);
  install it before agent work here. The scripts here load agentrc's shared scripts through
  `.claude/skills/hil-contract/scripts/agentrc.py`: the installed agentrc (`${AGENTRC_DIR:-$HOME/.claude}`), or in
  CI the commit `test/hil/agentrc.json` pins (`agentrc.py fetch`).
- Skill script tests live in `.claude/test/test_*.py`; pre-commit runs them.
- Hardware diagnosis and repair use agentrc's `hw-debugger`; independent hardware claims and committed-fix
  validation use `hw-validator`; routine HIL runs use `hil-operator`, all orchestrated by `chief`.
- Bootloader audit: the `code-audit` workflow with `dirs` (e.g. `src`, `src/usb`, `lib/sdk11/components/libraries/
  bootloader_dfu`) and `dimensions`: `UF2 and MSC input: block validation, family and address checks, bounds and
  arithmetic`; `DFU packets (serial, BLE): init packet, CRC, signature (SIGNED_FW), sizes and bank handling`; `flash,
  MBR and SoftDevice boundaries: app region, bootloader settings, UICR, bootloader self-update (COPY_BL)`; `boot
  decisions and app handoff: GPREGRET magic, double reset, app validity, SoftDevice enable/disable, interrupt
  forwarding`; `ISR and shared state, and register use vs the nRF52 product specifications and errata: cross-check
  them via the read-doc skill; a lookup that cannot run is pending and its conclusion unverified (never a web or
  filesystem substitute); a missing erratum workaround is a finding`; `build and configuration: CMake and the
  Makefile wrapper, board overrides, feature flags`; `style: repo conventions and line endings`.

## Build and Validate

- Build contract: `.claude/skills/build/SKILL.md`. It resolves a change to boards and builds each the way CI does,
  outside the source tree.
- HIL contract: `.claude/skills/hil-contract/SKILL.md`. Hardware validation comes from its report, never from a
  build; what no roster board on the host covers is reported pending.
- Before submitting, and again after a rebase: `pre-commit run --all-files`.
- For code changes: `python3 .claude/skills/build/scripts/check_build.py --base origin/master --worktree`.

## PRs and Follow-ups

- Before opening or updating a PR, follow Build and Validate.
- `pr-babysit` args: `{"pr": <num>, "protected": "^(lib/(tinyusb|nrfx|uf2|tinycrypt|softdevice/.*)|test/hil/[^/]+\\.json)$"}`;
  reviewers are agentrc's default. `protected` keeps the submodule pins, the SoftDevice binaries and the HIL roster
  and pin out of automated fixes.
- The follow-up label is `follow-up`.
