---
name: hil-contract
description: The Adafruit nRF52 Bootloader HIL contract - roster, selection, build record, battery, locks, report and recovery under agentrc's `hil` rules. Use when a change needs hardware evidence on a rig board, when a validation or debug session must take, flash, clean up or release the rig's nRF52840 DK, or when a workflow asks for "the project's HIL contract".
---

# HIL contract

`scripts/hil_test.py` holds the mechanics: admission of the build record, the lock, the J-Link and USB steps, cleanup
and the JSON report. Agents call it instead of composing J-Link, mtools or lock commands. This page is the bootloader's
contract under agentrc's `hil` skill, which is read first.

## Hosts/rosters

- One host: the ci.lan rig (`hostname` `ci`), roster `test/hil/ci.json`, the only tracked roster. A bench may keep an
  untracked `test/hil/local.json` (`.gitignore`) of the boards attached there; elsewhere a run fails at `install`,
  where no probe with the roster's serial is found, before anything is written.
- An entry: `name` the bootloader's board (`src/boards/<name>/`), `uid` the bootloader's USB serial (FICR DEVICEID as
  16 hex digits, `src/usb/usb_desc.c`), `probe` the J-Link serial registered in agentrc's board registry, which gives
  the flasher (`-device nrf52840_xxaa`) and the lock name, and `nrf52.reentry` (`gpregret`) once commissioned
  (`commissioning.md`). An entry without it is refused by `run`.
- The DK (`pca10056`, probe 683944004, lock `nrf52840dk`) is shared with tinyusb, Adafruit TinyUSB Arduino and the
  Adafruit nRF52 Arduino core, each taking the same lock.

## Selection

`python3 .claude/skills/build/scripts/check_build.py --select-only --base origin/master --hil-config test/hil/ci.json`
prints `hil.boards`, the roster boards a change puts under the battery, `hil.full`, and `hil.uncovered`: each board a
change reaches directly with no roster board, and for a shared change each MCU variant (nRF52832, nRF52833) no roster
board covers. Uncovered is reported pending, never passed. The battery is fixed; there is nothing else to select.

## Build

The run's firmware is the build contract's HIL record of a clean tree at HEAD:

```bash
python3 .claude/skills/build/scripts/check_build.py --board pca10056 --shared --hil-config test/hil/ci.json \
  > cmake-build/pca10056.hil.json
```

The record binds HEAD, the submodules, the roster's sha256, the agentrc identity and every artifact's sha256
(`.claude/skills/build/SKILL.md`, A HIL record). `hil_test.py` refuses a record that is not passing, dirty, of another
HEAD, submodules, roster or agentrc, or whose artifacts no longer hash as recorded, and copies the artifacts into the
run dir before use. What the battery checks comes from the record and this checkout: the version from the image's
`UF2 Bootloader <version>` string, VID/PID and `UF2_BOARD_ID` from `board.h`, the app start (`MBR_SIZE` plus the
SoftDevice's `SD_FLASH_SIZE`) and the RAM end from the record's SoftDevice and the MCU's linker script.

## Execution

On the rig, from a clean checkout at the record's HEAD (exit 2 otherwise):

```bash
python3 .claude/skills/hil-contract/scripts/hil_test.py run -b pca10056 \
  --build-record cmake-build/pca10056.hil.json [--lock-wait S] [--run-root DIR] test/hil/ci.json
```

`run` takes the lock itself (no pre-hold) and runs one board for up to about four minutes; its last stdout line is the
JSON report and its exit is final. The battery, each step judged from its own evidence:

1. `install`: ERASEALL through NVMC with the CPU halted (nRF52840 errata [184]), `READY` read back, the merged HEX
   (MBR, SoftDevice, bootloader, UICR) loaded and each of its runs verified with `verifybin`, then reset.
2. `dfu`: the UF2 drive with the board's VID:PID and roster serial; `INFO_UF2.TXT` names the record's version,
   `UF2_BOARD_ID` and SoftDevice.
3. `copy`: a generated app A (vector table at the app start, a counting loop, a 4 KiB pattern from a random nonce) as
   UF2 with the board's family ID, written with `mcopy` and flushed; the drive leaves.
4. `app-ran`: no probe reset first; the drive stays away, and two register samples show thread mode, PC in A's loop and
   R0 advancing.
5. `reenter`: GPREGRET = 0x57 written and read back, then SYSRESETREQ (a soft reset keeps GPREGRET); the drive returns
   and `CURRENT.UF2` reads back A's bytes at the app start. Forced entry, not a double reset.
6. `copy-b`, `app-ran-b`: app B from another nonce runs, so a stale image cannot pass.

`--fail-at STEP` fails that step on purpose, to exercise cleanup.

## Reservation

- Lock helper: agentrc's `hil_lock.py` through `scripts/agentrc.py`; lock name `nrf52840dk` from the registry.
- `run` locks itself (`take_board_lock`, protected) and releases after a passing cleanup. A board another `JLinkExe`
  is using is not run (`probeUsers`).
- Manual sessions (commissioning, debugging) hold first, then work under the hold:

```bash
python3 ~/.claude/skills/hil/scripts/hil_lock.py hold nrf52840dk --reason <reason>   # foreground; or under nohup
python3 .claude/skills/hil-contract/scripts/hil_test.py install|restore -b pca10056 \
  --build-record cmake-build/pca10056.hil.json --under-hold <reason> test/hil/ci.json
python3 ~/.claude/skills/hil/scripts/hil_lock.py release nrf52840dk
```

  `install` writes the candidate and checks its drive; `restore` is the cleanup below. Release only after a passing
  `restore`.
- Cleanup (`cleanup` in the row): untouched when nothing was written; otherwise the candidate installed again and a
  benign generated app copied through it and proven running (`fixture running`). The board is then left with MBR,
  SoftDevice, the candidate bootloader, UICR pointing at it, and that app: tinyusb and the Arduino HILs flash their
  own images over it (the nRF52 Arduino core's base check rewrites MBR, SoftDevice, bootloader and settings).

## Results

`<run-root>/run-<time>-<pid>/hil_report.json` and `.md` through agentrc's report core (`hil` Reporting), body schema
`nrf52-bootloader-hil-run`; the same JSON is the last stdout line. Fields: `pass`, `exitCode`, `caveat`, `error`,
`reportPath`, and per board `results[]`: `ran`, `pass`, `locked` (with `recoveryBlocker`), `probeUsers`, `detail`,
`firmware` (version, record, merged sha256, head), `steps[]` (`step`, `pass`, `detail`, `evidence`, `seconds`) and
`cleanup` (`pass`, `state`, `heldAfter`, `holder`).

Exits: 0 pass and cleaned up; 1 a step failed, the board did not run, or it was locked; 2 refused before any hardware
access; 4 cleanup failed and the board is held by a cleanup holder. `scripts/hil_run_report.py`'s `verdict()` maps a row
onto the `hil` verdicts: pass; fail (a step failed, cleanup passed); not-run (locked, refused, or failed with nothing
written); blocker (exit 4, `heldAfter`, a recovery blocker, another probe user, no report). A delegated run
(`hil-operator`) adds nothing to the native report; its caller retries a fail once per `hil` Caller rules.

## Recovery

- Exit 4 or `heldAfter`: the board is held by the cleanup holder (`hil_lock.py status` shows `role:
  cleanup-holder`). A `hw-validator` or the human takes it over (`hil_lock.py hold --take-over nrf52840dk --reason
  <reason>`), runs `restore --under-hold <reason>` with a record of a known-good HEAD, and releases only after it
  passes. A failing restore leaves the board held and goes to the human.
- A J-Link that no longer connects: agentrc's `usb-kernel-recover` rungs, then the registry's `flasher_recover`
  (openocd `nrf52.cfg`). An APPROTECT recover is never automatic.
- A UF2 drive that never enumerates after a passing `install`: check the DK's nRF USB cable (J3) and the hub port;
  `usb-kernel-recover`.

## Limits

- Covered: the nRF52840 with S140 6.1.1 on the DK, UF2 DFU end to end, GPREGRET entry and readback. Uncovered, named
  pending in every report that needs them: nRF52832 (serial DFU only, no USB), nRF52833, BLE OTA, CDC serial DFU,
  SIGNED_FW, dual bank, a physical double reset, an interrupted update and bootloader self-update.
- `INFO_UF2.TXT` is checked by its lines, not the drive's other files; `CURRENT.UF2` only over the app's span.
- Commissioning: `commissioning.md`.
