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
  the flasher (`-device nrf52840_xxaa`) and the lock name, and once commissioned (`commissioning.md`)
  `nrf52.reentry` (`gpregret`) and `nrf52.selfUpdateBase`, the `{version, file, sha256}` of a released
  `update-<board>_bootloader-<version>_nosd.uf2`. `run` refuses an entry without them.
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
mkdir -p cmake-build && python3 .claude/skills/build/scripts/check_build.py --board pca10056 --shared \
  --hil-config test/hil/ci.json > cmake-build/pca10056.hil.json
```

The record binds HEAD, the submodules, the roster's sha256, the agentrc identity and every artifact's sha256 of each
HIL profile (`.claude/skills/build/SKILL.md`, A HIL record). `hil_test.py` refuses a record that is not passing, dirty,
of another HEAD, submodules, roster or agentrc, whose profile is not built as this checkout defines it (options, test
key) or whose artifacts are incomplete or no longer hash as recorded, and copies the profile's artifacts into the run
dir's `<profile>/` before use; a profile other than `default` also admits the default's, which its cleanup
reinstalls. Before any hardware access it checks the merged
HEX's spans: only flash and UICR NRFFW, runs at 0 (MBR), 0x1000 (SoftDevice) and the bootloader start, nothing in the
app region, and NRFFW[0] pointing at the bootloader. What the battery checks comes from the record and this checkout:
the version from the image's `UF2 Bootloader <version>` string, VID, the UF2 and CDC-only PIDs and `UF2_BOARD_ID` from
`board.h`, the app start (`MBR_SIZE` plus the SoftDevice's `SD_FLASH_SIZE`) and the RAM end from the record's SoftDevice
and the MCU's linker script.

## Execution

On the rig, from a clean checkout at the record's HEAD (exit 2 otherwise):

```bash
python3 .claude/skills/hil-contract/scripts/hil_test.py run -b pca10056 \
  --build-record cmake-build/pca10056.hil.json --base-dir <dir> [--profile P] [--lock-wait S] [--run-root DIR] \
  test/hil/ci.json
```

`--base-dir` holds the roster's `selfUpdateBase` file, downloaded from that release; `run` refuses bytes that do not
hash as pinned and copies them into the run dir. `adafruit-nrfutil` (the farm's hil-venv) and `mcopy` must be on
`PATH`. `run` takes the lock itself (no pre-hold) and runs one board for up to about five minutes; its last stdout line
is the JSON report and its exit is final. `--profile` (default `default`) picks the record's profile and its
battery. The `default` battery, each step judged from its own evidence:

1. `install`: ERASEALL through NVMC with the CPU halted (nRF52840 errata [184]), `READY` read back, the merged HEX
   (MBR, SoftDevice, bootloader, UICR) loaded and each of its runs verified with `verifybin`, then reset.
2. `dfu`: the UF2 drive with the board's VID:PID and roster serial; `INFO_UF2.TXT` names the record's version,
   `UF2_BOARD_ID` and SoftDevice.
3. `copy`: a generated app A (vector table at the app start, a reset handler that loads A's marker word into R1 and
   counts in R0, a 4 KiB pattern from a random nonce) as UF2 with the board's family ID, written with `mcopy` and
   flushed; the drive leaves.
4. `app-ran`: no probe reset first; the drive stays away, and two register samples show thread mode, PC in the counting
   loop, R1 A's marker and R0 advancing.
5. `reenter`: GPREGRET = 0x57 written and read back, then SYSRESETREQ (a soft reset keeps GPREGRET); the drive returns
   and `CURRENT.UF2` reads back A's bytes at the app start. Forced entry, not a double reset.
6. `copy-b`, `app-ran-b`: app B, with another nonce and marker, runs, so a stale image cannot pass.
7. `double-reset`: nRESET pulses from J-Link `r0`/`r1` in a session that never connects to the target (the DK's
   interface MCU drives nRESET: DK guide, book 1602 pp14, 25). `src/main.c` enters DFU on a pin reset while it still
   waits 500 ms after the previous one, its magic word kept in RAM (nRF52840 PS v1.7 5.3.6.8: a pin reset never resets
   RAM, though its content may be corrupted; RESETREAS is cumulative, 5.3.7.11). Controls first, each leaving app B
   running: one pulse, then two 1500 ms apart; then two 150 ms apart bring the drive back, `INFO_UF2.TXT` as in `dfu`.
8. `serial`, `app-ran-c`: GPREGRET = 0x4E and SYSRESETREQ enumerate the bootloader as the CDC-only device (its PID and
   the roster serial, one tty, no drive); `adafruit-nrfutil dfu genpkg --dev-type 0x0052` packages app C and `dfu
   serial --singlebank` sends it ("Device programmed." required: it exits 0 on a failed upgrade); app C runs.
9. `update-base`: from the drive (re-entered through GPREGRET), the pinned release's update UF2 copied: the bootloader
   stages it at the top of the app region and the MBR copies it over itself; app C still runs, and re-entered, the
   drive names the release's version. `update`: the record's `update-*_nosd.uf2` the same way, back to the candidate's
   version, so the candidate both updates and is updated.
10. `interrupted`: an interrupted UF2 transfer. From the drive, app D, three flash pages and more (nRF52840 PS v1.7
   4.2.2: 4 kB pages), is copied only up to the first block of its third page; the drive stays, the bootloader having
   written a page only once the transfer moved past it (`src/flash_nrf5x.c`). J-Link reads the bootloader settings
   (`0xFF000`) and D's first two pages back (`verifybin`) and not its third, then resets. The outcome: bank 0 stays
   valid with CRC 0, which no transport computes (`m_image_crc` in `dfu_single_bank.c`, `src/usb/msc_uf2.c`), so
   `bootloader_app_is_valid` checks nothing and the bootloader starts the mixed image; D's vectors and code lie in its
   first page, so D runs. Then the whole of D, copied through the re-entered drive, verifies and runs.
11. `softdevice`: the record's release package (`genpkg --softdevice --bootloader`) through CDC-only serial DFU, a
   same-version rewrite of S140 6.1.1 and the candidate (no other SoftDevice is documented for a 6.1.1 build). The
   bootloader erases the app region to receive them, swaps them in through the MBR on the next boot and marks bank 0
   invalid (`DFU_UPDATE_SD_SWAPPED`), so it comes back in DFU by itself; every run of the merged HEX, SoftDevice and
   bootloader included, verifies, `INFO_UF2.TXT` names the candidate and SoftDevice, and app E copied through the
   drive runs.

The `dualbank` battery (`DUALBANK_FW=ON`, `dfu_dual_bank.c`), its cleanup reinstalling the `default` image:

1. `install`, `dfu` as above.
2. `serial`, `app-ran`, `serial-b`, `app-ran-b`: apps A, then B, through CDC-only serial DFU without `--singlebank`.
   The bootloader receives each into bank 1 (`DFU_BANK_1_REGION_START`: half the app region less
   `DFU_APP_DATA_RESERVED`, rounded down to two 4 kB pages; `0x88000` here), checks the init packet's CRC-16 and on
   activation copies bank 1 into bank 0, then resets. Once the CDC device left, J-Link verifies the app in both banks
   and bank 0 marked valid in the settings; the app runs.
3. `refused`: app C in a package whose init packet CRC-16 is flipped. The bootloader receives C into bank 1 and
   `dfu_init_postvalidate` refuses it, so it never activates: the CDC device stays (no activation reset;
   adafruit-nrfutil reports success either way), J-Link verifies C in bank 1 and B in bank 0, still marked valid,
   then resets, and B runs (README, Dual bank firmware support).
4. `interrupted`: an interrupted serial transfer. App D, about 240 KiB, is sent until adafruit-nrfutil's 12 s bound
   ends it (`CUT_AFTER`: about 6 s of erase wait and 16 s of sending at the DK's ~15 KB/s); J-Link verifies D's first
   page in bank 1 and not the start of its last, B in bank 0 still marked valid, then resets, and B runs.

The `signed` battery (`SIGNED_FW=ON` with the public key of `test/hil/signed-fw-test-key.json`, a test-only key;
`src/dfu_init.c`), its cleanup reinstalling the `default` image. The harness signs: adafruit-nrfutil's init packet
without its CRC-16, then the ECDS identifier, the image length and SHA-256, and an ECDSA P-256 signature (r || s) over
all of that, which `dfu_init_prevalidate` verifies before any data and `dfu_init_postvalidate` checks the image
against:

1. `install` as above; `dfu`: with no app and no drive in this build (`src/usb/tusb_config.h`), one CDC port on the UF2
   PID.
2. `serial`, `app-ran`: app A, signed with the test key, through CDC-only serial DFU; A runs.
3. `unsigned`, `wrong-key`, `tampered`: app C unsigned, app D signed by a random key, app E whose image changed after
   signing, each through CDC-only serial DFU. The start packet already erased bank 0 and marked it invalid
   (`dfu_single_bank.c`, `DFU_BANK_0_ERASED`), so a refused package leaves no app. The first two fail the signature
   check, which ends in `app_error_fault_handler`: a reset, or `BKPT` while a debugger left DHCSR's `C_DEBUGEN` set,
   which halts the core in Debug state (ARMv7-M ARM DDI 0403D C1.5, C1.6.2), so DHCSR's `S_HALT` is recorded. The
   third is received and fails the SHA-256 check, never activated. J-Link reads bank 0 marked invalid (`0xFF`) and at
   the app start erased flash, or E's received image, then resets; the bootloader is back in DFU. adafruit-nrfutil's
   own report is not judged.
4. `serial-b`, `app-ran-b`: app B, signed with the test key, installs and runs.

`--fail-at STEP` fails that step on purpose, to exercise cleanup. Every tool (JLinkExe, mcopy, adafruit-nrfutil, the FAT
reader and flush) runs under agentrc's `hil_process.run_tracked` with its files in the run dir's `ops`
(`hil_process.state_dir`); one whose process tree is not proven ended stops the battery, skips the restore and its
receipt goes to the board's cleanup log. Before its first tool, every `run`, `install` and `restore` claims the board
(agentrc `hil` Board locks): its pid, start time and ops dir in `<lock>.nrf52-bootloader-pending.json`
(`hil_lock.PendingSessions`, the record older harnesses' pending marks used), refused when that cannot be written.
Another invocation still alive refuses it (two commands under one hold). An earlier one that ended with a tool not
proven ended has that tool's receipt logged in the board's cleanup log, its claim removed, and the tool, and any
`JLinkExe` on the board's probe, get 30 s to end (`hil_test.py` `CLAIM_SETTLE`); still there, the attempt is not run,
and the next attempt no longer waits on it. An invocation removes its own claim once its own tools recheck `done`, and
keeps it otherwise for the next claim to judge.

### CI

`.github/workflows/hil.yml` runs the battery for a pull request from this repository (never a fork) and for a dispatch
naming boards (`fail_at` for acceptance only). `select` runs the selector above against the merge base and reports a
board pending commissioning as uncovered; `hil-build` builds each board's HIL record on a hosted runner with CI's
arm-gcc and the agentrc pin, downloads each board's `selfUpdateBase` release asset into `cmake-build/hil-base/`, and
uploads them with the record's artifacts; `hil-run`, on the rig's `nrf52-bootloader` runner (agentrc `hil-farm-setup`),
checks out the same commit, fetches `lib/nrfx` and the pin, and runs `run` per board with `--base-dir
cmake-build/hil-base` and a 30-minute lock wait into `$(dirname "$GITHUB_WORKSPACE")/hil-state/<run>-<attempt>/<board>`,
uploaded as `hil-run-<attempt>`; the rig builds nothing. `pr-comment` posts the sticky `hil-report` comment
(`hil_comment.py`) only from the newest attempt of the newest run for the PR's head, judging each board's newest report
by the report core's shared run rules. Runs are never cancelled: a newer push waits.

## Reservation

- Lock helper: agentrc's `hil_lock.py` through `scripts/agentrc.py`; lock name `nrf52840dk` from the registry.
- `run` locks itself (`take_board_lock`, protected). The lock ends with the run: after a passing cleanup the open failed
  cleanups it found (`previousCleanupFailure`) whose receipts recheck `done` are marked restored (`resolve_restored`)
  and the lock released; after a failed one `end_failed` logs it in the board's cleanup log, with the receipts of tools
  not proven ended, and releases. Nothing holds the board after a run. A board in use (Execution) is not run
  (`locked`, or `occupiedBy` when an earlier invocation's tool or another `JLinkExe` stayed).
- Manual sessions (commissioning, debugging) hold first, then work under the hold:

```bash
python3 ~/.claude/skills/hil/scripts/hil_lock.py hold nrf52840dk --reason <reason>   # foreground; or under nohup
python3 .claude/skills/hil-contract/scripts/hil_test.py install|restore -b pca10056 \
  --build-record cmake-build/pca10056.hil.json --under-hold <reason> test/hil/ci.json
python3 ~/.claude/skills/hil/scripts/hil_lock.py release nrf52840dk
```

  `install` writes the candidate and checks its drive; `restore` is the cleanup below, and once it passes marks
  restored the board's open failed cleanups whose receipts recheck `done`. Release only after a passing `restore`.
  Neither ever ends the hold: a failed `restore`, or a tool not proven ended, is logged in the board's cleanup log
  (`log_cleanup_failure`, `failureLogged`) and exits 4, the hold staying the caller's.
- Cleanup (`cleanup` in the row): untouched when nothing was written; otherwise the candidate installed again and a
  benign generated app copied through it and proven running (`fixture running`). A failed step, an unexpected error
  inside a step or a failed check is restored the same way; an interrupt, a tool not proven ended or a failed restore
  leaves `unknown`, logged in the board's cleanup log with the lock released (exit 4). The board is then left with MBR,
  SoftDevice, the candidate bootloader, UICR pointing at it, and that app: tinyusb and the Arduino HILs flash their
  own images over it (the nRF52 Arduino core's base check rewrites MBR, SoftDevice, bootloader and settings).

## Results

`<run-root>/run-<time>-<pid>/hil_report.json` and `.md` through agentrc's report core (`hil` Reporting), body schema
`nrf52-bootloader-hil-run`; the same JSON is the last stdout line. Fields: `pass`, `exitCode`, `caveat`, `error`,
`reportPath`, `uncovered` (what the rig cannot establish, Limits), and per board `results[]`: `ran`, `pass`, `locked`
(with `recoveryBlocker`), `occupiedBy`, `detail`, `claimNote`, `previousCleanupFailure` and `openCleanupFailures` (the
board's open failed cleanups the run found, `hil_lock.failure_report`), `firmware` (version, record, merged sha256,
head), `steps[]` (`step`, `pass`, `detail`, `evidence`, `seconds`) and `cleanup` (`pass`, `state`, `receipts`,
`released`, `failureLogged` and `endErrors` after a failed cleanup, `restoredNote` naming failures kept open).

Exits: 0 pass and cleaned up; 1 a step failed, the board did not run, or it was locked; 2 refused before any hardware
access; 4 cleanup failed, logged in the board's cleanup log (`failureLogged`) and the lock released.
`scripts/hil_run_report.py`'s `verdict()` maps a row onto the `hil` verdicts: pass; fail (a step failed, cleanup
passed); not-run (locked, refused, or failed with nothing written); blocker (exit 4 or a failed cleanup, a recovery
blocker, `occupiedBy`, no report). A delegated run (`hil-operator`) adds nothing to the native report; its caller
retries a fail once per `hil` Caller rules.

## Recovery

- Exit 4, a `previousCleanupFailure`, or `hil_lock.py status` naming an open failed cleanup: the board's cleanup log
  (`/tmp/tinyusb-hil-locks/nrf52840dk.cleanup.jsonl`) keeps it and nothing holds the board, so no unlock step follows.
  The next `run` installs from an erase anyway and, once its cleanup passes, marks it restored when its receipts
  recheck `done`; to restore without a run, hold, `restore --under-hold`, release (Reservation). A failure that stays
  open (`restoredNote`) names a tool not proven ended: once no `JLinkExe`, `mcopy`, `adafruit-nrfutil` or
  `hil_test.py` child of it is left on the host, a passing `restore` resolves it; a tool that never ends (D state) is
  `usb-kernel-recover`'s, then the human's.
- `recoveryBlocker`: an older helper's cleanup holder still holds the lock (`hil_lock.py status` shows `role:
  cleanup-holder` and its pid) and never frees on its own. Once no tool of its run is left on the host, a
  `hw-validator` or the human runs, with that run's record or one rebuilt on a known-good clean HEAD:

  ```bash
  python3 .claude/skills/hil-contract/scripts/hil_test.py restore -b pca10056 --build-record <record> \
    --recover-holder <pid> test/hil/ci.json
  ```

  It proves `<pid>` is the board's live cleanup holder, restores under it, and only after the restore passed ends the
  holder by its token (`recovered.released`). A failed restore (exit 4) is logged and leaves the holder: the board goes
  to the human. A dead holder's record on a free lock needs nothing: the next taker logs it as a failed cleanup.
- `occupiedBy`: an earlier invocation's tool, or another `JLinkExe` on the probe, outlived the 30 s settle; the earlier
  invocation's failure is logged and its claim removed, so a later attempt does not wait on it again (the failure stays
  open until a restore resolves it); a `JLinkExe` still on the probe refuses every attempt.
- A J-Link that no longer connects: agentrc's `usb-kernel-recover` rungs, then the registry's `flasher_recover`
  (openocd `nrf52.cfg`). An APPROTECT recover is never automatic.
- A UF2 drive that never enumerates after a passing `install`: check the DK's nRF USB cable (J3) and the hub port;
  `usb-kernel-recover`.

## Limits

- Covered: the nRF52840 with S140 6.1.1 on the DK, UF2 DFU end to end, GPREGRET entry (UF2 and CDC-only) and
  readback, a double reset on the reset pin, CDC serial DFU of an app, bootloader self-update from the pinned release
  and back, an interrupted UF2 transfer (the bootloader starts the partly written image), a same-version SoftDevice
  and bootloader rewrite through the release package; on a dual-bank build an app's serial DFU, a refused and an
  interrupted one; on a SIGNED_FW build a signed app and the refusal of unsigned, wrong-key and tampered packages.
  Uncovered, named pending in every report that needs them: nRF52832 (serial DFU only, no USB), nRF52833, BLE OTA,
  another SoftDevice version than the build's, and a SoftDevice or bootloader update on a dual-bank or SIGNED_FW
  build.
- `INFO_UF2.TXT` is checked by its lines, not the drive's other files; `CURRENT.UF2` only over the app's span.
- Commissioning: `commissioning.md`.
