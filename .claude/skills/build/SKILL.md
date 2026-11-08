---
name: build
description: The Adafruit nRF52 Bootloader build contract. Use when a change must be verified by building, when an agent needs the boards a diff affects, or when a workflow asks for "the project's build". Resolves scope to boards, builds each with CMake the way CI does in a private dir, reports pass/fail with the first error.
---

# Build contract

`scripts/check_build.py` is the entry point for verifying a change; every agent and workflow that does so calls it
rather than composing cmake or make lines. A debug build for a probe session is not verification and uses CMake
directly (below).

```bash
B=.claude/skills/build/scripts/check_build.py
python3 $B --scope <changed paths or dirs>     # boards a change affects
python3 $B --base origin/master                 # same, from the branch diff
python3 $B --base origin/master --worktree      # same, plus uncommitted and untracked changes
python3 $B --select-only (--scope ... | --base REF [--worktree] | --diff-file FILE) [--hil-config ROSTER]
python3 $B --board feather_nrf52840_express [--board ...]
python3 $B --board feather_nrf52840_express -D DUALBANK_FW=ON   # a CMake option
python3 $B --board pca10056 --shared --hil-config test/hil/ci.json   # the HIL's build record
```

The last stdout line is JSON. An agent acts on `pass`; `firstError`, and `error` on exit 2, name the cause or the
remedy; `boards[]` (`status` `ok` or `failed`, `log`, `artifacts`, `buildDir`, `firstError`; `backend: make` for a build
through the Makefile wrapper); `tests` when the contract tests ran; `nothingToBuild`, each changed path with its reason;
`paths`, every changed path's `rule` and `reason`, the answer to why a path selected what it did. `input` records what
the build used: HEAD, `dirty` and `dirtyInputs`, submodule commits, and the arm-gcc, cmake, ninja, adafruit-nrfutil and
Python versions. Exit 0 pass, 1 a build or the tests failed, 2 usage, resolution or a missing prerequisite.

`--select-only` prints the selection (`build.{boards,make,tests}`, `paths`) and stops before any toolchain check. With
`--hil-config <the host's roster>` it adds `hil.{boards,full,uncovered}`: the roster boards the change puts under the
HIL battery, and as uncovered each board a change reaches directly (its own files, a SoftDevice it uses) and, for a
shared change, each MCU variant that no roster board on this host covers. The mandated boards a build adds are build
coverage, not HIL scope.

## Reporting

An agent that reports `buildOk` and `notes` (agentrc's `code-writer`) maps the JSON this way: `buildOk` is `pass`. A
pass with nothing built is "nothing to build": quote each `nothingToBuild` reason in `notes`, so nobody mistakes it for
a build that ran. Exit 1 or 2 quotes `firstError` (and `error`).

## Judgment

- **Selection is the script's**, first match per path; `paths[].reason` says why. A mixed change takes the union.
  - A board's files (`src/boards/<board>/`, the directory itself included) build that board.
  - Shared code (`src/`, `lib/`, `linker/`, `CMakeLists.txt`, `cmake/`, `tools/hexmerge.py`) and anything
    unclassified build the representatives: one board per MCU variant (feather_nrf52840_express, feather_nrf52832,
    pca10100 for nRF52833) and the rig's pca10056. A SoftDevice under `lib/softdevice/<sd>/` builds the boards that
    use it.
  - Every selection that builds firmware also builds feather_nrf52840_express and feather_nrf52832.
  - `Makefile`, the CMake wrapper: the contract tests (`test_makefile.py` runs it with stub flash tools) and a build of
    those two boards through it; `tools/build_all.py` those two boards.
  - This script, the HIL contract's scripts, `test/hil/` (the fixture included) and `hil.yml` run the contract tests,
    build the representatives and select every roster board for HIL; `githubci.yml` runs the tests and the mandated
    boards; `.claude/test/` and `.pre-commit-config.yaml` the tests alone. No firmware build verifies workflow syntax.
  - Docs and agent, editor and repo metadata (`*.md` at the top level and under `.claude/skills/<skill>/`, `.idea/`,
    `.github/ISSUE_TEMPLATE/`, ...) build nothing.
- **Builds are CI's.** `cmake -S . -B <dir> -G Ninja -DBOARD=<board>` then `cmake --build <dir>`, the build type
  CMakeLists.txt defaults (MinSizeRel), with compiler and CMake inputs (`CFLAGS`, `LDFLAGS`, `CPATH`,
  `LIBRARY_PATH`, `CMAKE_*` and the like, `SCRUBBED_ENV`) removed from the environment. Each board's `config` records
  the CMake cache entries that configured it (board, generator, build type, objcopy, nrfutil, options), and
  `input.sourceDateEpoch` the `SOURCE_DATE_EPOCH` the build saw.
  A board passes only with every artifact (`expected_artifacts()`): `bootloader.elf` and its map, `bootloader.hex`,
  `bootloader_mbr.hex` and `.uf2`, and the release files `<board>_bootloader-<version>_<sd>_<sdversion>.hex`
  (bootloader, MBR and SoftDevice), its `.zip` DFU package and `update-<board>_bootloader-<version>_nosd.uf2`. CI's
  compiler is arm-gcc 12.3.Rel1; `input.tools` records the one used here.
- **The `Makefile` wraps CMake.** `make BOARD=<board>` configures `_build/build-<board>` from an empty cache with its
  variables as `-D` and builds through CMake; a board's override in `board.cmake` is guarded with `if (NOT DEFINED
  ...)` so `-D` wins. `__DATE__`/`__TIME__` (`src/usb/uf2/compile_date.h`) differ between any two builds; set
  `SOURCE_DATE_EPOCH` on both when comparing images.
- **`-D`** takes `NAME=VALUE` for the options CMakeLists.txt defines (`DUALBANK_FW`, `DEFAULT_TO_OTA_DFU`, `SIGNED_FW`
  with `SIGNED_FW_QX`/`SIGNED_FW_QY`, `FORCE_UF2`, `SD_VERSION`, `USE_NFCT`); anything else is refused. ANT
  (`ANT_LICENSE_KEY`, `SD_NAME=s340`) is not offered: the s340 SoftDevice is license-excluded from the repository.
- **A HIL record** (`--board <b> --shared --hil-config <roster>`) builds every HIL profile of a clean tree, each in its
  own dir: `default` in `cmake-build/cmake-build-<b>`, `dualbank` (`DUALBANK_FW=ON`) and `signed` (`SIGNED_FW=ON` with
  the published test key `test/hil/signed-fw-test-key.json`, never for a product) in `cmake-build-<b>-<profile>`
  (`hil_profiles()`); `-D` and a dirty tree are refused. It adds `hilRecord` (the roster's path and sha256, `agentrc`,
  the identity of the agentrc the HIL contract's `scripts/agentrc.py` selects, `profiles` and the test key's sha256),
  a row per board and profile with its `profile`, `defines`, `sha256` of every artifact and `softdevice` (name,
  version, hex and its sha256), and fails with exit 2 when HEAD, the tree, the roster or an agentrc closure file
  changed while it built.
- **Private dirs for parallel agents.** Each build starts from an empty `cmake-build/cmake-build-agent-<pid>-<board>`
  and is removed when it passes; its `artifacts` say what it produced, nothing is left to flash. A failed one is kept
  for its log (`<buildDir>/check_build.log`; `-v` also streams it to stderr). `--shared` rebuilds
  `cmake-build/cmake-build-<board>` from clean and keeps it; it needs exclusive ownership of that board.
- **Dependencies are never fetched here.** `lib/tinyusb`, `lib/nrfx`, `lib/uf2` and `lib/tinycrypt` are submodules
  (`git submodule update --init`); a firmware build needs arm-none-eabi-gcc, cmake, ninja, `adafruit-nrfutil` and
  Python `intelhex`, checked only when a board is selected (a docs or tests-only change needs none).
- **Debug builds** for a probe session: `cmake -S . -B <dir> -G Ninja -DBOARD=<b> -DCMAKE_BUILD_TYPE=Debug` uses the
  `_debug.ld` linker scripts and RTT (`CFG_DEBUG`), with a larger bootloader region; `make DEBUG=1` is the same at
  `-Os`. A debug image is never a HIL record.
- **Printing a command is not verification.** A verdict is the JSON of a run that happened; an agent that cannot run
  the script reports the build as not run.
