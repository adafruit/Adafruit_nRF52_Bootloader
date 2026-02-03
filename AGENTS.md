# Agent Instructions

## Style
- Follow the repo `.clang-format` when making changes.

## Build / Verify
- Prefer CMake for testing/verification.
- Always build both boards during verification:
  - `feather_nrf52832`
  - `feather_nrf52840_express`

### CMake (preferred)
```bash
cmake -S . -B cmake-build-feather_nrf52832 -DBOARD=feather_nrf52832
cmake --build cmake-build-feather_nrf52832

cmake -S . -B cmake-build-feather_nrf52840_express -DBOARD=feather_nrf52840_express
cmake --build cmake-build-feather_nrf52840_express
```

### Make (alternate)
```bash
make BOARD=feather_nrf52832 all
make BOARD=feather_nrf52840_express all
```
