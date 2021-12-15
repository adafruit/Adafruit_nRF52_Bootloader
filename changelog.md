# Adafruit nRF52 Bootloader Changelog

## 0.6.2 - 2021.09.10

- Add new board "LED Glasses Driver nRF52840"

## 0.6.1 - 2021.08.04

- Fix failed to upgrade ssue when flashing uf2 with more than 512KB payload
- Increased application reserved from 28KB  to 40KB for nrf52840 to match circuitpython usage. 

## 0.6.0 - 2021.06.19

- Update tinyusb to latest to fix the race condition with control transfer, which could cause failed to connect to DFU Serial occasionally.

## 0.5.1 - 2021.06.02

- more boards support 

## 0.5.0 - 2021.04.04

- Update tinyusb to latest to fix the race condition causing cdc out dropping packet. Which cause DFU failed occasionally
- Add support for Dotstar LED (APA102)
- Add sparkfun_nrf52840_micromod board
- Allow skipping DFU entirely when reset or wakeup from deep sleep
- Fix an issue with OTA when using shared bond with application

## 0.4.1 - 2021.02.17

- Add USB connect timeout for app reset to UF2 or Serial via GPREGRET
- Fix pollution of REGOUT0 reserved bits

## 0.4.0 - 2021.01.19

- Decouple bootloader and softdevice i.e bootloader will always work with and/or without softdevice present. This allows application to pack firmware + softdevice into an uf2/serial for DFU.
- Add self-update feature with update-{board}.uf2. This allow bootloader to update itself easily (requires running bootloader with at least 0.4.0)
- Enlarge the fake FAT disk to ~ 32MB to allow piggy-pack multiple application (different family ID) within the same uf2.
- Support uf2 with family ID = vendor ID + product ID
- Reset into application if there is no USB connection in ~3 seconds.
- Remove DFU idle 300 seconds timeout
- Support power supply configuration with `ENABLE_DCDC_0` and `ENABLE_DCDC_1`
- Add new boards support: nice nano, bast ble, ikigaisense vita, nrf52840 M2, Pitaya Go, AE-BL652-BO, BlueMicro, ADM_B_NRF52840_1

## 0.3.2 - 2020.03.12

- Make sure all pins are in reset state when jumping to app mode.

## 0.3.1 - 2020.03.05
- Correct LED1 for Feather Sense and change volume name to FTHRSNSBOOT.

## 0.3.0 - 2020.01.13

- Upgrade nrfx to v2 for supporting future nrf mcu such as nrf52833, nrf5340
- Upgrade TinyUSB
- New boards support
  - Arduino ble nano 33
  - Adafruit CLUE
  - Raytac MDBT50Q-RX dongle
- Migrate CI to github Action

## 0.2.13 - 2019.09.25

- Fixed button pulldown with Adafruit Circuit Playground Bluefruit, PR #77 thanks to @khanning

## 0.2.12 - 2019.08.22

- Fixed block alignment bug also in dfu_sd_image_validate (single & dual banks), PR #71 thanks to @fanoush
- Added new board: Adafruit Circuit Playground Bluefruit
- Added new board: Adafruit ItsyBitsy nRF52840 Express (WIP)
- Fixed bug in __DOSTIME__ macro, PR #75 thanks to @henrygab

## 0.2.11

- Fixed various FAT issues, thanks to @henrygab
- Added MakerDiary MDK nrf52840 USB dongle support, thanks to @gpshead
- Fixed incorrect button mapping for Feather nRF52840
- NFC pins are forced to GPIO mode by bootloader
- Added Metro nRF52840 Express VID/PID
- Enhance board management
- Added electronut/papyr_support

## 0.2.9

- Fixed incomplete writes on Windows. Upated tinyusb to handle write10 completion, and use it for finalizing dfu process. Will prevent windows error message pop up. This will also give an extra time for flash writing to complete.

## 0.2.8

- Fixed OTA issue with nrfConnect on iOS
- Increased LED conn cycle when ble connected for easier recognition

## 0.2.7

- Fixed PWM psel[1] is not reset
- Fixed #41 move RXD, TXD into board header
- Added Metro nRF52840 Rev A
- Fixed #40 OTA issue with BLE_GAP_EVT_PHY_UPDATE_REQUEST e.g connecting with iPhone X

## 0.2.6

- Fixed copy/restore current.uf2 issue
- Fixed neopixel won't turn off after dfu cdc

## 0.2.5

- Make led pattern more consistent
- Fixed issue nrf52840 not reset properly when upgrading bootloader+sd combo
- Make led pattern more consistent

## 0.2.3

- Fully support Feather nRF52840
- Update bootloader with new led pattern
- Fix #203: return software timer handle
