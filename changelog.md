# Adafruit nRF52 Bootloader Changelog

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

