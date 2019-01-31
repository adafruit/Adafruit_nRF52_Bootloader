# Adafruit nRF52 Bootloader Changelog

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

