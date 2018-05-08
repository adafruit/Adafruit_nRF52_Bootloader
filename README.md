# Adafruit nRF52840 Bootloader

This repository contains the USB bootloader for Adafruit nRF52840 boards (1MB flash, 256KB SRAM).
This repository depend on the  [tinyusb](https://github.com/hathach/tinyusb/tree/develop) as submodule, after cloing this repo you need to run this command

    git submodule update --init

## Build with makefile

Navigate to `src/singlebank` (recommended) or `src/dualbank` (work in progress), and use feather52840 target for building bootloader.

	make feather52840

To flash bootloader + S140

	make flash_feather52840
    
To only flash S140

	make flash_sd

## Build using Segger Embeded Studio

For better debugging you can also use SES, The project file is located at `src/segger/nrf52840_bootloader.emProject`.  Note: the SES only flash the bootloader when you click download, you need to flash Softdevice beforehand if you haven't done so ( use `make flash_sd` mentioned above )

