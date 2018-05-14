# Adafruit nRF52840 Bootloader

This repository contains the USB bootloader for Adafruit nRF52840 boards (1MB flash, 256KB SRAM).
This repository depend on the  [tinyusb](https://github.com/hathach/tinyusb/tree/develop) as submodule, after cloing this repo you need to run this command

    git submodule update --init

## Option 1: Build with makefile

Navigate to `src/singlebank` (recommended) or `src/dualbank` (work in progress), and use feather52840 target for building bootloader.

	make feather52840

To flash bootloader + S140

	make flash_feather52840

To only flash S140

	make flash_sd

### Common makefile problems

#### 1. `arm-none-eabi-gcc`: No such file or directory

If you get the following error:

```
$ make feather52840
Compiling file: dfu_single_bank.c
/bin/sh: /usr/bin/arm-none-eabi-gcc: No such file or directory
make: *** [_build/dfu_single_bank.o] Error 127
```

You make need to edit the `Makefile` (for example `src/singlebank/Makefile`),
and update the `GNU_INSTALL_ROOT` to point to the root path of your GCC ARM
toolchain.

#### 2. `mergehex: No such file or directory`

Make sure that `mergehex` is available from the command-line. This binary is
part of of Nordic's [nRF5x Command Line Tools](http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.tools%2Fdita%2Ftools%2Fnrf5x_command_line_tools%2Fnrf5x_installation.html).


## Option 2: Build using Segger Embeded Studio

For better debugging you can also use SES, The project file is located at `src/segger/nrf52840_bootloader.emProject`.  Note: the SES only flash the bootloader when you click download, you need to flash Softdevice beforehand if you haven't done so ( use `make flash_sd` mentioned above )
