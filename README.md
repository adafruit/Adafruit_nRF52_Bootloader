# Adafruit Bluefruit nRF52 Bootloader

This repository contains the bootloader for Adafruit nRF52 series and other popular boards

- [Bluefruit Feather nRF52832](https://www.adafruit.com/product/3406)
- Bluefruit Feather nRF52840 Express
- Nordic nRF52840DK PCA10056

[adafruit-nrfutil](https://github.com/adafruit/Adafruit_nRF52_nrfutil), modified version of [Nordic nrfutil](https://github.com/NordicSemiconductor/pc-nrfutil), is required to perform DFU. Install python3 to your system if it is not installed already and run this command to install adafruit-nrfutil from PyPi:

	$ pip3 install --user adafruit-nrfutil

This repository depends on following submodule

- [tinyusb](https://github.com/hathach/tinyusb/tree/develop)
- [nrfx](https://github.com/NordicSemiconductor/nrfx)

Get the code with following command:

	git clone <URL>.git Adafruit_nRF52_Bootloader
    cd Adafruit_nRF52_Bootloader
    git submodule update --init

## Features

- DFU over Serial and OTA ( Application, Bootloader+SD )
- Self-upgradble via Serial and OTA
- DFU using USB Flashing Format a.k.a [UF2](https://github.com/Microsoft/uf2) (Applicatoin only)
- Auto enter DFU briefly on startup for DTR auto-reset trick (832 only)

## How to use

There are 2 pins **DFU & FRST** that bootloader will check upon reset/power

- `Double Reset` Reset twice within 500 ms will enter DFU with UF2 and CDC support (only works with nRF52840)
- `DFU = Low and FRST = High` Enter DFU with UF2 and CDC support
- `DFU = Low and FRST = Low` Enter DFU with OTA, can be upgraded with mobile application such as Nordic nrfConnect/Toolbox
- `DFU = High and FRST = Low` Factory Reset mode, erase firmware application and its data
- `DFU = High and FRST = High` Goto application code if it is present, otherwise enter DFU with UF2
- Depending on value of the `GPREGRET` register, Bootloader can enter any of above mode (plus a CDC only mode for Arduino). `GPREGRET` is set by application before performing a softreset.

For the exact pin location, please check your board definition for details

## Burn & Upgrade with pre-built binaries

You can burn and/or upgrade bootloader with either jlink or dfu (serial) to a specific pre-built binary version without the hassle to install toolchain and compile the code. This is preferred if you are not developing/customizing the bootloader

To flash version `6.1.0r0` using jlink

	$ make BOARD=feather_nrf52840_express VERSION=6.1.0r0 flash

To upgrade with dfu serial

	$ make BOARD=feather_nrf52840_express VERSION=6.1.0r0 dfu-flash

You can run following command for the list of pre-builtin binaries

	$ ls bin/*
	bin/feather_nrf52832:
	2.0.1  5.0.0  6.1.0r0

	bin/feather_nrf52840_express:
	6.1.0r0

	bin/pca10056:
	6.1.0r0


Note: bootloader is downgradable, since the binary release is merged of bootloader and Softdevice, you could freely "upgrade" to any version you like.

## Compile with latest codes

You should only continue if you are looking to develop bootloader for your own. Having a jlink to **de-brick** your device is a must.

### Option 1: Build with makefile

Prerequisite

- ARM GCC
- Nordic's [nRF5x Command Line Tools](http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.tools%2Fdita%2Ftools%2Fnrf5x_command_line_tools%2Fnrf5x_installation.html)

To build

	$ make BOARD=feather_nrf52840_express all

To flash bootloader

	$ make BOARD=feather_nrf52840_express flash

To flash SoftDevice (chip erase)

	$ make BOARD=feather_nrf52840_express sd

To erase chip

	$ make BOARD=feather_nrf52840_express erase

For the list of supported boards, try to type make without `BOARD=`

	$ make
	You must provide a BOARD parameter with 'BOARD='
	Supported boards are: feather_nrf52840_express feather_nrf52840_express pca10056
	Makefile:90: *** BOARD not defined.  Stop

### Common makefile problems

#### 1. `arm-none-eabi-gcc`: No such file or directory

If you get the following error ...

    $ make BOARD=feather_nrf52840_express all
    Compiling file: main.c
    /bin/sh: /usr/bin/arm-none-eabi-gcc: No such file or directory
    make: *** [_build/main.o] Error 127

... you may need to edit the `Makefile` and update the `GNU_INSTALL_ROOT` to point to the root path of your GCC ARM toolchain.

#### 2. `mergehex: No such file or directory`

Make sure that `mergehex` is available from the command-line. This binary is
part of of Nordic's nRF5x Command Line Tools

#### 3. `make: nrfjprog: No such file or directory`

Make sure that `nrfjprog` is available from the command-line. This binary is
part of of Nordic's nRF5x Command Line Tools.

On POSIX-type systems you can temporarily add the path to `nrfjprog` via a
variation on the following command:

```
$ export PATH=$PATH:/Users/Kevin/Downloads/nRF5x-Command-Line-Tools_9_7_2_OSX/nrfjprog
```

### Option 2: Build using Segger Embeded Studio

For better debugging you can also use [SES](https://www.segger.com/products/development-tools/embedded-studio/).
The project file is located at `src/segger/Adafruit_nRF52_Bootloader.emProject`.

> **Note**: SES only flashes the bootloader when you click download, you need to
flash Softdevice beforehand if you haven't done so ( use `make BOARD=feather_nrf52840_express sd`
mentioned above )
