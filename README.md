# Adafruit Bluefruit nRF52 Bootloader

This repository contains the bootloader for Adafruit nRF52 series board

- [Bluefruit Feather nRF52832](https://www.adafruit.com/product/3406)
- Bluefruit Feather nRF52840

## Features

### nRF52832

- DFU over Serial and OTA ( Application, Bootloader+SD )
- DFU auto start to work with Arduino DTR auto-reset

### nRF52840

- DFU over USB CDC and OTA ( Application, Bootloader+SD )
- DFU using USB Flashing Format a.k.a [UF2](https://github.com/Microsoft/uf2) (Applicatoin only)


## Compile

This repository depend on following submodule

- [tinyusb](https://github.com/hathach/tinyusb/tree/develop)
- [nrfx](https://github.com/NordicSemiconductor/nrfx)

Get the code with following command:

	git clone <URL>.git Adafruit_nRF52_Bootloader
    cd Adafruit_nRF52_Bootloader
    git submodule update --init

### Option 1: Build with makefile

Prerequisite

- ARM GCC
- Nordic's [nRF5x Command Line Tools](http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.tools%2Fdita%2Ftools%2Fnrf5x_command_line_tools%2Fnrf5x_installation.html)

To build

	$ make BOARD=feather52840 all

To flash bootloader

	$ make BOARD=feather52840 flash

To flash SoftDevice (with full chip erase)

	$ make BOARD=feather52840 sd

For the list of supported boards, try to type make without `BOARD=`

	$ make
	You must provide a BOARD parameter with 'BOARD='
	Supported boards are: feather52832 feather52840 pca10056
	Makefile:90: *** BOARD not defined.  Stop

### Common makefile problems

#### 1. `arm-none-eabi-gcc`: No such file or directory

If you get the following error ...

    $ make BOARD=feather52840 all
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

## Option 2: Build using Segger Embeded Studio

For better debugging you can also use [SES](https://www.segger.com/products/development-tools/embedded-studio/).
The project file is located at `src/segger/nrf52840_bootloader.emProject`.

> **Note**: SES only flashes the bootloader when you click download, you need to
flash Softdevice beforehand if you haven't done so ( use `make flash_sd`
mentioned above )
