# Adafruit nRF52 Bootloader

[![Build Status](https://github.com/adafruit/Adafruit_nRF52_Bootloader/workflows/Build/badge.svg)](https://github.com/adafruit/Adafruit_nRF52_Bootloader/actions)

A CDC/DFU/UF2 bootloader for Nordic nRF52 microcontroller. UF2 is an easy-to-use bootloader that appears as a flash drive. You can just copy `.uf2`-format application images to the flash drive to load new firmware. See https://github.com/Microsoft/uf2 for more information.

DFU via serial/CDC requires [adafruit-nrfutil](https://github.com/adafruit/Adafruit_nRF52_nrfutil), a modified version of [Nordic nrfutil](https://github.com/NordicSemiconductor/pc-nrfutil). Install `python3` if it is not installed already and run this command to install adafruit-nrfutil from PyPi:

```
$ pip3 install --user adafruit-nrfutil
```

## Supported Boards

Officially supported boards are:

- [Adafruit CLUE](https://www.adafruit.com/product/4500)
- [Adafruit Circuit Playground Bluefruit](https://www.adafruit.com/product/4333)
- [Adafruit Feather nRF52832](https://www.adafruit.com/product/3406)
- [Adafruit Feather nRF52840 Express](https://www.adafruit.com/product/4062)
- [Adafruit Feather nRF52840 Sense](https://www.adafruit.com/product/4516)
- [Adafruit ItsyBitsy nRF52840 Express](https://www.adafruit.com/product/4481)
- [Adafruit LED Glasses Driver nRF52840](https://www.adafruit.com/product/5217)
- Adafruit Metro nRF52840 Express
- [Raytac MDBT50Q-RX Dongle](https://www.adafruit.com/product/5199)

In addition, there is also lots of other 3rd-party boards which are added by other makers, users and community. Check out the [complete list of all boards here](/src/boards).

## Features

- DFU over Serial and OTA ( application, Bootloader+SD )
- Self-upgradable via Serial and OTA
- DFU using UF2 (https://github.com/Microsoft/uf2) (application only)
- Auto-enter DFU briefly on startup for DTR auto-reset trick (832 only)
- Supports dual bank firmware updates (disabled by default)
- Supports signed firmware updates (disabled by default)

## How to use

There are two pins, `DFU` and `FRST` that bootloader will check upon reset/power:

- `Double Reset` Reset twice within 500 ms will enter DFU with UF2 and CDC support (only works with nRF52840)
- `DFU = LOW` and `FRST = HIGH`: Enter bootloader with UF2 and CDC support
- `DFU = LOW` and `FRST = LOW`: Enter bootloader with OTA, to upgrade with a mobile application such as Nordic nrfConnect/Toolbox
- <s>`DFU = HIGH` and `FRST = LOW`: Factory Reset mode: erase firmware application and its data</s>
- `DFU = HIGH` and `FRST = HIGH`: Go to application code if it is present, otherwise enter DFU with UF2
- The `GPREGRET` register can also be set to force the bootloader can enter any of above modes (plus a CDC-only mode for Arduino).
`GPREGRET` is set by the application before performing a soft reset.

```c
#include "nrf_nvic.h"
void reset_to_uf2(void) {
  NRF_POWER->GPREGRET = 0x57; // 0xA8 OTA, 0x4e Serial
  NVIC_SystemReset();         // or sd_nvic_SystemReset();
}
```

On the Nordic PCA10056 DK board, `DFU` is connected to **Button1**, and `FRST` is connected to **Button2**.
So holding down **Button1** while clicking **RESET** will put the board into USB bootloader mode, with UF2 and CDC support.
Holding down **Button2** while clicking **RESET** will put the board into OTA (over-the-air) bootloader mode.

On the Nordic PCA10059 Dongle board, `DFU` is connected to the white button.
`FRST` is connected to pin 1.10. Ground it to pull `FRST` low, as if you had pushed an `FRST`  button.
There is an adjacent ground pad.

For other boards, please check the board definition for details.

### Making your own UF2

To create your own UF2 DFU update image, simply use the [Python conversion script](https://github.com/Microsoft/uf2/blob/master/utils/uf2conv.py) on a .bin file or .hex file, specifying the family as **0xADA52840** (nRF52840) or **0x621E937A** (nRF52833).

```
nRF52840
uf2conv.py firmware.hex -c -f 0xADA52840

nRF52833
uf2conv.py firmware.hex -c -f 0x621E937A
```

If using a .bin file with the conversion script you must specify application address with the -b switch, this address depend on the SoftDevice size/version e.g S140 v6 is 0x26000, v7 is 0x27000

```
nRF52840
uf2conv.py firmware.bin -c -b 0x26000 -f 0xADA52840

nRF52833
uf2conv.py firmware.bin -c -b 0x27000 -f 0x621E937A
```

To create a UF2 image for bootloader from a .hex file using separated family of **0xd663823c**

```
uf2conv.py bootloader.hex -c -f 0xd663823c
```

## Burn & Upgrade with pre-built binaries

You can burn and/or upgrade the bootloader with either a J-link or DFU (serial) to a specific pre-built binary version
without the hassle of installing a toolchain and compiling the code.
This is preferred if you are not developing/customizing the bootloader.
Pre-builtin binaries are available on GitHub [releases](https://github.com/adafruit/Adafruit_nRF52_Bootloader/releases)

Note: The bootloader can be downgraded. Since the binary release is a merged version of
both bootloader and the Nordic SoftDevice, you can freely upgrade/downgrade to any version you like.

## How to compile and build 

You should only continue if you are looking to develop bootloader for your own.
You must have have a J-Link available to "unbrick" your device.

### Prerequisites

- ARM GCC
- Nordic's [nRF5x Command Line Tools](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF-Command-Line-Tools)
- [Python IntelHex](https://pypi.org/project/IntelHex/)

### Build:

Firstly clone this repo including its submodules with following command:

```
git clone --recurse-submodules https://github.com/adafruit/Adafruit_nRF52_Bootloader
```

For git versions before `2.13.0` you have to do that manually:
```
git clone https://github.com/adafruit/Adafruit_nRF52_Bootloader
cd Adafruit_nRF52_Bootloader
git submodule update --init
```

#### Build using `make`
Then build it with `make BOARD={board} all`, for example:

```
make BOARD=feather_nrf52840_express all
```

For the list of supported boards, run `make` without `BOARD=` :

```
make
You must provide a BOARD parameter with 'BOARD='
Supported boards are: feather_nrf52840_express feather_nrf52840_express pca10056
Makefile:90: *** BOARD not defined.  Stop
```

#### Build using `cmake`

Firstly initialize your build environment by passing your board to `cmake` via `-DBOARD={board}`:

```bash
mkdir build
cd build
cmake .. -DBOARD=feather_nrf52840_express 
```

And then build it with:

```bash
make
```

You can also use the generator of your choice. For example adding the `-GNinja` and then you can invoke `ninja build` instead of `make`.

To list all supported targets, run:

```bash
cmake --build . --target help
``` 

To build individual targets, you can specify it directly with `make` or using `cmake --build`:

```bash
make bootloader
cmake --build . --target bootloader
```

### Flash

To flash the bootloader (without softdevice/mbr) using JLink:

```
make BOARD=feather_nrf52840_express flash
```

If you are using pyocd as debugger, add `FLASHER=pyocd` to make command:

```
make BOARD=feather_nrf52840_express FLASHER=pyocd flash
```

To upgrade the bootloader using DFU Serial via port /dev/ttyACM0

```
make BOARD=feather_nrf52840_express SERIAL=/dev/ttyACM0 flash-dfu
```

To flash SoftDevice (will also erase chip):

```
make BOARD=feather_nrf52840_express flash-sd
```

To flash MBR only

```
make BOARD=feather_nrf52840_express flash-mbr
```

### Dual bank firmware support:

This bootloader can split the FLASH memory into 2 partitions, one for the last successfully uploaded firmware, and the other for the new firmware that is being uploaded. In case the firmware upload fails, the bootloader will revert to the last working firmware version. The only drawback is that the maximum firmware size is half of the device FLASH size: That is why it is disabled by default.
You can enable this feature by passing DUALBANK_FW=1 to the make process while compiling the bootloader

### Signed firmware support:
This bootloader can validate that the uploaded firmware is digitally signed, and refuse to install unsigned or signed with the improper key firmware. Because this will make Arduino uploads stop working (because they are not digitally signed), this feature is disabled by default.
But if you want to ensure noone else is allowed to upload firmware to your device, this option is probably what you want to enable.

To create a signing key, you will need the adafruit-nrfutil utility. First, generate a signing key and store at given path by using

```
adafruit-nrfutil keys --gen-key stored_key.pem
```

This command will create a signing key and store it in the file stored_key.pem.  STORE THIS FILE IN A SAFE PLACE, as without it, you won't be able to sign your firmware packages, and the bootloader will refuse to flash them!!

Now, you must show the verification key (that is calculated from the signing key):

```
adafruit-nrfutil keys --show-vk code stored_key.pem
```

This command will read your signing key (from the file stored_key.pem) and display C source code with the Qx and Qy arrays: Something along the lines

```
static uint8_t Qx[] = { ... };
static uint8_t Qy[] = { ... };
```

You must save thw Qx and Qy values, remove all spaces, an pass them to the make command line
```
make BOARD=feather_nrf52840_express SIGNED_FW=1 SIGNED_FW_QX='Qx values' SIGNED_FW_QY='Qy values'
```

And replace Qx values with the Qx array values, and the Qy values with the Qy array values. That will create a bootloader that ONLY accepts firmware signed with the stored_key.pem signing key. Take into account that the option SIGNED_FW=1 is what enables to build a bootloader that enforces signing, and will ALSO result in disabling UF2 support, as UF2 format does NOT support signing keys. There is an option (only for testing!) called FORCE_UF2 that will FORCE UF2 support on bootloaders that enforce firmware security. It is there to bypass security so you can test a secure bootloader and have a way to recover it something goes wrong, but should NEVER be used if you want to enforce security

Finally, to create a signed firmware application/bootloader update package, you can use, for example

```
adafruit-nrfutil dfu genpkg --dev-type 0x0052 --sd-req 0x0123 --application "application.hex" --key-file "stored_key.pem" "application_package.zip"
```

Please, use full paths for all files. The examples omit them for clarity!

And, to upload it to your device, 

```
adafruit-nrfutil.exe --verbose dfu serial -pkg "application_package.zip" -p /dev/tty0 -b 115200
```

### Common makefile problems

#### `arm-none-eabi-gcc`: No such file or directory

If you get the following error ...

```
make BOARD=feather_nrf52840_express all
Compiling file: main.c
/bin/sh: /usr/bin/arm-none-eabi-gcc: No such file or directory
make: *** [_build/main.o] Error 127
```

... you may need to pass the location of the GCC ARM toolchain binaries to `make` using
the variable `CROSS_COMPILE` as below:
```
make CROSS_COMPILE=/opt/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi- BOARD=feather_nrf52832 all
```

For other compile errors, check the gcc version with `arm-none-eabi-gcc --version` to insure it is at least 9.x.

#### `ModuleNotFoundError: No module named 'intelhex'`

Install python-intelhex with

```
pip install intelhex
```

#### `make: nrfjprog: No such file or directory`

Make sure that `nrfjprog` is available from the command-line. This binary is
part of Nordic's nRF5x Command Line Tools.
