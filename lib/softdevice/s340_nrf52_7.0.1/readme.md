## Working with SoftDevice S340

The SoftDevice S340 is closed-source, not publicly available and is only distributed by Garmin Canada Inc.

In order to be able to download the required ANT+ capable SoftDevice and place SoftDevice S340 v7.0.1 files here. You need to register an 'ANT+ Adopter' account at [thisisant.com](https://www.thisisant.com/register/). After around one business day you will receive access to the resources there. Then do the following steps:
- Download the SoftDevice S340 v7.0.1 [(here)](https://www.thisisant.com/developer/components/nrf52832#tab_protocol_stacks_tab) and extract its contents
- Under `lib/softdevice` in this repository there is a folder called `s340_nrf52_7.0.1`
    - Copy the API folder `ANT_s340_nrf52_7.0.1.API`, the license agreement `License_Agreement_ANT_Softdevice_rev3_3.pdf` and the hex file `ANT_s340_nrf52_7.0.1.hex` from the extracted contents to it.
    - Rename the API folder to `s340_nrf52_7.0.1_API`
    - Rename the hex file to `s340_nrf52_7.0.1_softdevice.hex`
    - Modify `lib/softdevice/s340_nrf52_7.0.1_API/include/nrf_sdm.h` on line 191 and remove the two slashes at the beginning of `//#define...` to use the *evaluation key* for the ANT SoftDevice.
    - **VERY IMPORTANT:** You MUST obtain a valid commercial license key BEFORE releasing a product to market that uses the ANT SoftDevice!

To add or modify a board with an ANT+ capable SoftDevice S340 the `SD_VERSION` and `SD_NAME` parameters in the corresponding `board.mk` file have to be set:
```
SD_VERSION = 7.0.1
SD_NAME = s340
```
**Important:** When adding a new board you must add the suffix `_s340` to the folder name to exclude it from automatic builds.