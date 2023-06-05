# PR description

Pull request contains three segger studio projects for dual-core nrf5340_DK board:
- mbr project that builds mbr file which is a replacement for original nordic sdk mbr.hex for other microcontrollers
- application core bootloader project,
- networking core bootloader project.
Bootloader settings and configuration sections are currently deactivated in this pull request.

Application bootloader is almost the same as for other previous micros, it implements additional mechanizm which
basing on uf2 block address decides whether its destination is application core flash or networking core flash.
In case it is networking core flash it copies data into allocated RAM area (shared with networking core) and sends request
to networking core to write it under proper address. When all blocks are flashed application core sends request to 
networking core to start execution of newely received firmware.


Building and flashing sequence:
- connect USB cable to nrf USB on nrf5340 board and to PC
- build all segger projects: MBR project, application core bootloader project, networking core bootloader project
- erase FLASH memory on both cores:
	  nrfjprog --recover --coprocessor CP_NETWORK; nrfjprog --recover
- flash networking core bootloader (F5 and disconnect)
- flash MBR (F5 and disconnect)
- flash application core bootloader (F5 and disconnect)
- press and hold button 1 on the board
- press reset button

Mass storage device should show up on PC.
All bootloaders should be now ready for receiving firmware (diodes LED 1 and 2 should go on).
Drag and drop binaries, example: blinky_nrf5340_app_zephyr_0x1000_2Hz.uf2  (located in uf2binaries folder)

- press and hold button 1 on the board again
- press reset button

Bootloaders should be now ready again for receiving new firmware.
Drag and drop binaries, example: blinky_nrf5340_net_zephyr_0x01004000_05Hz.uf2  

Both cores should start executing flashed firmware, diodes LED3 and LED 4 should be blinking.
