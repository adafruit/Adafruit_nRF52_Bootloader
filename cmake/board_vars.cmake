# Writes what the Makefile wrapper's erase, flash-sd and flash-mbr need to OUT, without configuring the build (no
# compiler or submodule needed):
#   cmake -DBOARD=<board> [-DSD_NAME=..] [-DSD_VERSION=..] [-DSD_HEX=..] -DOUT=<file> -P cmake/board_vars.cmake
include(${CMAKE_CURRENT_LIST_DIR}/../src/boards/${BOARD}/board.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/softdevice.cmake)
file(WRITE ${OUT}
  "MCU_VARIANT = ${MCU_VARIANT}\n"
  "SD_HEX = ${SD_HEX}\n"
  "MBR_HEX = ${MBR_HEX}\n"
  )
