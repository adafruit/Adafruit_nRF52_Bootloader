# A board's SoftDevice and the MBR: board.cmake's settings over the MCU variant's defaults. Included after board.cmake
# by CMakeLists.txt and by board_vars.cmake.
get_filename_component(SOFTDEVICE_DIR ${CMAKE_CURRENT_LIST_DIR}/../lib/softdevice ABSOLUTE)
set(MBR_HEX ${SOFTDEVICE_DIR}/mbr/hex/mbr_nrf52_2.4.1_mbr.hex)

# SD_VERSION can be overwritten by board.cmake
if (NOT DEFINED SD_VERSION)
  if (MCU_VARIANT STREQUAL "nrf52833")
    set(SD_VERSION 7.3.0)
  else ()
    set(SD_VERSION 6.1.1)
  endif ()
endif ()

# SD_NAME can be overwritten by board.cmake or -D, as make's command line overrode it
if (NOT DEFINED SD_NAME)
  if (MCU_VARIANT STREQUAL "nrf52")
    set(SD_NAME s132)
  else ()
    set(SD_NAME s140)
  endif ()
endif ()

set(SD_FILENAME ${SD_NAME}_nrf52_${SD_VERSION})
if (DEFINED SD_HEX)
  # relative to the checkout, as make resolved it: the merge and DFU package commands run in the build directory
  get_filename_component(SD_HEX "${SD_HEX}" ABSOLUTE BASE_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
else ()
  set(SD_HEX ${SOFTDEVICE_DIR}/${SD_FILENAME}/${SD_FILENAME}_softdevice.hex)
endif ()
