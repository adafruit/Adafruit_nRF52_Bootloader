cmake_minimum_required(VERSION 3.17)
find_package(Python COMPONENTS Interpreter)

if (NOT DEFINED BOARD)
  message(FATAL_ERROR "BOARD is not defined")
endif ()

if (NOT DEFINED TOOLCHAIN)
  set(TOOLCHAIN gcc)
endif ()

# include board specific
include(${CMAKE_CURRENT_LIST_DIR}/src/boards/${BOARD}/board.cmake)

# toolchain set up
if (MCU_VARIANT STREQUAL "nrf5340_application")
  set(CMAKE_SYSTEM_PROCESSOR cortex-m33 CACHE INTERNAL "System Processor")
  set(JLINK_DEVICE nrf5340_xxaa_app)
else ()
  set(CMAKE_SYSTEM_PROCESSOR cortex-m4 CACHE INTERNAL "System Processor")
  set(JLINK_DEVICE ${MCU_VARIANT}_xxaa)
endif ()

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/cmake/toolchain/arm_${TOOLCHAIN}.cmake)

project(Adafruit_nRF52_Bootloader C ASM)

set(NRFX ${CMAKE_CURRENT_LIST_DIR}/lib/nrfx)
set(SDK11 ${CMAKE_CURRENT_LIST_DIR}/lib/sdk11/components)
set(SDK ${CMAKE_CURRENT_LIST_DIR}/lib/sdk/components)
set(SOFTDEVICE ${CMAKE_CURRENT_LIST_DIR}/lib/softdevice)
set(TUSB ${CMAKE_CURRENT_LIST_DIR}/lib/tinyusb/src)

set(UF2CONV_PY ${CMAKE_CURRENT_LIST_DIR}/lib/uf2/utils/uf2conv.py)
set(UF2_FAMILY_ID_BOOTLOADER 0xd663823c)

#-------------------
# Bootloader
#-------------------
set(CMAKE_EXECUTABLE_SUFFIX .elf)
add_executable(bootloader)

# SD_VERSION can be overwritten by board.cmake
if(NOT DEFINED SD_VERSION)
  set(SD_VERSION 6.1.1)
endif ()

set(MBR_HEX ${SOFTDEVICE}/mbr/hex/mbr_nrf52_2.4.1_mbr.hex)

target_sources(bootloader PUBLIC
  # src
  src/dfu_ble_svc.c
  src/dfu_init.c
  src/flash_nrf5x.c
  src/main.c
  src/screen.c
  src/images.c
  src/boards/boards.c
  # nrfx
  ${NRFX}/drivers/src/nrfx_power.c
  ${NRFX}/drivers/src/nrfx_nvmc.c
  ${NRFX}/mdk/system_${MCU_VARIANT}.c
  # sdk 11
  ${SDK11}/libraries/bootloader_dfu/bootloader.c
  ${SDK11}/libraries/bootloader_dfu/bootloader_settings.c
  ${SDK11}/libraries/bootloader_dfu/bootloader_util.c
  ${SDK11}/libraries/bootloader_dfu/dfu_transport_serial.c
  ${SDK11}/libraries/bootloader_dfu/dfu_transport_ble.c
  ${SDK11}/libraries/bootloader_dfu/dfu_single_bank.c
  ${SDK11}/ble/ble_services/ble_dfu/ble_dfu.c
  ${SDK11}/ble/ble_services/ble_dis/ble_dis.c
  ${SDK11}/drivers_nrf/pstorage/pstorage_raw.c
  # latest sdk
  ${SDK}/libraries/timer/app_timer.c
  ${SDK}/libraries/scheduler/app_scheduler.c
  ${SDK}/libraries/util/app_error.c
  ${SDK}/libraries/util/app_util_platform.c
  ${SDK}/libraries/crc16/crc16.c
  ${SDK}/libraries/hci/hci_mem_pool.c
  ${SDK}/libraries/hci/hci_slip.c
  ${SDK}/libraries/hci/hci_transport.c
  ${SDK}/libraries/util/nrf_assert.c
  # ASM
  ${NRFX}/mdk/gcc_startup_${MCU_VARIANT}.S
  )
target_include_directories(bootloader PUBLIC
  src
  src/boards
  src/boards/${BOARD}
  src/cmsis/include
  src/usb
  ${TUSB}
  # nrfx
  ${NRFX}
  ${NRFX}/mdk
  ${NRFX}/hal
  ${NRFX}/drivers/include
  ${NRFX}/drivers/src
  # sdk 11 for cdc/ble dfu
  ${SDK11}/libraries/bootloader_dfu
  ${SDK11}/libraries/bootloader_dfu/hci_transport
  ${SDK11}/drivers_nrf/pstorage
  ${SDK11}/ble/common
  ${SDK11}/ble/ble_services/ble_dfu
  ${SDK11}/ble/ble_services/ble_dis
  # later sdk with updated drivers
  ${SDK}/libraries/timer
  ${SDK}/libraries/scheduler
  ${SDK}/libraries/crc16
  ${SDK}/libraries/util
  ${SDK}/libraries/hci/config
  ${SDK}/libraries/hci
  ${SDK}/libraries/uart
  ${SDK}/drivers_nrf/delay
  # Softdevice
  ${SOFTDEVICE}/mbr/headers
  )

# Debug option
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  # TODO not work yet, also need to add segger rtt, DFU_APP_DATA_RESERVED=0, BOOTLOADER_REGION_START=0xED000
  set(LD_FILE ${CMAKE_CURRENT_LIST_DIR}/linker/${MCU_VARIANT}_debug.ld)

  target_sources(bootloader PUBLIC
    lib/SEGGER_RTT/RTT/SEGGER_RTT.c
    )
  target_include_directories(bootloader PUBLIC
    lib/SEGGER_RTT/RTT
    )

  target_compile_definitions(bootloader PUBLIC
    CFG_DEBUG
    SEGGER_RTT_MODE_DEFAULT=SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL
    DFU_APP_DATA_RESERVED=0
    )

  if (MCU_VARIANT STREQUAL "nrf52840")
    target_compile_definitions(bootloader PUBLIC BOOTLOADER_REGION_START=0xEA000)
  else ()
    target_compile_definitions(bootloader PUBLIC BOOTLOADER_REGION_START=0x6D000)
  endif ()
else ()
  set(LD_FILE ${CMAKE_CURRENT_LIST_DIR}/linker/${MCU_VARIANT}.ld)
endif ()

target_link_options(bootloader PUBLIC
  "LINKER:--script=${LD_FILE}"
  -L${NRFX}/mdk
  --specs=nosys.specs --specs=nano.specs
  )
target_compile_options(bootloader PUBLIC
  -fno-builtin
  -fshort-enums
  -fstack-usage
  -fno-strict-aliasing
  -Wall
  -Wextra
  -Werror
  -Wfatal-errors
  -Werror-implicit-function-declaration
  -Wfloat-equal
  -Wundef
  -Wshadow
  -Wwrite-strings
  -Wsign-compare
  -Wmissing-format-attribute
  -Wno-endif-labels
  -Wunreachable-code
  # Suppress warning caused by SDK
  -Wno-unused-parameter -Wno-expansion-to-defined -Wno-array-bounds
  )
target_compile_definitions(bootloader PUBLIC
  SOFTDEVICE_PRESENT
  )

if (TRACE_ETM STREQUAL "1")
  # ENABLE_TRACE will cause system_nrf5x.c to set up ETM trace
  target_compile_definitions(bootloader PUBLIC ENABLE_TRACE)
endif ()

if (MCU_VARIANT STREQUAL "nrf52")
  # UART transport
  target_sources(bootloader PUBLIC
    ${SDK}/libraries/uart/app_uart.c
    ${SDK}/drivers_nrf/uart/nrf_drv_uart.c
    ${SDK}/drivers_nrf/common/nrf_drv_common.c
    )
  target_include_directories(bootloader PUBLIC
    ${SDK11}/libraries/util
    ${SDK}/drivers_nrf/common
    ${SDK}/drivers_nrf/uart
    )
else ()
  # USB transport
  target_sources(bootloader PUBLIC
    src/boards/${BOARD}/pinconfig.c
    src/usb/msc_uf2.c
    src/usb/usb.c
    src/usb/usb_desc.c
    src/usb/uf2/ghostfat.c
    # TinyUSB
    ${TUSB}/portable/nordic/nrf5x/dcd_nrf5x.c
    ${TUSB}/common/tusb_fifo.c
    ${TUSB}/device/usbd.c
    ${TUSB}/device/usbd_control.c
    ${TUSB}/class/cdc/cdc_device.c
    ${TUSB}/class/msc/msc_device.c
    ${TUSB}/tusb.c
    )
endif ()

#----------------------------------------------------
# MCU Variant differences
# Supported are: nrf52 (nrf52832), nrf52833, nrf52840
#----------------------------------------------------
if (MCU_VARIANT STREQUAL "nrf52")
  set(SD_NAME s132)
  set(DFU_DEV_REV 0xADAF)
  set(DFU_APP_DATA_RESERVED 7*4096)
  target_compile_definitions(bootloader PUBLIC
    NRF52
    NRF52832_XXAA
    S132
    )
  target_include_directories(bootloader PUBLIC
    ${SOFTDEVICE}/s132_nrf52_6.1.1/s132_nrf52_6.1.1_API/include
    )

elseif (MCU_VARIANT STREQUAL "nrf52833")
  set(SD_NAME s140)
  set(DFU_DEV_REV 52833)
  set(DFU_APP_DATA_RESERVED 7*4096)
  target_compile_definitions(bootloader PUBLIC
    NRF52833_XXAA
    S140
    )
  target_include_directories(bootloader PUBLIC
    ${SOFTDEVICE}/s140_nrf52_6.1.1/s140_nrf52_6.1.1_API/include
    )

elseif (MCU_VARIANT STREQUAL "nrf52840")
  set(SD_NAME s140)
  set(DFU_DEV_REV 52840)
  # App reserved 40KB (8+32) to match circuitpython for 840
  set(DFU_APP_DATA_RESERVED 10*4096)
  target_compile_definitions(bootloader PUBLIC
    NRF52840_XXAA
    S140
    )
  target_include_directories(bootloader PUBLIC
    ${SOFTDEVICE}/s140_nrf52_6.1.1/s140_nrf52_6.1.1_API/include
    )

else ()
  message(FATAL_ERROR "MCU_VARIANT ${MCU_VARIANT} is unknown")
endif ()

set(SD_FILENAME ${SD_NAME}_nrf52_${SD_VERSION})
set(SD_HEX ${SOFTDEVICE}/${SD_FILENAME}/${SD_FILENAME}_softdevice.hex)

if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_definitions(bootloader PUBLIC DFU_APP_DATA_RESERVED=${DFU_APP_DATA_RESERVED})
endif ()

#----------------------------------
# Get UF2 version from git
#----------------------------------
execute_process(COMMAND git describe --dirty --always --tags OUTPUT_VARIABLE GIT_VERSION)
string(STRIP ${GIT_VERSION} GIT_VERSION)

execute_process(COMMAND bash "-c" "git submodule status | cut -d\" \" -f3,4 | paste -s -d\" \" -"
  OUTPUT_VARIABLE GIT_SUBMODULE_VERSIONS
  )
string(STRIP ${GIT_SUBMODULE_VERSIONS} GIT_SUBMODULE_VERSIONS)
string(REPLACE ../ "" GIT_SUBMODULE_VERSIONS ${GIT_SUBMODULE_VERSIONS})
string(REPLACE lib/ "" GIT_SUBMODULE_VERSIONS ${GIT_SUBMODULE_VERSIONS})

string(REPLACE "-" ";" RELEASE_VERSION ${GIT_VERSION})
list(GET RELEASE_VERSION 0 RELEASE_VERSION)
string(REPLACE "." ";" RELEASE_VERSION ${RELEASE_VERSION})
list(GET RELEASE_VERSION 0 RELEASE_VERSION_MAJOR)
list(GET RELEASE_VERSION 1 RELEASE_VERSION_MINOR)
list(GET RELEASE_VERSION 2 RELEASE_VERSION_PATCH)
math(EXPR MK_BOOTLOADER_VERSION "(${RELEASE_VERSION_MAJOR} << 16) + (${RELEASE_VERSION_MINOR} << 8) + ${RELEASE_VERSION_PATCH}")

cmake_print_variables(GIT_VERSION GIT_SUBMODULE_VERSIONS MK_BOOTLOADER_VERSION)

target_compile_definitions(bootloader PUBLIC
  UF2_VERSION_BASE="${GIT_VERSION}"
  UF2_VERSION="${GIT_VERSION} - ${GIT_SUBMODULE_VERSIONS}"
  BLEDIS_FW_VERSION="${GIT_VERSION} ${SD_NAME} ${SD_VERSION}"
  MK_BOOTLOADER_VERSION=${MK_BOOTLOADER_VERSION}
  )

#----------------------------------
# Post build
#----------------------------------

# run size after build
add_custom_command(TARGET bootloader POST_BUILD
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:bootloader>
  )

# Add bin/hex output
add_custom_command(TARGET bootloader POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:bootloader> $<TARGET_FILE_DIR:bootloader>/bootloader.bin
  COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:bootloader> $<TARGET_FILE_DIR:bootloader>/bootloader.hex
  COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/tools/hexmerge.py --overlap=replace -o $<TARGET_FILE_DIR:bootloader>/bootloader_mbr.hex $<TARGET_FILE_DIR:bootloader>/bootloader.hex ${MBR_HEX}
  COMMAND ${Python_EXECUTABLE} ${UF2CONV_PY} -f ${UF2_FAMILY_ID_BOOTLOADER} -c -o $<TARGET_FILE_DIR:bootloader>/bootloader_mbr.uf2 $<TARGET_FILE_DIR:bootloader>/bootloader_mbr.hex
  VERBATIM)

#----------------------------------
# Flashing target
#----------------------------------

if (NOT DEFINED NRFJPROG)
  set(NRFJPROG nrfjprog)
endif()

add_custom_target(flash-jlink
  DEPENDS bootloader
  COMMAND ${NRFJPROG} --program $<TARGET_FILE:bootloader> --verify --sectoranduicrerase -f nrf52 --reset
  )

add_custom_target(flash-uf2
  DEPENDS bootloader
  COMMAND ${Python_EXECUTABLE} ${UF2CONV_PY} -f ${UF2_FAMILY_ID_BOOTLOADER} --deploy $<TARGET_FILE_DIR:bootloader>/bootloader_mbr.uf2
  )

add_custom_target(flash-sd
  COMMAND ${NRFJPROG} --program ${SD_HEX} --verify --sectorerase -f nrf52 --reset
  )

add_custom_target(flash-mbr
  COMMAND ${NRFJPROG} --program ${MBR_HEX} --verify --sectorerase -f nrf52 --reset
  )

add_custom_target(flash-erase
  COMMAND ${NRFJPROG} -f nrf52 --eraseall
  )