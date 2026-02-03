cmake_minimum_required(VERSION 3.17)
include(CMakePrintHelpers)
find_package(Python COMPONENTS Interpreter)

if (NOT DEFINED BOARD)
  message(FATAL_ERROR "BOARD is not defined")
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

include(${CMAKE_CURRENT_LIST_DIR}/cmake/toolchain/arm_gcc.cmake)

set(NRFX_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/nrfx)
set(SDK11_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/sdk11/components)
set(SDK_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/sdk/components)
set(SOFTDEVICE_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/softdevice)
set(MBR_HEX ${SOFTDEVICE_DIR}/mbr/hex/mbr_nrf52_2.4.1_mbr.hex)
set(TINYUSB_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/tinyusb/src)
set(TCRYPT_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/tinycrypt/lib)

option(DEFAULT_TO_OTA_DFU "Default to OTA DFU instead of Serial DFU" OFF)
option(SIGNED_FW "Enable signed firmware verification" OFF)
option(DUALBANK_FW "Enable dual bank DFU support" OFF)
option(FORCE_UF2 "Force UF2 support even with SIGNED_FW" OFF)
set(SIGNED_FW_QX "" CACHE STRING "Public key Qx (comma-separated bytes)")
set(SIGNED_FW_QY "" CACHE STRING "Public key Qy (comma-separated bytes)")

set(UF2CONV_PY ${CMAKE_CURRENT_LIST_DIR}/lib/uf2/utils/uf2conv.py)
set(UF2_FAMILY_ID_BOOTLOADER 0xd663823c)

#-------------------
# Bootloader
#-------------------
if (NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE MinSizeRel CACHE STRING "Build type" FORCE)
endif ()

project(Adafruit_nRF52_Bootloader C ASM)
set(CMAKE_EXECUTABLE_SUFFIX .elf)
add_executable(bootloader
  # src
  src/dfu_ble_svc.c
  src/dfu_init.c
  src/flash_nrf5x.c
  src/main.c
  src/screen.c
  src/images.c
  src/boards/boards.c
  # nrfx
  ${NRFX_DIR}/drivers/src/nrfx_power.c
  ${NRFX_DIR}/drivers/src/nrfx_nvmc.c
  ${NRFX_DIR}/mdk/system_${MCU_VARIANT}.c
  # sdk 11
  ${SDK11_DIR}/libraries/bootloader_dfu/bootloader.c
  ${SDK11_DIR}/libraries/bootloader_dfu/bootloader_settings.c
  ${SDK11_DIR}/libraries/bootloader_dfu/bootloader_util.c
  ${SDK11_DIR}/libraries/bootloader_dfu/dfu_transport_serial.c
  ${SDK11_DIR}/libraries/bootloader_dfu/dfu_transport_ble.c
  ${SDK11_DIR}/ble/ble_services/ble_dfu/ble_dfu.c
  ${SDK11_DIR}/ble/ble_services/ble_dis/ble_dis.c
  ${SDK11_DIR}/drivers_nrf/pstorage/pstorage_raw.c
  # latest sdk
  ${SDK_DIR}/libraries/timer/app_timer.c
  ${SDK_DIR}/libraries/scheduler/app_scheduler.c
  ${SDK_DIR}/libraries/util/app_error.c
  ${SDK_DIR}/libraries/util/app_util_platform.c
  ${SDK_DIR}/libraries/crc16/crc16.c
  ${SDK_DIR}/libraries/hci/hci_mem_pool.c
  ${SDK_DIR}/libraries/hci/hci_slip.c
  ${SDK_DIR}/libraries/hci/hci_transport.c
  ${SDK_DIR}/libraries/util/nrf_assert.c
  # startup
  ${NRFX_DIR}/mdk/gcc_startup_${MCU_VARIANT}.S
  )

if (DUALBANK_FW)
  target_sources(bootloader PUBLIC
    ${SDK11_DIR}/libraries/bootloader_dfu/dfu_dual_bank.c
  )
else ()
  target_sources(bootloader PUBLIC
    ${SDK11_DIR}/libraries/bootloader_dfu/dfu_single_bank.c
  )
endif ()

if (SIGNED_FW)
  target_sources(bootloader PUBLIC
    ${TCRYPT_DIR}/source/sha256.c
    ${TCRYPT_DIR}/source/ecc.c
    ${TCRYPT_DIR}/source/ecc_dsa.c
    ${TCRYPT_DIR}/source/utils.c
  )

  target_include_directories(bootloader PUBLIC
    ${TCRYPT_DIR}/include
  )
endif ()

target_include_directories(bootloader PUBLIC
  src
  src/boards
  src/boards/${BOARD}
  src/cmsis/include
  src/usb
  ${TINYUSB_DIR}
  # nrfx
  ${NRFX_DIR}
  ${NRFX_DIR}/mdk
  ${NRFX_DIR}/hal
  ${NRFX_DIR}/drivers/include
  ${NRFX_DIR}/drivers/src
  # sdk 11 for cdc/ble dfu
  ${SDK11_DIR}/libraries/bootloader_dfu
  ${SDK11_DIR}/libraries/bootloader_dfu/hci_transport
  ${SDK11_DIR}/drivers_nrf/pstorage
  ${SDK11_DIR}/ble/common
  ${SDK11_DIR}/ble/ble_services/ble_dfu
  ${SDK11_DIR}/ble/ble_services/ble_dis
  # later sdk with updated drivers
  ${SDK_DIR}/libraries/timer
  ${SDK_DIR}/libraries/scheduler
  ${SDK_DIR}/libraries/crc16
  ${SDK_DIR}/libraries/util
  ${SDK_DIR}/libraries/hci/config
  ${SDK_DIR}/libraries/hci
  ${SDK_DIR}/libraries/uart
  ${SDK_DIR}/drivers_nrf/delay
  # Softdevice
  ${SOFTDEVICE_DIR}/mbr/headers
  )

# Debug linker file
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(LD_FILE ${CMAKE_CURRENT_LIST_DIR}/linker/${MCU_VARIANT}_debug.ld)
else ()
  set(LD_FILE ${CMAKE_CURRENT_LIST_DIR}/linker/${MCU_VARIANT}.ld)
endif ()

target_link_options(bootloader PUBLIC
  "LINKER:--script=${LD_FILE}"
  -L${NRFX_DIR}/mdk
  -nostartfiles
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
  CONFIG_GPIO_AS_PINRESET
  __START=main
  __STARTUP_CLEAR_BSS
  )

if (SIGNED_FW)
  if (SIGNED_FW_QX STREQUAL "" OR SIGNED_FW_QY STREQUAL "")
    message(FATAL_ERROR "SIGNED_FW requires SIGNED_FW_QX and SIGNED_FW_QY to be set")
  endif ()
  string(REPLACE " " "" SIGNED_FW_QX_STR "${SIGNED_FW_QX}")
  string(REPLACE " " "" SIGNED_FW_QY_STR "${SIGNED_FW_QY}")
  target_compile_definitions(bootloader PUBLIC
    SIGNED_FW
    "SIGNED_FW_QX=${SIGNED_FW_QX_STR}"
    "SIGNED_FW_QY=${SIGNED_FW_QY_STR}"
  )
endif ()

if (FORCE_UF2)
  target_compile_definitions(bootloader PUBLIC FORCE_UF2)
endif ()

if (DEFAULT_TO_OTA_DFU)
  target_compile_definitions(bootloader PUBLIC DEFAULT_TO_OTA_DFU)
endif ()

if (TRACE_ETM STREQUAL "1")
  # ENABLE_TRACE will cause system_nrf5x.c to set up ETM trace
  target_compile_definitions(bootloader PUBLIC ENABLE_TRACE)
endif ()

if (MCU_VARIANT STREQUAL "nrf52")
  # UART transport
  target_sources(bootloader PUBLIC
    ${SDK_DIR}/libraries/uart/app_uart.c
    ${SDK_DIR}/drivers_nrf/uart/nrf_drv_uart.c
    ${SDK_DIR}/drivers_nrf/common/nrf_drv_common.c
    )
  target_include_directories(bootloader PUBLIC
    ${SDK11_DIR}/libraries/util
    ${SDK_DIR}/drivers_nrf/common
    ${SDK_DIR}/drivers_nrf/uart
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
    ${TINYUSB_DIR}/portable/nordic/nrf5x/dcd_nrf5x.c
    ${TINYUSB_DIR}/common/tusb_fifo.c
    ${TINYUSB_DIR}/device/usbd.c
    ${TINYUSB_DIR}/device/usbd_control.c
    ${TINYUSB_DIR}/class/cdc/cdc_device.c
    ${TINYUSB_DIR}/class/msc/msc_device.c
    ${TINYUSB_DIR}/tusb.c
    )
endif ()

#----------------------------------------------------
# MCU Variant differences
# Supported are: nrf52 (nrf52832), nrf52833, nrf52840
#----------------------------------------------------

# SD_VERSION can be overwritten by board.cmake
if(NOT DEFINED SD_VERSION)
  if(MCU_VARIANT STREQUAL "nrf52833")
    set(SD_VERSION 7.3.0)
  else()
    set(SD_VERSION 6.1.1)
  endif()
endif ()

if (MCU_VARIANT STREQUAL "nrf52")
  set(SD_NAME s132)
  set(DFU_DEV_REV 0xADAF)
  set(DFU_APP_DATA_RESERVED 7*4096)
  target_compile_definitions(bootloader PUBLIC NRF52 NRF52832_XXAA)
elseif (MCU_VARIANT STREQUAL "nrf52833")
  set(DFU_DEV_REV 52833)
  set(DFU_APP_DATA_RESERVED 7*4096)
  target_compile_definitions(bootloader PUBLIC NRF52833_XXAA)
  if (NOT DEFINED SD_NAME)
    set(SD_NAME s140)
  endif ()
elseif (MCU_VARIANT STREQUAL "nrf52840")
  set(DFU_DEV_REV 52840)
  # App reserved 40KB (8+32) to match circuitpython for 840
  set(DFU_APP_DATA_RESERVED 10*4096)
  target_compile_definitions(bootloader PUBLIC NRF52840_XXAA)
  if (NOT DEFINED SD_NAME)
    set(SD_NAME s140)
  endif ()
else ()
  message(FATAL_ERROR "MCU_VARIANT ${MCU_VARIANT} is unknown")
endif ()

set(SD_FILENAME ${SD_NAME}_nrf52_${SD_VERSION})
set(SD_HEX ${SOFTDEVICE_DIR}/${SD_FILENAME}/${SD_FILENAME}_softdevice.hex)

string(TOUPPER ${SD_NAME} SD_NAME_UPPER)
target_compile_definitions(bootloader PUBLIC ${SD_NAME_UPPER})
target_include_directories(bootloader PUBLIC
  ${SOFTDEVICE_DIR}/${SD_FILENAME}/${SD_FILENAME}_API/include
  )

# debug build
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  # TODO not work yet, also need to add segger rtt, DFU_APP_DATA_RESERVED=0, BOOTLOADER_REGION_START=0xED000
  target_sources(bootloader PUBLIC lib/SEGGER_RTT/RTT/SEGGER_RTT.c)
  target_include_directories(bootloader PUBLIC lib/SEGGER_RTT/RTT)
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
  target_compile_definitions(bootloader PUBLIC DFU_APP_DATA_RESERVED=${DFU_APP_DATA_RESERVED})
endif ()

#----------------------------------
# ANT_LICENSE_KEY handling
#----------------------------------
if (DEFINED ANT_LICENSE_KEY)
  target_compile_definitions(bootloader PUBLIC ANT_LICENSE_KEY=${ANT_LICENSE_KEY})
endif()

#----------------------------------
# Get UF2 version from git
#----------------------------------
execute_process(COMMAND git describe --dirty --always --tags OUTPUT_VARIABLE GIT_VERSION)
string(STRIP ${GIT_VERSION} GIT_VERSION)

string(REPLACE "-" ";" RELEASE_VERSION ${GIT_VERSION})
list(GET RELEASE_VERSION 0 RELEASE_VERSION)
string(REPLACE "." ";" RELEASE_VERSION ${RELEASE_VERSION})
list(GET RELEASE_VERSION 0 RELEASE_VERSION_MAJOR)
list(GET RELEASE_VERSION 1 RELEASE_VERSION_MINOR)
list(GET RELEASE_VERSION 2 RELEASE_VERSION_PATCH)
math(EXPR MK_BOOTLOADER_VERSION "(${RELEASE_VERSION_MAJOR} << 16) + (${RELEASE_VERSION_MINOR} << 8) + ${RELEASE_VERSION_PATCH}")

cmake_print_variables(GIT_VERSION MK_BOOTLOADER_VERSION)

target_compile_definitions(bootloader PUBLIC
  UF2_VERSION="${GIT_VERSION}"
  BLEDIS_FW_VERSION="${GIT_VERSION} ${SD_NAME} ${SD_VERSION}"
  MK_BOOTLOADER_VERSION=${MK_BOOTLOADER_VERSION}
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

add_custom_target(flash-bootloader
  DEPENDS bootloader
  COMMAND ${NRFJPROG} --program $<TARGET_FILE:bootloader> --verify --sectoranduicrerase -f nrf52 --reset
  )

add_custom_target(flash-uf2
  DEPENDS bootloader
  COMMAND ${Python_EXECUTABLE} ${UF2CONV_PY} -f ${UF2_FAMILY_ID_BOOTLOADER} --deploy $<TARGET_FILE_DIR:bootloader>/bootloader_mbr.uf2
  )

add_custom_target(flash-sd
  DEPENDS flash-mbr
  COMMAND ${NRFJPROG} --program ${SD_HEX} --verify --sectorerase -f nrf52
  )

add_custom_target(flash-mbr
  COMMAND ${NRFJPROG} --program ${MBR_HEX} --verify --sectorerase -f nrf52
  )

add_custom_target(flash-erase
  COMMAND ${NRFJPROG} -f nrf52 --eraseall
  )

add_custom_target(flash-all
  DEPENDS bootloader flash-erase
  COMMAND ${NRFJPROG} --program ${SD_HEX} --verify --sectorerase -f nrf52
  COMMAND ${NRFJPROG} --program $<TARGET_FILE:bootloader> --verify --sectoranduicrerase -f nrf52 --reset
  )

# flash skip crc magic ( app valid = 0x0001, crc = 0x0000 )
#add_custom_target(flash-skip-crc
#  #COMMAND ${NRFJPROG} --memwr $(BOOT_SETTING_ADDR) --val 0x00000001 -f nrf52
#  COMMAND ${NRFJPROG} --memwr 0xFF000 --val 0x00000001 -f nrf52
#  #COMMAND ${NRFJPROG} --memwr 0x7F000 --val 0x00000001 -f nrf52
#  )
