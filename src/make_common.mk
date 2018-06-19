#******************************************************************************
# CONFIGURE (no spaces!)
# - VERSION_MAJOR, VERSION_MINOR, VERSION_REVISION: e.g 5.0.0 or 5.1.0 (usually
# is the same to the SoftDevice if possible)
#
# - SDK_PATH   : path to SDK directory
# - SRC_PATH   : path to src folder
#
# - SD_NAME    : e.g s132, s140
# - SD_VERSION : e.g 5.1.0
# - SD_HEX     : path to bootloader hex binary
#******************************************************************************
VERSION_MAJOR    = 6
VERSION_MINOR    = 0
VERSION_REVISION = 0

SDK_PATH         = ../../lib/sdk/components
SDK11_PATH       = ../../lib/sdk11/components

SRC_PATH         = ..
TUSB_PATH		     = ../../lib/tinyusb/src

SD_NAME          = s140
SD_VERSION       = 6.0.0

MK_DIS_FIRMWARE  = "$(subst s,S,$(SD_NAME)) $(SD_VERSION), $(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_REVISION)"

SD_PATH          = ../../lib/softdevice/$(SD_VERSION)
SD_HEX           = $(SD_PATH)/$(SD_NAME)/hex/$(SD_NAME)_nrf52_$(SD_VERSION)_softdevice.hex
LINKER_SCRIPT    = $(SRC_PATH)/$(SD_NAME)_$(SD_VERSION).ld

ifeq ($(VERSION_SINGLEBANK),1)
BANKMODE = single
C_SOURCE_FILES += $(SDK_PATH)/libraries/bootloader_dfu/dfu_single_bank.c
else
BANKMODE = dual
C_SOURCE_FILES += $(SDK_PATH)/libraries/bootloader_dfu/dfu_dual_bank.c
endif

BOOTLOADER_SD_SUFFIX = $(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_REVISION)_$(SD_NAME)_$(BANKMODE)

#******************************************************************************
# Tool configure
#******************************************************************************
NRFUTIL = nrfutil

ifeq ($(OS),Windows_NT)
PROGFILES = C:/Program Files (x86)
GNU_INSTALL_ROOT = $(PROGFILES)/GNU Tools ARM Embedded/6 2017-q2-update
else
GNU_INSTALL_ROOT = /usr
endif

MK := mkdir
RM := rm -rf

ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

GNU_PREFIX = arm-none-eabi

# Toolchain commands
CC      := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS      := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR      := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD      := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM      := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE    := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))


#******************************************************************************
# SOURCE FILES
#******************************************************************************

C_SOURCE_FILES += $(SRC_PATH)/main.c
C_SOURCE_FILES += $(SRC_PATH)/dfu_ble_svc.c
C_SOURCE_FILES += $(SRC_PATH)/usb/tusb_descriptors.c
C_SOURCE_FILES += $(SRC_PATH)/usb/msc_flash.c

# SDK 11 files
C_SOURCE_FILES += $(SDK11_PATH)/libraries/bootloader_dfu/bootloader.c
C_SOURCE_FILES += $(SDK11_PATH)/libraries/bootloader_dfu/bootloader_settings.c
C_SOURCE_FILES += $(SDK11_PATH)/libraries/bootloader_dfu/bootloader_util.c
C_SOURCE_FILES += $(SDK11_PATH)/libraries/bootloader_dfu/dfu_init_template.c
C_SOURCE_FILES += $(SDK11_PATH)/libraries/bootloader_dfu/dfu_transport_serial.c
C_SOURCE_FILES += $(SDK11_PATH)/libraries/bootloader_dfu/dfu_transport_ble.c

C_SOURCE_FILES += $(SDK11_PATH)/drivers_nrf/pstorage/pstorage_raw.c

C_SOURCE_FILES += $(SDK11_PATH)/ble/ble_services/ble_dfu/ble_dfu.c
C_SOURCE_FILES += $(SDK11_PATH)/ble/ble_services/ble_dis/ble_dis.c

# Latest SDK files
C_SOURCE_FILES += $(SDK_PATH)/libraries/timer/app_timer.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/scheduler/app_scheduler.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/util/app_error.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/util/app_util_platform.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/crc16/crc16.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/hci/hci_mem_pool.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/hci/hci_slip.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/hci/hci_transport.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/util/nrf_assert.c

#C_SOURCE_FILES += $(SDK_PATH)/libraries/uart/app_uart.c
#C_SOURCE_FILES += $(SDK_PATH)/drivers_nrf/uart/nrf_drv_uart.c

C_SOURCE_FILES += $(SDK_PATH)/drivers_nrf/common/nrf_drv_common.c
C_SOURCE_FILES += $(SDK_PATH)/drivers_nrf/hal/nrf_nvmc.c

C_SOURCE_FILES += $(SDK_PATH)/toolchain/system_nrf52840.c

# Tinyusb stack
C_SOURCE_FILES += $(TUSB_PATH)/portable/nordic/nrf5x/dcd_nrf5x.c
C_SOURCE_FILES += $(TUSB_PATH)/portable/nordic/nrf5x/hal_nrf5x.c
C_SOURCE_FILES += $(TUSB_PATH)/common/tusb_fifo.c
C_SOURCE_FILES += $(TUSB_PATH)/device/usbd.c
C_SOURCE_FILES += $(TUSB_PATH)/class/cdc/cdc_device.c
C_SOURCE_FILES += $(TUSB_PATH)/class/msc/msc_device.c
C_SOURCE_FILES += $(TUSB_PATH)/class/custom/custom_device.c
C_SOURCE_FILES += $(TUSB_PATH)/tusb.c

#******************************************************************************
# Assembly Files
#******************************************************************************
ASM_SOURCE_FILES  = $(SDK_PATH)/toolchain/gcc/gcc_startup_nrf52840.S

#******************************************************************************
# INCLUDE PATH
#******************************************************************************
INC_PATHS += -I$(SRC_PATH)/
INC_PATHS += -I$(SRC_PATH)/usb
INC_PATHS += -I$(TUSB_PATH)/

INC_PATHS += -I$(SDK11_PATH)/libraries/bootloader_dfu/hci_transport
INC_PATHS += -I$(SDK11_PATH)/libraries/bootloader_dfu
INC_PATHS += -I$(SDK11_PATH)/drivers_nrf/pstorage
INC_PATHS += -I$(SDK11_PATH)/ble/common
INC_PATHS += -I$(SDK11_PATH)/ble/ble_services/ble_dfu
INC_PATHS += -I$(SDK11_PATH)/ble/ble_services/ble_dis
INC_PATHS += -I$(SDK11_PATH)/libraries/util

INC_PATHS += -I$(SDK_PATH)/libraries/timer
INC_PATHS += -I$(SDK_PATH)/libraries/scheduler
INC_PATHS += -I$(SDK_PATH)/libraries/crc16
INC_PATHS += -I$(SDK_PATH)/libraries/util
INC_PATHS += -I$(SDK_PATH)/libraries/hci/config
INC_PATHS += -I$(SDK_PATH)/libraries/uart
INC_PATHS += -I$(SDK_PATH)/libraries/hci

INC_PATHS += -I$(SDK_PATH)/drivers_nrf/common
INC_PATHS += -I$(SDK_PATH)/drivers_nrf/hal
INC_PATHS += -I$(SDK_PATH)/drivers_nrf/config
INC_PATHS += -I$(SDK_PATH)/drivers_nrf/delay
INC_PATHS += -I$(SDK_PATH)/drivers_nrf/uart
INC_PATHS += -I$(SDK_PATH)/drivers_nrf/power
INC_PATHS += -I$(SDK_PATH)/drivers_nrf/usbd

INC_PATHS += -I$(SDK_PATH)/device

INC_PATHS += -I$(SDK_PATH)/toolchain/cmsis/include
INC_PATHS += -I$(SDK_PATH)/toolchain/gcc
INC_PATHS += -I$(SDK_PATH)/toolchain


INC_PATHS += -I$(SD_PATH)/common
INC_PATHS += -I$(SD_PATH)/$(SD_NAME)/headers
INC_PATHS += -I$(SD_PATH)/$(SD_NAME)/headers/nrf52


OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY))

#******************************************************************************
# Compiler Flags
# - Additional compiler flags
#******************************************************************************

#flags common to all targets
#CFLAGS += -DENABLE_SWO

#*************************
# Defined Symbol (MACROS)
#*************************
CFLAGS += -DBOOTLOADER_VERSION=0x0$(VERSION_MAJOR)0$(VERSION_MINOR)0$(VERSION_REVISION)0$(VERSION_SINGLEBANK)UL
CFLAGS += -DNRF52840_XXAA
CFLAGS += -D__HEAP_SIZE=0
CFLAGS += -DS140
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DMK_DIS_FIRMWARE='$(MK_DIS_FIRMWARE)'

CFLAGS += -DDFU_APP_DATA_RESERVED=7*4096

CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -Os -g3
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums 

#******************************************************************************
# Linker Flags
#
#******************************************************************************

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L$(SRC_PATH)/ -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections

# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

#******************************************************************************
# Assembler flags
#
#******************************************************************************
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += -D__HEAP_SIZE=0
ASMFLAGS += -DS140
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DBSP_DEFINES_ONLY
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DSOFTDEVICE_PRESENT

ASMFLAGS += -DFLOAT_ABI_HARD

MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

#******************************************************************************
# BUILD TARGETS
#******************************************************************************

ifeq ("$(VERBOSE)","1")
$(info CFLAGS   $(CFLAGS))
$(info )
$(info LDFLAGS  $(LDFLAGS))
$(info )
endif

#default target - first one defined
default: all

#building all targets
all:
	@$(MAKE) -s -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e clean
	@echo Making Feather52840 board
	@echo ----------------------
	@$(MAKE) -s -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e feather52840
	
	#@$(MAKE) -s -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e clean
	#@echo Making Metro52 board
	#@echo --------------------
	#@$(MAKE) -s -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e metro52

#target for printing all targets
help:
	@echo following targets are available:
	@echo - feather52840       : build for Feather nrf52 board
	@echo - metro52         : build for Metro nrf52 board
	@echo - flash_feather52840 : flash Feather nrf52 board
	@echo - flash_metro52   : flash Metro nrf52 board

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.S=.o) )

vpath %.c $(C_PATHS)
vpath %.S $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

export OUTPUT_FILENAME
export FINAL_BIN_DIR
BOOTLOADER_WITH_SD_NAME := $(OUTPUT_FILENAME)_$(BOOTLOADER_SD_SUFFIX)

# Target for Feather nrf52 board
feather52840: OUTPUT_FILENAME := feather52840_bootloader
feather52840: FINAL_BIN_DIR := ../../bin/feather52840/$(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_REVISION)/$(BANKMODE)
feather52840: CFLAGS += -DBOARD_FEATHER52840
feather52840: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

# Target for Metro nrf52 board
metro52: OUTPUT_FILENAME := metro52_bootloader
metro52: FINAL_BIN_DIR := ../../bin/metro52/$(BANKMODE)
metro52: CFLAGS += -DBOARD_METRO52
metro52: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	@$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.S
	@echo Assembly file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<
	
# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out

finalize: genhex genbin genpkg echosize

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex $(BOOTLOADER_WITH_SD_NAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
	@mergehex -q -m $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex $(SD_HEX) -o $(OUTPUT_BINARY_DIRECTORY)/$(BOOTLOADER_WITH_SD_NAME).hex
	@mkdir -p $(FINAL_BIN_DIR)
	@cp $(OUTPUT_BINARY_DIRECTORY)/$(BOOTLOADER_WITH_SD_NAME).hex $(FINAL_BIN_DIR)/

## Create .bin file
genbin:
	@echo Preparing: $(BOOTLOADER_WITH_SD_NAME).bin
	$(NO_ECHO)$(OBJCOPY) -j .text -j .data -j .bss -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(FINAL_BIN_DIR)/$(BOOTLOADER_WITH_SD_NAME).bin

## Create pkg file for bootloader only and bootloader+SD combo to use with DFU
genpkg:
	@echo Preparing: $(BOOTLOADER_WITH_SD_NAME).zip
	@$(NRFUTIL) dfu genpkg --dev-type 0x0052 --dev-revision 0xADAF --bootloader $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex --softdevice $(SD_HEX) $(FINAL_BIN_DIR)/$(BOOTLOADER_WITH_SD_NAME).zip 

echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

clean:
	@$(RM) $(BUILD_DIRECTORIES)

flash_feather52840: BOOTLOADER_WITH_SD_NAME := feather52840_bootloader_$(BOOTLOADER_SD_SUFFIX)
flash_feather52840: feather52840
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$(BOOTLOADER_WITH_SD_NAME).hex
	nrfjprog --program $(OUTPUT_BINARY_DIRECTORY)/$(BOOTLOADER_WITH_SD_NAME).hex -f nrf52 --chiperase --reset

flash_metro52: BOOTLOADER_WITH_SD_NAME := metro52_bootloader_$(BOOTLOADER_SD_SUFFIX)
flash_metro52: metro52
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$(BOOTLOADER_WITH_SD_NAME).hex
	nrfjprog --program $(OUTPUT_BINARY_DIRECTORY)/$(BOOTLOADER_WITH_SD_NAME).hex -f nrf52 --chiperase --reset
	
flash_sd:
	@echo Flashing: $(SD_HEX)
	nrfjprog --program $(SD_HEX) -f nrf52 --chiperase --reset
