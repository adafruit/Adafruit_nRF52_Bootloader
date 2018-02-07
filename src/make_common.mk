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
VERSION_MAJOR    = 5
VERSION_MINOR    = 1
VERSION_REVISION = 0

SDK_PATH         = ../../nRF5_SDK_11.0.0_89a8197/components
SRC_PATH         = ..

SD_NAME          = s132
SD_VERSION       = 5.1.0

SD_PATH          = ../../softdevice/$(SD_NAME)/$(SD_VERSION)
SD_HEX           = $(SD_PATH)/hex/$(SD_NAME)_nrf52_$(SD_VERSION)_softdevice.hex
LINKER_SCRIPT    = $(SRC_PATH)/$(SD_NAME)_$(SD_VERSION).ld

ifeq ($(VERSION_SINGLEBANK),1)
BANKMODE = single
C_SOURCE_FILES += $(SDK_PATH)/libraries/bootloader_dfu/dfu_single_bank.c
else
BANKMODE = dual
C_SOURCE_FILES += $(SDK_PATH)/libraries/bootloader_dfu/dfu_dual_bank.c
endif

BOOTLOADER_S132_SUFFIX = $(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_REVISION)_$(SD_NAME)_$(BANKMODE)
FINAL_BIN_DIR := ../../bin

TEMPLATE_PATH = $(SDK_PATH)/toolchain/gcc


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
#source common to all targets
C_SOURCE_FILES += $(SRC_PATH)/main.c
C_SOURCE_FILES += $(SRC_PATH)/dfu_ble_svc.c

C_SOURCE_FILES += $(SDK_PATH)/libraries/bootloader_dfu/bootloader.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/bootloader_dfu/bootloader_settings.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/bootloader_dfu/bootloader_util.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/bootloader_dfu/dfu_init_template.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/bootloader_dfu/dfu_transport_serial.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/bootloader_dfu/dfu_transport_ble.c

C_SOURCE_FILES += $(SDK_PATH)/libraries/timer/app_timer.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/timer/app_timer_appsh.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/scheduler/app_scheduler.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/util/app_error.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/util/app_error_weak.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/util/app_util_platform.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/crc16/crc16.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/hci/hci_mem_pool.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/hci/hci_slip.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/hci/hci_transport.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/util/nrf_assert.c
C_SOURCE_FILES += $(SDK_PATH)/libraries/uart/app_uart.c

C_SOURCE_FILES += $(SDK_PATH)/drivers_nrf/delay/nrf_delay.c
C_SOURCE_FILES += $(SDK_PATH)/drivers_nrf/common/nrf_drv_common.c
C_SOURCE_FILES += $(SDK_PATH)/drivers_nrf/uart/nrf_drv_uart.c

C_SOURCE_FILES += $(SDK_PATH)/ble/common/ble_advdata.c
C_SOURCE_FILES += $(SDK_PATH)/ble/common/ble_conn_params.c
C_SOURCE_FILES += $(SDK_PATH)/ble/common/ble_srv_common.c
C_SOURCE_FILES += $(SDK_PATH)/ble/ble_services/ble_dfu/ble_dfu.c
C_SOURCE_FILES += $(SDK_PATH)/ble/ble_services/ble_dis/ble_dis.c

C_SOURCE_FILES += $(SDK_PATH)/drivers_nrf/pstorage/pstorage_raw.c
C_SOURCE_FILES += $(SDK_PATH)/toolchain/system_nrf52.c

C_SOURCE_FILES += ../../softdevice/common/softdevice_handler/softdevice_handler.c
C_SOURCE_FILES += ../../softdevice/common/softdevice_handler/softdevice_handler_appsh.c



#******************************************************************************
# Assembly Files
#******************************************************************************
ASM_SOURCE_FILES  = $(SDK_PATH)/toolchain/gcc/gcc_startup_nrf52.S

#******************************************************************************
# INCLUDE PATH
#******************************************************************************
INC_PATHS += -I$(SRC_PATH)/

INC_PATHS += -I$(SDK_PATH)/libraries/bootloader_dfu/hci_transport
INC_PATHS += -I$(SDK_PATH)/libraries/bootloader_dfu

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

INC_PATHS += -I../../softdevice/common
INC_PATHS += -I../../softdevice/common/softdevice_handler/
INC_PATHS += -I$(SD_PATH)/headers
INC_PATHS += -I$(SD_PATH)/headers/nrf52

INC_PATHS += -I$(SDK_PATH)/device
INC_PATHS += -I$(SDK_PATH)/drivers_nrf/pstorage

INC_PATHS += -I$(SDK_PATH)/toolchain/cmsis/include
INC_PATHS += -I$(SDK_PATH)/toolchain/gcc
INC_PATHS += -I$(SDK_PATH)/toolchain

INC_PATHS += -I$(SDK_PATH)/ble/common
INC_PATHS += -I$(SDK_PATH)/ble/ble_services/ble_dfu
INC_PATHS += -I$(SDK_PATH)/ble/ble_services/ble_dis

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
CFLAGS += -DNRF52
CFLAGS += -DNRF52_PAN_12
CFLAGS += -DNRF52_PAN_15
CFLAGS += -DNRF52_PAN_58
CFLAGS += -DNRF52_PAN_55
CFLAGS += -DNRF52_PAN_54
CFLAGS += -DNRF52_PAN_31
CFLAGS += -DNRF52_PAN_30
CFLAGS += -DNRF52_PAN_51
CFLAGS += -DNRF52_PAN_36
CFLAGS += -DNRF52_PAN_53
CFLAGS += -D__HEAP_SIZE=0
CFLAGS += -DS132
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DNRF52_PAN_20
CFLAGS += -DNRF52_PAN_64
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF52_PAN_62
CFLAGS += -DNRF52_PAN_63

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
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
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
ASMFLAGS += -DNRF52
ASMFLAGS += -DNRF52_PAN_12
ASMFLAGS += -DNRF52_PAN_15
ASMFLAGS += -DNRF52_PAN_58
ASMFLAGS += -DNRF52_PAN_55
ASMFLAGS += -DNRF52_PAN_54
ASMFLAGS += -DNRF52_PAN_31
ASMFLAGS += -DNRF52_PAN_30
ASMFLAGS += -DNRF52_PAN_51
ASMFLAGS += -DNRF52_PAN_36
ASMFLAGS += -DNRF52_PAN_53
ASMFLAGS += -D__HEAP_SIZE=0
ASMFLAGS += -DS132
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DBSP_DEFINES_ONLY
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DNRF52_PAN_20
ASMFLAGS += -DNRF52_PAN_64
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF52_PAN_62
ASMFLAGS += -DNRF52_PAN_63

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
	@echo Making Feather52 board
	@echo ----------------------
	@$(MAKE) -s -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e feather52
	@$(MAKE) -s -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e clean
	@echo Making Metro52 board
	@echo --------------------
	@$(MAKE) -s -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e metro52

#target for printing all targets
help:
	@echo following targets are available:
	@echo - feather52       : build for Feather nrf52 board
	@echo - metro52         : build for Metro nrf52 board
	@echo - flash_feather52 : flash Feather nrf52 board
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
BOOTLOADER_WITH_S132_NAME := $(OUTPUT_FILENAME)_$(BOOTLOADER_S132_SUFFIX)

# Target for Feather nrf52 board
feather52: OUTPUT_FILENAME := feather52_bootloader
feather52: FINAL_BIN_DIR := $(FINAL_BIN_DIR)/feather52/$(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_REVISION)/$(BANKMODE)
feather52: CFLAGS += -DBOARD_FEATHER52
feather52: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

# Target for Metro nrf52 board
metro52: OUTPUT_FILENAME := metro52_bootloader
metro52: FINAL_BIN_DIR := $(FINAL_BIN_DIR)/metro52/$(BANKMODE)
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
	@echo Preparing: $(OUTPUT_FILENAME).hex $(BOOTLOADER_WITH_S132_NAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
	@mergehex -q -m $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex $(SD_HEX) -o $(OUTPUT_BINARY_DIRECTORY)/$(BOOTLOADER_WITH_S132_NAME).hex
	@mkdir -p $(FINAL_BIN_DIR)
	@cp $(OUTPUT_BINARY_DIRECTORY)/$(BOOTLOADER_WITH_S132_NAME).hex $(FINAL_BIN_DIR)/

## Create .bin file
genbin:
	@echo Preparing: $(BOOTLOADER_WITH_S132_NAME).bin
	$(NO_ECHO)$(OBJCOPY) -j .text -j .data -j .bss -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(FINAL_BIN_DIR)/$(BOOTLOADER_WITH_S132_NAME).bin

## Create pkg file for bootloader only and bootloader+SD combo to use with DFU
genpkg:
	@echo Preparing: $(BOOTLOADER_WITH_S132_NAME).zip
	@$(NRFUTIL) dfu genpkg --dev-type 0x0052 --dev-revision 0xADAF --bootloader $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex --softdevice $(SD_HEX) $(FINAL_BIN_DIR)/$(BOOTLOADER_WITH_S132_NAME).zip 

echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

clean:
	@$(RM) $(BUILD_DIRECTORIES)

flash_feather52: BOOTLOADER_WITH_S132_NAME := feather52_bootloader_$(BOOTLOADER_S132_SUFFIX)
flash_feather52: feather52
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$(BOOTLOADER_WITH_S132_NAME).hex
	nrfjprog --program $(OUTPUT_BINARY_DIRECTORY)/$(BOOTLOADER_WITH_S132_NAME).hex -f nrf52 --chiperase --reset

flash_metro52: BOOTLOADER_WITH_S132_NAME := metro52_bootloader_$(BOOTLOADER_S132_SUFFIX)
flash_metro52: metro52
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$(BOOTLOADER_WITH_S132_NAME).hex
	nrfjprog --program $(OUTPUT_BINARY_DIRECTORY)/$(BOOTLOADER_WITH_S132_NAME).hex -f nrf52 --chiperase --reset	