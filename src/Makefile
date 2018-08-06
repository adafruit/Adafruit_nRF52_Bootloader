#******************************************************************************
# CONFIGURE (no spaces!)

#
# - SDK_PATH : path to SDK directory
# - SRC_PATH : path to src folder
#
# - SD_NAME  : e.g s132, s140
# - SD_VER1, SD_VER2, SD_VER3: SoftDevice version e.g 6.0.0
# - SD_VER4  : is build number for bootloader
# - SD_HEX   : to bootloader hex binary
#******************************************************************************
SDK_PATH        = ../lib/sdk/components
SDK11_PATH      = ../lib/sdk11/components

SRC_PATH        = .
TUSB_PATH       = ../lib/tinyusb/src


SD_VER1         = 6
SD_VER2         = 0
SD_VER3         = 0
SD_VER4         = 0
SD_VERSION      = $(SD_VER1).$(SD_VER2).$(SD_VER3)
SD_VERSION_FULL = $(SD_VERSION)r$(SD_VER4)

SD_PATH         = ../lib/softdevice/$(SD_VERSION)
SD_HEX          = $(SD_PATH)/$(SD_NAME)/hex/$(SD_NAME)_nrf52_$(SD_VERSION)_softdevice.hex
LINKER_SCRIPT   = $(SRC_PATH)/$(SD_NAME)_$(SD_VERSION).ld

OUTPUT_FILENAME = $(BOARD)_bootloader
BOOT_SD_NAME    = $(OUTPUT_FILENAME)_$(SD_NAME)_$(SD_VERSION_FULL)

BETA_DIR        = ../bin/$(BOARD)/beta
RELEASE_DIR     = ../bin/$(BOARD)/$(SD_VERSION_FULL)


MK_DIS_FIRMWARE = "$(SD_NAME) $(SD_VERSION_FULL)"

# Select the board to build for.
ifeq ($(BOARD),)
  $(info You must provide a BOARD parameter with 'BOARD=')
  $(info Possible values are:)
  $(info $(sort $(subst .h,,$(subst boards/,,$(wildcard boards/*)))))
  $(error BOARD not defined)
else
  ifeq ($(wildcard boards/$(BOARD).h),)
    $(error Invalid BOARD specified)
  endif
endif

BUILD = _build-$(BOARD)

NRF52840_BOARDLIST = pca10056 feather52840
IS_NRF52840 = $(findstring $(BOARD),$(NRF52840_BOARDLIST)) 

ifneq ($(IS_NRF52840),)
#nrf52840 board
SD_NAME = s140
else
#nrf52832 board
SD_NAME = s132
endif


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

ifeq ("$(V)","2")
QUIET := 
else
QUIET := @
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
C_SOURCE_FILES += $(SRC_PATH)/usb/msc_uf2.c
C_SOURCE_FILES += $(SRC_PATH)/usb/uf2/ghostfat.c

# SDK 11 files
C_SOURCE_FILES += $(SDK11_PATH)/libraries/bootloader_dfu/bootloader.c
C_SOURCE_FILES += $(SDK11_PATH)/libraries/bootloader_dfu/bootloader_settings.c
C_SOURCE_FILES += $(SDK11_PATH)/libraries/bootloader_dfu/bootloader_util.c
C_SOURCE_FILES += $(SDK11_PATH)/libraries/bootloader_dfu/dfu_init_template.c
C_SOURCE_FILES += $(SDK11_PATH)/libraries/bootloader_dfu/dfu_transport_serial.c
C_SOURCE_FILES += $(SDK11_PATH)/libraries/bootloader_dfu/dfu_transport_ble.c
C_SOURCE_FILES += $(SDK11_PATH)/libraries/bootloader_dfu/dfu_single_bank.c

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
C_SOURCE_FILES += $(TUSB_PATH)/device/usbd_auto_desc.c
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

#******************************************************************************
# Compiler Flags
# - Additional compiler flags
#******************************************************************************

#flags common to all targets
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -Os -g3
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums 

# Defined Symbol (MACROS)
CFLAGS += -DBOOTLOADER_VERSION=0x0$(SD_VER1)0$(SD_VER2)0$(SD_VER3)0$(SD_VER4)UL
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

CFLAGS += -DBOARD_$(shell echo $(BOARD) | tr '[:lower:]' '[:upper:]')


#******************************************************************************
# Linker Flags
#
#******************************************************************************

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(BUILD)/$(OUTPUT_FILENAME).map
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



C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(BUILD)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(BUILD)/, $(ASM_SOURCE_FILE_NAMES:.S=.o) )

vpath %.c $(C_PATHS)
vpath %.S $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

#******************************************************************************
# BUILD TARGETS
#******************************************************************************

ifeq ("$(V)","1")
$(info CFLAGS   $(CFLAGS))
$(info )
$(info LDFLAGS  $(LDFLAGS))
$(info )
endif

.phony: all clean size flash sd

all: $(BUILD)/$(OUTPUT_FILENAME).out size

flash: $(BUILD)/$(OUTPUT_FILENAME).hex
	@echo Flashing: $<
	nrfjprog --program $< --sectoranduicrerase -f nrf52 --reset
	
sd:
	@echo Flashing: $(SD_HEX)
	nrfjprog --program $(SD_HEX) -f nrf52 --chiperase  --reset	


## Create build directories
$(BUILD):
	@$(MK) $@

clean:
	@$(RM) $(BUILD)
	
# Create objects from C SRC files
$(BUILD)/%.o: %.c
	@echo CC $(notdir $<)
	$(QUIET)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(BUILD)/%.o: %.S
	@echo AS $(notdir $<)
	$(QUIET)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<
	
# Link
$(BUILD)/$(OUTPUT_FILENAME).out: $(BUILD) $(OBJECTS)
	@echo LD $(OUTPUT_FILENAME).out
	$(QUIET)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(BUILD)/$(OUTPUT_FILENAME).out

size: $(BUILD)/$(OUTPUT_FILENAME).out
	-@echo ''
	$(QUIET)$(SIZE) $<
	-@echo ''


.phony: genhex genpkg beta release

## Create binary .hex file from the .out file
genhex: $(BUILD)/$(OUTPUT_FILENAME).hex

$(BUILD)/$(OUTPUT_FILENAME).hex: $(BUILD)/$(OUTPUT_FILENAME).out
	@echo CR $(OUTPUT_FILENAME).hex
	@echo CR $(BOOT_SD_NAME).hex
	$(QUIET)$(OBJCOPY) -O ihex $< $@
	@mergehex -q -m $@ $(SD_HEX) -o $(BUILD)/$(BOOT_SD_NAME).hex

## Create pkg file for bootloader+SD combo to use with DFU
genpkg: $(BUILD)/$(BOOT_SD_NAME).zip

$(BUILD)/$(BOOT_SD_NAME).zip: $(BUILD)/$(OUTPUT_FILENAME).hex
	@$(NRFUTIL) dfu genpkg --dev-type 0x0052 --dev-revision 0xADAF --bootloader $< --softdevice $(SD_HEX) $@ 

# Create SD+bootloader combo with hex & dfu package at beta folder
beta: genhex genpkg
	@echo CR $(BETA_DIR)/$(BOOT_SD_NAME).hex
	@echo CR $(BETA_DIR)/$(BOOT_SD_NAME).zip
	@mkdir -p $(BETA_DIR)
	@cp $(BUILD)/$(BOOT_SD_NAME).hex $(BETA_DIR)/$(BOOT_SD_NAME).hex
	@cp $(BUILD)/$(BOOT_SD_NAME).zip $(BETA_DIR)/$(BOOT_SD_NAME).zip

release: genhex genpkg
	@echo CR $(RELEASE_DIR)/$(BOOT_SD_NAME).hex
	@echo CR $(RELEASE_DIR)/$(BOOT_SD_NAME).zip
	@mkdir -p $(RELEASE_DIR)
	@cp $(BUILD)/$(BOOT_SD_NAME).hex $(RELEASE_DIR)/$(BOOT_SD_NAME).hex
	@cp $(BUILD)/$(BOOT_SD_NAME).zip $(RELEASE_DIR)/$(BOOT_SD_NAME).zip
	