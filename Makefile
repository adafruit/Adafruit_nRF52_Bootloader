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
SDK_PATH        = lib/sdk/components
SDK11_PATH      = lib/sdk11/components

SRC_PATH        = src
TUSB_PATH       = lib/tinyusb/src
NRFX_PATH				= lib/nrfx


SD_VER1         = 6
SD_VER2         = 0
SD_VER3         = 0
SD_VER4         = 0
SD_VERSION      = $(SD_VER1).$(SD_VER2).$(SD_VER3)
SD_VERSION_FULL = $(SD_VERSION)r$(SD_VER4)

SD_PATH         = lib/softdevice/$(SD_NAME)/$(SD_VERSION)
SD_HEX          = $(SD_PATH)/hex/$(SD_NAME)_nrf52_$(SD_VERSION)_softdevice.hex
LD_FILE   			= $(SRC_PATH)/linker/$(SD_NAME)_v$(SD_VER1).ld

OUTPUT_FILENAME = $(BOARD)_bootloader
BOOT_SD_NAME    = $(OUTPUT_FILENAME)_$(SD_NAME)_$(SD_VERSION_FULL)

BETA_DIR        = bin/$(BOARD)/beta
RELEASE_DIR     = bin/$(BOARD)/$(SD_VERSION_FULL)


MK_DIS_FIRMWARE = "$(SD_NAME) $(SD_VERSION_FULL)"

#******************************************************************************
# Tool configure
#******************************************************************************
NRFUTIL = nrfutil


ifneq ($(JLINK),)
NRFJPROG = nrfjprog -s $(JLINK)
else
NRFJPROG = nrfjprog 
endif

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

#*********************************
# Select the board to build
#*********************************
ifeq ($(BOARD),)
  $(info You must provide a BOARD parameter with 'BOARD=')
  $(info Supported boards are: $(sort $(subst .h,,$(subst src/boards/,,$(wildcard src/boards/*)))))
  $(error BOARD not defined)
else
  ifeq ($(wildcard src/boards/$(BOARD).h),)
    $(error Invalid BOARD specified)
  endif
endif

BUILD = _build-$(BOARD)

NRF52840_BOARDLIST = pca10056 feather52840
IS_NRF52840 = $(findstring $(BOARD),$(NRF52840_BOARDLIST))

ifneq ($(IS_NRF52840),)
#nrf52840 board
$(info nRF52840)
SD_NAME = s140
else
#nrf52832 board
$(info nRF52832)
SD_NAME = s132
endif



#******************************************************************************
# SOURCE FILES
#******************************************************************************

# src
C_SOURCE_FILES += $(SRC_PATH)/main.c
C_SOURCE_FILES += $(SRC_PATH)/dfu_ble_svc.c

# nrfx

C_SOURCE_FILES += $(NRFX_PATH)/drivers/src/nrfx_power.c

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



C_SOURCE_FILES += $(SDK_PATH)/drivers_nrf/hal/nrf_nvmc.c


ifneq ($(IS_NRF52840),)

# src
C_SOURCE_FILES += $(SRC_PATH)/usb/tusb_descriptors.c
C_SOURCE_FILES += $(SRC_PATH)/usb/usb.c
C_SOURCE_FILES += $(SRC_PATH)/usb/msc_uf2.c
C_SOURCE_FILES += $(SRC_PATH)/usb/uf2/ghostfat.c

# nrfx
C_SOURCE_FILES += $(NRFX_PATH)/mdk/system_nrf52840.c

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

else

C_SOURCE_FILES += $(NRFX_PATH)/mdk/system_nrf52.c

C_SOURCE_FILES += $(SDK_PATH)/libraries/uart/app_uart.c
C_SOURCE_FILES += $(SDK_PATH)/drivers_nrf/uart/nrf_drv_uart.c
C_SOURCE_FILES += $(SDK_PATH)/drivers_nrf/common/nrf_drv_common.c

IPATH += $(SDK_PATH)/drivers_nrf/common
IPATH += $(SDK_PATH)/drivers_nrf/uart

endif


#******************************************************************************
# Assembly Files
#******************************************************************************
ifneq ($(IS_NRF52840),)
ASM_SOURCE_FILES  = $(NRFX_PATH)/mdk/gcc_startup_nrf52840.S
else
ASM_SOURCE_FILES  = $(NRFX_PATH)/mdk/gcc_startup_nrf52.S
endif

#******************************************************************************
# INCLUDE PATH
#******************************************************************************

# src
IPATH += $(SRC_PATH)
IPATH += $(SRC_PATH)/cmsis/include
IPATH += $(SRC_PATH)/usb
IPATH += $(TUSB_PATH)

# nrfx
IPATH += $(NRFX_PATH)
IPATH += $(NRFX_PATH)/mdk
IPATH += $(NRFX_PATH)/hal
IPATH += $(NRFX_PATH)/drivers/include

IPATH += $(SDK11_PATH)/libraries/bootloader_dfu/hci_transport
IPATH += $(SDK11_PATH)/libraries/bootloader_dfu
IPATH += $(SDK11_PATH)/drivers_nrf/pstorage
IPATH += $(SDK11_PATH)/ble/common
IPATH += $(SDK11_PATH)/ble/ble_services/ble_dfu
IPATH += $(SDK11_PATH)/ble/ble_services/ble_dis
IPATH += $(SDK11_PATH)/libraries/util

IPATH += $(SDK_PATH)/libraries/timer
IPATH += $(SDK_PATH)/libraries/scheduler
IPATH += $(SDK_PATH)/libraries/crc16
IPATH += $(SDK_PATH)/libraries/util
IPATH += $(SDK_PATH)/libraries/hci/config
IPATH += $(SDK_PATH)/libraries/uart
IPATH += $(SDK_PATH)/libraries/hci


#IPATH += $(SDK_PATH)/drivers_nrf/hal
#IPATH += $(SDK_PATH)/drivers_nrf/config
IPATH += $(SDK_PATH)/drivers_nrf/delay
#IPATH += $(SDK_PATH)/drivers_nrf/power

# Softdevice
IPATH += $(SD_PATH)/headers
IPATH += $(SD_PATH)/headers/nrf52


INC_PATHS = $(addprefix -I,$(IPATH))

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
CFLAGS += -DMK_BOOTLOADER_VERSION=0x0$(SD_VER1)0$(SD_VER2)0$(SD_VER3)0$(SD_VER4)UL

CFLAGS += -D__HEAP_SIZE=0
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DMK_DIS_FIRMWARE='$(MK_DIS_FIRMWARE)'
CFLAGS += -DDFU_APP_DATA_RESERVED=7*4096

CFLAGS += -DBOARD_$(shell echo $(BOARD) | tr '[:lower:]' '[:upper:]')

ifneq ($(IS_NRF52840),)

CFLAGS += -DNRF52840_XXAA
CFLAGS += -DS140

else

CFLAGS += -DNRF52
CFLAGS += -DS132

endif


#******************************************************************************
# Linker Flags
#
#******************************************************************************

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(BUILD)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L$(SRC_PATH)/linker -T$(LD_FILE)
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
ASMFLAGS += -D__HEAP_SIZE=0
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DBSP_DEFINES_ONLY
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DFLOAT_ABI_HARD

ifneq ($(IS_NRF52840),)

ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += -DS140

else

ASMFLAGS += -DNRF52
ASMFLAGS += -DS132

endif




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
	$(NRFJPROG) --program $< --sectoranduicrerase -f nrf52 --reset
	
sd:
	@echo Flashing: $(SD_HEX)
	$(NRFJPROG) --program $(SD_HEX) -f nrf52 --chiperase  --reset	


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
	