#******************************************************************************
# CONFIGURE
# - SDK_PATH    : path to SDK directory
# - SRC_PATH    : path to src folder
#
# - SD_NAME     : e.g s132, s140
# - SD_VERSION : SoftDevice version e.g 6.0.0
# - SD_HEX      : to bootloader hex binary
#******************************************************************************
SRC_PATH     = src

SDK_PATH     = lib/sdk/components
SDK11_PATH   = lib/sdk11/components
SD_PATH      = lib/softdevice/$(SD_FILENAME)

TUSB_PATH    = lib/tinyusb/src
NRFX_PATH    = lib/nrfx

SD_VERSION   = 6.1.1
SD_FILENAME  = $(SD_NAME)_nrf52_$(SD_VERSION)
SD_API_PATH  = $(SD_PATH)/$(SD_FILENAME)_API
SD_HEX       = $(SD_PATH)/$(SD_FILENAME)_softdevice.hex

LD_FILE      = $(SRC_PATH)/linker/$(MCU_SUB_VARIANT)_$(SD_NAME)_v$(word 1, $(subst ., ,$(SD_VERSION))).ld

MERGED_FNAME = $(OUTPUT_FILENAME)_$(SD_NAME)_$(SD_VERSION)

GIT_VERSION = $(shell git describe --dirty --always --tags)
GIT_SUBMODULE_VERSIONS = $(shell git submodule status | cut -d' ' -f3,4 | paste -s -d" " -)

OUTPUT_FILENAME = $(BOARD)_bootloader-$(GIT_VERSION)

#******************************************************************************
# Tool configure
#******************************************************************************
NRFUTIL = adafruit-nrfutil

ifneq ($(JLINK),)
NRFJPROG = nrfjprog -s $(JLINK)
else
NRFJPROG = nrfjprog
endif

ifeq ($(OS),Windows_NT)
PROGFILES = C:/Program Files (x86)
GNU_INSTALL_ROOT = $(PROGFILES)/GNU Tools ARM Embedded/7 2018-q2-update/bin/
endif

MK := mkdir
RM := rm -rf

# Verbose mode (V=). 0: default, 1: print out CFLAG, LDFLAG 2: print all compile command
ifeq ("$(V)","2")
QUIET =
else
QUIET = @
endif

GNU_PREFIX = arm-none-eabi

# Toolchain commands
CC      := '$(GNU_INSTALL_ROOT)$(GNU_PREFIX)-gcc'
AS      := '$(GNU_INSTALL_ROOT)$(GNU_PREFIX)-as'
AR      := '$(GNU_INSTALL_ROOT)$(GNU_PREFIX)-ar' -r
LD      := '$(GNU_INSTALL_ROOT)$(GNU_PREFIX)-ld'
NM      := '$(GNU_INSTALL_ROOT)$(GNU_PREFIX)-nm'
OBJDUMP := '$(GNU_INSTALL_ROOT)$(GNU_PREFIX)-objdump'
OBJCOPY := '$(GNU_INSTALL_ROOT)$(GNU_PREFIX)-objcopy'
SIZE    := '$(GNU_INSTALL_ROOT)$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#*********************************
# Select the board to build
#*********************************
BOARD_LIST = $(sort $(subst .h,,$(subst src/boards/,,$(wildcard src/boards/*))))

ifeq ($(filter $(BOARD),$(BOARD_LIST)),)
  $(info You must provide a BOARD parameter with 'BOARD='. Supported boards are:)
  $(info $(BOARD_LIST))
  $(error Invalid BOARD specified)
endif

# Build directory
BUILD = _build-$(BOARD)

# Board specific
-include src/boards/$(BOARD)/board.mk

# MCU_SUB_VARIANT can be nrf52 (nrf52832), nrf52833, nrf52840
ifeq ($(MCU_SUB_VARIANT),nrf52)
  SD_NAME = s132
  DFU_DEV_REV = 0xADAF
  MCU_FLAGS = -DNRF52 -DNRF52832_XXAA -DS132
else ifeq ($(MCU_SUB_VARIANT),nrf52833)
  SD_NAME = s140
  DFU_DEV_REV = 52840
  MCU_FLAGS = -DNRF52833_XXAA -DS140
else ifeq ($(MCU_SUB_VARIANT),nrf52840)
  SD_NAME = s140
  DFU_DEV_REV = 52840
  MCU_FLAGS = -DNRF52840_XXAA -DS140
else
  $(error Sub Variant $(MCU_SUB_VARIANT) is unknown)
endif

#******************************************************************************
# SOURCE FILES
#******************************************************************************

# src
C_SOURCE_FILES += $(SRC_PATH)/main.c
C_SOURCE_FILES += $(SRC_PATH)/boards.c
C_SOURCE_FILES += $(SRC_PATH)/flash_nrf5x.c
C_SOURCE_FILES += $(SRC_PATH)/dfu_ble_svc.c
C_SOURCE_FILES += $(SRC_PATH)/dfu_init.c

# nrfx
C_SOURCE_FILES += $(NRFX_PATH)/drivers/src/nrfx_power.c
C_SOURCE_FILES += $(NRFX_PATH)/drivers/src/nrfx_nvmc.c
C_SOURCE_FILES += $(NRFX_PATH)/mdk/system_$(MCU_SUB_VARIANT).c

# SDK 11 files
C_SOURCE_FILES += $(SDK11_PATH)/libraries/bootloader_dfu/bootloader.c
C_SOURCE_FILES += $(SDK11_PATH)/libraries/bootloader_dfu/bootloader_settings.c
C_SOURCE_FILES += $(SDK11_PATH)/libraries/bootloader_dfu/bootloader_util.c
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

# UART or USB Serial
ifeq ($(MCU_SUB_VARIANT),nrf52)
C_SOURCE_FILES += $(SDK_PATH)/libraries/uart/app_uart.c
C_SOURCE_FILES += $(SDK_PATH)/drivers_nrf/uart/nrf_drv_uart.c
C_SOURCE_FILES += $(SDK_PATH)/drivers_nrf/common/nrf_drv_common.c

IPATH += $(SDK11_PATH)/libraries/util
IPATH += $(SDK_PATH)/drivers_nrf/common
IPATH += $(SDK_PATH)/drivers_nrf/uart

else
# src
C_SOURCE_FILES += $(SRC_PATH)/usb/usb_desc.c
C_SOURCE_FILES += $(SRC_PATH)/usb/usb.c
C_SOURCE_FILES += $(SRC_PATH)/usb/msc_uf2.c
C_SOURCE_FILES += $(SRC_PATH)/usb/uf2/ghostfat.c

# TinyUSB stack
C_SOURCE_FILES += $(TUSB_PATH)/portable/nordic/nrf5x/dcd_nrf5x.c
C_SOURCE_FILES += $(TUSB_PATH)/common/tusb_fifo.c
C_SOURCE_FILES += $(TUSB_PATH)/device/usbd.c
C_SOURCE_FILES += $(TUSB_PATH)/device/usbd_control.c
C_SOURCE_FILES += $(TUSB_PATH)/class/cdc/cdc_device.c
C_SOURCE_FILES += $(TUSB_PATH)/class/msc/msc_device.c
C_SOURCE_FILES += $(TUSB_PATH)/tusb.c

endif


#******************************************************************************
# Assembly Files
#******************************************************************************
ASM_SOURCE_FILES  = $(NRFX_PATH)/mdk/gcc_startup_$(MCU_SUB_VARIANT).S

#******************************************************************************
# INCLUDE PATH
#******************************************************************************

# src
IPATH += $(SRC_PATH)
IPATH += $(SRC_PATH)/boards/$(BOARD)

IPATH += $(SRC_PATH)/cmsis/include
IPATH += $(SRC_PATH)/usb
IPATH += $(SRC_PATH)/boards
IPATH += $(TUSB_PATH)

# nrfx
IPATH += $(NRFX_PATH)
IPATH += $(NRFX_PATH)/mdk
IPATH += $(NRFX_PATH)/hal
IPATH += $(NRFX_PATH)/drivers/include
IPATH += $(NRFX_PATH)/drivers/src

IPATH += $(SDK11_PATH)/libraries/bootloader_dfu/hci_transport
IPATH += $(SDK11_PATH)/libraries/bootloader_dfu
IPATH += $(SDK11_PATH)/drivers_nrf/pstorage
IPATH += $(SDK11_PATH)/ble/common
IPATH += $(SDK11_PATH)/ble/ble_services/ble_dfu
IPATH += $(SDK11_PATH)/ble/ble_services/ble_dis

IPATH += $(SDK_PATH)/libraries/timer
IPATH += $(SDK_PATH)/libraries/scheduler
IPATH += $(SDK_PATH)/libraries/crc16
IPATH += $(SDK_PATH)/libraries/util
IPATH += $(SDK_PATH)/libraries/hci/config
IPATH += $(SDK_PATH)/libraries/uart
IPATH += $(SDK_PATH)/libraries/hci
IPATH += $(SDK_PATH)/drivers_nrf/delay

# Softdevice
IPATH += $(SD_API_PATH)/include
IPATH += $(SD_API_PATH)/include/nrf52

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
CFLAGS += -fno-builtin --short-enums -fstack-usage

# Defined Symbol (MACROS)
CFLAGS += -D__HEAP_SIZE=0
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DCONFIG_NFCT_PINS_AS_GPIOS
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DDFU_APP_DATA_RESERVED=7*4096
CFLAGS += $(MCU_FLAGS)

CFLAGS += -DUF2_VERSION='"$(GIT_VERSION) $(GIT_SUBMODULE_VERSIONS) $(SD_NAME) $(SD_VERSION)"'
CFLAGS += -DBLEDIS_FW_VERSION='"$(GIT_VERSION) $(SD_NAME) $(SD_VERSION)"'

_VER = $(subst ., ,$(word 1, $(subst -, ,$(GIT_VERSION))))
CFLAGS += -DMK_BOOTLOADER_VERSION='($(word 1,$(_VER)) << 16) + ($(word 2,$(_VER)) << 8) + $(word 3,$(_VER))'


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
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += $(MCU_FLAGS)

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
$(info ASMFLAGS $(ASMFLAGS))
$(info )
endif

.phony: all clean size flash sd erase

# default target to build
all: $(BUILD)/$(OUTPUT_FILENAME)-nosd.out size

#******************* Flash target *******************

check_defined = \
    $(strip $(foreach 1,$1, \
    $(call __check_defined,$1,$(strip $(value 2)))))
__check_defined = \
    $(if $(value $1),, \
    $(error Undefined make flag: $1$(if $2, ($2))))

# Flash the compiled
flash: $(BUILD)/$(OUTPUT_FILENAME)-nosd.hex
	@echo Flashing: $<
	$(NRFJPROG) --program $< --sectoranduicrerase -f nrf52 --reset

dfu-flash: $(BUILD)/$(MERGED_FNAME).zip
	@:$(call check_defined, SERIAL, example: SERIAL=/dev/ttyACM0)
	$(NRFUTIL) --verbose dfu serial --package $< -p $(SERIAL) -b 115200 --singlebank --touch 1200

sd:
	@echo Flashing: $(SD_HEX)
	$(NRFJPROG) --program $(SD_HEX) -f nrf52 --chiperase  --reset

erase:
	@echo Erasing chip
	$(NRFJPROG) --eraseall -f nrf52

#******************* Compile rules *******************

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
$(BUILD)/$(OUTPUT_FILENAME)-nosd.out: $(BUILD) $(OBJECTS)
	@echo LD $(OUTPUT_FILENAME)-nosd.out
	$(QUIET)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $@

size: $(BUILD)/$(OUTPUT_FILENAME)-nosd.out
	-@echo ''
	$(QUIET)$(SIZE) $<
	-@echo ''


#******************* Binary generator *******************
.phony: genhex genpkg

## Create binary .hex file from the .out file
genhex: $(BUILD)/$(OUTPUT_FILENAME)-nosd.hex

$(BUILD)/$(OUTPUT_FILENAME)-nosd.hex: $(BUILD)/$(OUTPUT_FILENAME)-nosd.out
	@echo CR $(OUTPUT_FILENAME)-nosd.hex
	$(QUIET)$(OBJCOPY) -O ihex $< $@

# merge bootloader and sd hex together
combinehex: $(BUILD)/$(MERGED_FNAME).hex

$(BUILD)/$(MERGED_FNAME).hex: $(BUILD)/$(OUTPUT_FILENAME)-nosd.hex
	@echo CR $(MERGED_FNAME).hex
	@mergehex -q -m $< $(SD_HEX) -o $@

## Create pkg file for bootloader+SD combo to use with DFU
genpkg: $(BUILD)/$(MERGED_FNAME).zip

$(BUILD)/$(MERGED_FNAME).zip: $(BUILD)/$(OUTPUT_FILENAME)-nosd.hex
	@$(NRFUTIL) dfu genpkg --dev-type 0x0052 --dev-revision $(DFU_DEV_REV) --bootloader $< --softdevice $(SD_HEX) $@
