#******************************************************************************
# CONFIGURE
# - SDK_PATH : path to SDK directory
# - SRC_PATH : path to src folder
#
# - SD_NAME  : e.g s132, s140
# - SD_VER1, SD_VER2, SD_VER3: SoftDevice version e.g 6.0.0
# - SD_HEX   : to bootloader hex binary
#******************************************************************************
SRC_PATH     = src

SDK_PATH     = lib/sdk/components
SDK11_PATH   = lib/sdk11/components
SD_PATH      = lib/softdevice/$(SD_FILENAME)

TUSB_PATH    = lib/tinyusb/src
NRFX_PATH    = lib/nrfx

SD_VER1      = 6
SD_VER2      = 1
SD_VER3      = 1

SD_VERSION   = $(SD_VER1).$(SD_VER2).$(SD_VER3)
SD_FILENAME  = $(SD_NAME)_nrf52_$(SD_VERSION)
SD_API_PATH  = $(SD_PATH)/$(SD_FILENAME)_API
SD_HEX       = $(SD_PATH)/$(SD_FILENAME)_softdevice.hex

LD_FILE      = $(SRC_PATH)/linker/$(SD_NAME)_v$(SD_VER1).ld

MERGED_FNAME = $(OUTPUT_FILENAME)_$(SD_NAME)_$(SD_VERSION)
RELEASE_DIR  = bin/$(BOARD)


MK_DIS_FIRMWARE = "$(SD_NAME) $(SD_VERSION)"

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
BOARD_LIST = $(sort $(subst .h,,$(subst src/boards/,,$(wildcard src/boards/*.h))))

NRF52832_BOARDLIST = feather_nrf52832
IS_52832 = $(filter $(BOARD),$(NRF52832_BOARDLIST))

ifeq ($(filter $(MAKECMDGOALS),all-board all-release help),)
  ifeq ($(BOARD),)
    $(info You must provide a BOARD parameter with 'BOARD=')
    $(info Supported boards are: $(BOARD_LIST))
    $(info Run 'make help' for usage)
    $(error BOARD not defined)
  else
    ifeq ($(filter $(BOARD),$(BOARD_LIST)),)
      $(error Invalid BOARD specified)
    endif
  endif
endif

BUILD = _build-$(BOARD)

ifneq ($(IS_52832),)
SD_NAME = s132
DFU_DEV_REV = 0xADAF
else
SD_NAME = s140
DFU_DEV_REV = 52840
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
C_SOURCE_FILES += $(NRFX_PATH)/hal/nrf_nvmc.c

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

ifneq ($(IS_52832),)

C_SOURCE_FILES += $(NRFX_PATH)/mdk/system_nrf52.c

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

# nrfx
C_SOURCE_FILES += $(NRFX_PATH)/mdk/system_nrf52840.c

# Tinyusb stack
C_SOURCE_FILES += $(TUSB_PATH)/portable/nordic/nrf5x/dcd_nrf5x.c
C_SOURCE_FILES += $(TUSB_PATH)/portable/nordic/nrf5x/hal_nrf5x.c
C_SOURCE_FILES += $(TUSB_PATH)/common/tusb_fifo.c
C_SOURCE_FILES += $(TUSB_PATH)/device/usbd.c
C_SOURCE_FILES += $(TUSB_PATH)/device/usbd_control.c
C_SOURCE_FILES += $(TUSB_PATH)/class/cdc/cdc_device.c
C_SOURCE_FILES += $(TUSB_PATH)/class/msc/msc_device.c
C_SOURCE_FILES += $(TUSB_PATH)/class/custom/custom_device.c
C_SOURCE_FILES += $(TUSB_PATH)/tusb.c

endif


#******************************************************************************
# Assembly Files
#******************************************************************************
ifneq ($(IS_52832),)
ASM_SOURCE_FILES  = $(NRFX_PATH)/mdk/gcc_startup_nrf52.S
else
ASM_SOURCE_FILES  = $(NRFX_PATH)/mdk/gcc_startup_nrf52840.S
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
CFLAGS += -DMK_BOOTLOADER_VERSION=0x0$(SD_VER1)0$(SD_VER2)0$(SD_VER3)UL

CFLAGS += -D__HEAP_SIZE=0
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DMK_DIS_FIRMWARE='$(MK_DIS_FIRMWARE)'
CFLAGS += -DDFU_APP_DATA_RESERVED=7*4096

CFLAGS += -DUF2_VERSION='"$(GIT_VERSION) $(GIT_SUBMODULE_VERSIONS) $(SD_NAME) $(SD_VERSION)"'
CFLAGS += -DBOARD_$(shell echo $(BOARD) | tr '[:lower:]' '[:upper:]')
CFLAGS += -DBOARD_HEADER_FILE='"boards/$(BOARD).h"'

ifneq ($(IS_52832),)
CFLAGS += -DNRF52
CFLAGS += -DNRF52832_XXAA
CFLAGS += -DS132
else
CFLAGS += -DNRF52840_XXAA
CFLAGS += -DS140
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

ifneq ($(IS_52832),)
ASMFLAGS += -DNRF52
ASMFLAGS += -DS132
else
ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += -DS140
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
$(info ASMFLAGS $(ASMFLAGS))
$(info )
endif

.phony: all clean size flash sd erase

# default target to build
all: $(BUILD)/$(OUTPUT_FILENAME)-nosd.out size

# Rule using BOARD_LIST, nl is newline
define nl


endef

_make_board = $(MAKE) -s -f $(MAKEFILE_LIST) -e BOARD=$1 $2 $(nl)
_make_all_board = $(foreach b,$(BOARD_LIST), $(call _make_board,$b,$1))

# build all the boards
all-board:
	$(call _make_all_board,clean all)

all-release:
	$(call _make_all_board,clean all release)

help:
	@echo To flash (with jlink) a pre-built binary with a specific version to a board
	@echo $$ make BOARD=feather_nrf52840_express VERSION=6.1.1r0 flash
	@echo
	@echo To flash (with dfu) a pre-built binary with a specific version to a board
	@echo $$ make BOARD=feather_nrf52840_express VERSION=6.1.1r0 SERIAL=/dev/ttyACM0 dfu0-flash
	@echo
	@echo To compile and build the current code for a board
	@echo $$ make BOARD=feather_nrf52840_express all
	@echo
	@echo To flash current code using jlink
	@echo $$ make BOARD=feather_nrf52840_express flash
	@echo
	@echo To flash current code using existing bootloader dfu
	@echo $$ make BOARD=feather_nrf52840_express SERIAL=/dev/ttyACM0 dfu-flash

#******************* Flash target *******************

check_defined = \
    $(strip $(foreach 1,$1, \
    $(call __check_defined,$1,$(strip $(value 2)))))
__check_defined = \
    $(if $(value $1),, \
    $(error Undefined make flag: $1$(if $2, ($2))))

ifeq ($(VERSION),)

# Flash the compiled
flash: $(BUILD)/$(OUTPUT_FILENAME)-nosd.hex
	@echo Flashing: $<
	$(NRFJPROG) --program $< --sectoranduicrerase -f nrf52 --reset

dfu-flash: $(BUILD)/$(MERGED_FNAME).zip
	@:$(call check_defined, SERIAL, example: SERIAL=/dev/ttyACM0)
	$(NRFUTIL) --verbose dfu serial --package $< -p $(SERIAL) -b 115200 --singlebank

else

ifeq ($(VERSION),latest)
VERSION_FPATH = $(RELEASE_DIR)/$(MERGED_FNAME)
else
VERSION_FPATH = bin/$(BOARD)/$(VERSION)/$(OUTPUT_FILENAME)_$(SD_NAME)_$(VERSION)
endif

# Flash specific version in binary release folder
flash:
	@echo Flashing: $(VERSION_FPATH).hex
	$(NRFJPROG) --program $(VERSION_FPATH).hex --chiperase -f nrf52 --reset

dfu-flash:
	@:$(call check_defined, SERIAL, example: SERIAL=/dev/ttyACM0)
	$(NRFUTIL) --verbose dfu serial --package $(VERSION_FPATH).zip -p $(SERIAL) -b 115200 --singlebank

endif

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
.phony: genhex genpkg release

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

# Create SD+bootloader combo with hex & dfu package at release folder
release: combinehex genpkg
	@echo CR $(RELEASE_DIR)/$(MERGED_FNAME).hex
	@echo CR $(RELEASE_DIR)/$(MERGED_FNAME).zip
	@mkdir -p $(RELEASE_DIR)
	@cp $(BUILD)/$(MERGED_FNAME).hex $(RELEASE_DIR)/$(MERGED_FNAME).hex
	@cp $(BUILD)/$(MERGED_FNAME).zip $(RELEASE_DIR)/$(MERGED_FNAME).zip
