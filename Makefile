#------------------------------------------------------------------------------
# CONFIGURE
# - SDK_PATH   : path to SDK directory
#
# - SD_NAME    : e.g s132, s140
# - SD_VERSION : SoftDevice version e.g 6.0.0
# - SD_HEX     : to bootloader hex binary
#------------------------------------------------------------------------------

# local customization
-include Makefile.user

SDK_PATH     = lib/sdk/components
SDK11_PATH   = lib/sdk11/components
TUSB_PATH    = lib/tinyusb/src
NRFX_PATH    = lib/nrfx
SD_PATH      = lib/softdevice/$(SD_FILENAME)

# SD_VERSION can be overwritten by board.mk
ifndef SD_VERSION
SD_VERSION   = 6.1.1
endif

SD_FILENAME  = $(SD_NAME)_nrf52_$(SD_VERSION)
SD_HEX       = $(SD_PATH)/$(SD_FILENAME)_softdevice.hex

MBR_HEX			 = lib/softdevice/mbr/hex/mbr_nrf52_2.4.1_mbr.hex

# linker by MCU eg. nrf52840.ld
ifeq ($(DEBUG), 1)
  LD_FILE    = linker/$(MCU_SUB_VARIANT)_debug.ld
else
  LD_FILE    = linker/$(MCU_SUB_VARIANT).ld
endif

GIT_VERSION := $(shell git describe --dirty --always --tags)
GIT_SUBMODULE_VERSIONS := $(shell git submodule status | cut -d" " -f3,4 | paste -s -d" " -)

# compiled file name
OUT_NAME = $(BOARD)_bootloader-$(GIT_VERSION)

# merged file = compiled + sd
MERGED_FILE = $(OUT_NAME)_$(SD_NAME)_$(SD_VERSION)

#------------------------------------------------------------------------------
# Tool Configure
#------------------------------------------------------------------------------

# Toolchain commands
# Should be added to your PATH
CROSS_COMPILE ?= arm-none-eabi-
CC      = $(CROSS_COMPILE)gcc
AS      = $(CROSS_COMPILE)as
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE    = $(CROSS_COMPILE)size
GDB     = $(CROSS_COMPILE)gdb

# Set make directory command, Windows tries to create a directory named "-p" if that flag is there.
ifneq ($(OS), Windows_NT)
  MKDIR = mkdir -p
else
  MKDIR = mkdir
endif

RM = rm -rf
CP = cp

# Flasher utility options
NRFUTIL = adafruit-nrfutil
NRFJPROG = nrfjprog
FLASHER ?= nrfjprog
PYOCD ?= pyocd

# Flasher will default to nrfjprog,
# Check for pyocd, error on unexpected value.
ifeq ($(FLASHER),nrfjprog)
  FLASH_CMD = $(NRFJPROG) --program $1 --sectoranduicrerase -f nrf52 --reset
  FLASH_NOUICR_CMD = $(NRFJPROG) --program $1 -f nrf52 --sectorerase --reset
  FLASH_ERASE_CMD = $(NRFJPROG) -f nrf52 --eraseall
else ifeq ($(FLASHER),pyocd)
  FLASH_CMD = $(PYOCD) flash -t $(MCU_SUB_VARIANT) $1
  FLASH_NOUICR_CMD = $(PYOCD) flash -t $(MCU_SUB_VARIANT) $1
  FLASH_ERASE_CMD = $(PYOCD) erase -t $(MCU_SUB_VARIANT) --chip
else
  $(error Unsupported flash utility: "$(FLASHER)")
endif

# auto-detect BMP on macOS, otherwise have to specify
BMP_PORT ?= $(shell ls -1 /dev/cu.usbmodem????????1 | head -1)
GDB_BMP = $(GDB) -ex 'target extended-remote $(BMP_PORT)' -ex 'monitor swdp_scan' -ex 'attach 1'

#---------------------------------
# Select the board to build
#---------------------------------
# Note: whitespace is not allowed in the filenames... it WILL break this part of the script
BOARD_LIST = $(sort $(filter-out boards.h boards.c,$(notdir $(wildcard src/boards/*))))

ifeq ($(filter $(BOARD),$(BOARD_LIST)),)
  $(info You must provide a BOARD parameter with 'BOARD='. Supported boards are:)
  $(foreach b,$(BOARD_LIST),$(info - $(b)))
  $(error Invalid BOARD specified)
endif

# Build directory
BUILD = _build/build-$(BOARD)
BIN = _bin/$(BOARD)

# Board specific
-include src/boards/$(BOARD)/board.mk

# MCU_SUB_VARIANT can be nrf52 (nrf52832), nrf52833, nrf52840
ifeq ($(MCU_SUB_VARIANT),nrf52)
  SD_NAME = s132
  DFU_DEV_REV = 0xADAF
  CFLAGS += -DNRF52 -DNRF52832_XXAA -DS132
  DFU_APP_DATA_RESERVED=7*4096
else ifeq ($(MCU_SUB_VARIANT),nrf52833)
  SD_NAME = s140
  DFU_DEV_REV = 52833
  CFLAGS += -DNRF52833_XXAA -DS140
  DFU_APP_DATA_RESERVED=7*4096
else ifeq ($(MCU_SUB_VARIANT),nrf52840)
  SD_NAME = s140
  DFU_DEV_REV = 52840
  CFLAGS += -DNRF52840_XXAA -DS140
  DFU_APP_DATA_RESERVED=10*4096
else
  $(error Sub Variant $(MCU_SUB_VARIANT) is unknown)
endif

#------------------------------------------------------------------------------
# SOURCE FILES
#------------------------------------------------------------------------------

# all files in src
C_SRC += \
  src/dfu_ble_svc.c \
  src/dfu_init.c \
  src/flash_nrf5x.c \
  src/main.c \

# all files in boards
C_SRC += src/boards/boards.c

# nrfx
C_SRC += $(NRFX_PATH)/drivers/src/nrfx_power.c
C_SRC += $(NRFX_PATH)/drivers/src/nrfx_nvmc.c
C_SRC += $(NRFX_PATH)/mdk/system_$(MCU_SUB_VARIANT).c

# SDK 11 files: serial + OTA DFU
C_SRC += $(SDK11_PATH)/libraries/bootloader_dfu/bootloader.c
C_SRC += $(SDK11_PATH)/libraries/bootloader_dfu/bootloader_settings.c
C_SRC += $(SDK11_PATH)/libraries/bootloader_dfu/bootloader_util.c
C_SRC += $(SDK11_PATH)/libraries/bootloader_dfu/dfu_transport_serial.c
C_SRC += $(SDK11_PATH)/libraries/bootloader_dfu/dfu_transport_ble.c
C_SRC += $(SDK11_PATH)/libraries/bootloader_dfu/dfu_single_bank.c
C_SRC += $(SDK11_PATH)/ble/ble_services/ble_dfu/ble_dfu.c
C_SRC += $(SDK11_PATH)/ble/ble_services/ble_dis/ble_dis.c
C_SRC += $(SDK11_PATH)/drivers_nrf/pstorage/pstorage_raw.c

# Latest SDK files: peripheral drivers
C_SRC += $(SDK_PATH)/libraries/timer/app_timer.c
C_SRC += $(SDK_PATH)/libraries/scheduler/app_scheduler.c
C_SRC += $(SDK_PATH)/libraries/util/app_error.c
C_SRC += $(SDK_PATH)/libraries/util/app_util_platform.c
C_SRC += $(SDK_PATH)/libraries/crc16/crc16.c
C_SRC += $(SDK_PATH)/libraries/hci/hci_mem_pool.c
C_SRC += $(SDK_PATH)/libraries/hci/hci_slip.c
C_SRC += $(SDK_PATH)/libraries/hci/hci_transport.c
C_SRC += $(SDK_PATH)/libraries/util/nrf_assert.c

# UART or USB Serial
ifeq ($(MCU_SUB_VARIANT),nrf52)

C_SRC += $(SDK_PATH)/libraries/uart/app_uart.c
C_SRC += $(SDK_PATH)/drivers_nrf/uart/nrf_drv_uart.c
C_SRC += $(SDK_PATH)/drivers_nrf/common/nrf_drv_common.c

IPATH += $(SDK11_PATH)/libraries/util
IPATH += $(SDK_PATH)/drivers_nrf/common
IPATH += $(SDK_PATH)/drivers_nrf/uart

else

# pinconfig is required for 840 for CF2
C_SRC += src/boards/$(BOARD)/pinconfig.c

# USB Application ( MSC + UF2 )
C_SRC += \
	src/usb/msc_uf2.c \
	src/usb/usb_desc.c \
	src/usb/usb.c \
	src/usb/uf2/ghostfat.c

# TinyUSB stack
C_SRC += \
	$(TUSB_PATH)/portable/nordic/nrf5x/dcd_nrf5x.c \
	$(TUSB_PATH)/common/tusb_fifo.c \
	$(TUSB_PATH)/device/usbd.c \
	$(TUSB_PATH)/device/usbd_control.c \
	$(TUSB_PATH)/class/cdc/cdc_device.c \
	$(TUSB_PATH)/class/msc/msc_device.c \
	$(TUSB_PATH)/tusb.c

endif

#------------------------------------------------------------------------------
# Assembly Files
#------------------------------------------------------------------------------
ASM_SRC = $(NRFX_PATH)/mdk/gcc_startup_$(MCU_SUB_VARIANT).S

#------------------------------------------------------------------------------
# INCLUDE PATH
#------------------------------------------------------------------------------

# src
IPATH += \
  src \
  src/boards \
  src/boards/$(BOARD) \
  src/cmsis/include \
  src/usb \
  $(TUSB_PATH)

# nrfx
IPATH += \
  $(NRFX_PATH) \
  $(NRFX_PATH)/mdk \
  $(NRFX_PATH)/hal \
  $(NRFX_PATH)/drivers/include \
  $(NRFX_PATH)/drivers/src

# sdk11 for cdc/ble dfu
IPATH += \
  $(SDK11_PATH)/libraries/bootloader_dfu/hci_transport \
  $(SDK11_PATH)/libraries/bootloader_dfu \
  $(SDK11_PATH)/drivers_nrf/pstorage \
  $(SDK11_PATH)/ble/common \
  $(SDK11_PATH)/ble/ble_services/ble_dfu \
  $(SDK11_PATH)/ble/ble_services/ble_dis

# later sdk with updated drivers
IPATH += \
  $(SDK_PATH)/libraries/timer \
  $(SDK_PATH)/libraries/scheduler \
  $(SDK_PATH)/libraries/crc16 \
  $(SDK_PATH)/libraries/util \
  $(SDK_PATH)/libraries/hci/config \
  $(SDK_PATH)/libraries/uart \
  $(SDK_PATH)/libraries/hci \
  $(SDK_PATH)/drivers_nrf/delay

# SoftDevice
IPATH += \
  $(SD_PATH)/$(SD_FILENAME)_API/include \
  $(SD_PATH)/$(SD_FILENAME)_API/include/nrf52

#------------------------------------------------------------------------------
# Compiler Flags
#------------------------------------------------------------------------------

#flags common to all targets
CFLAGS += \
	-mthumb \
	-mabi=aapcs \
	-mcpu=cortex-m4 \
	-mfloat-abi=hard \
	-mfpu=fpv4-sp-d16 \
	-ggdb \
	-Os \
	-ffunction-sections \
	-fdata-sections \
	-fno-builtin \
	-fshort-enums \
	-fstack-usage \
	-fno-strict-aliasing \
	-Wall \
	-Wextra \
	-Werror \
	-Wfatal-errors \
	-Werror-implicit-function-declaration \
	-Wfloat-equal \
	-Wundef \
	-Wshadow \
	-Wwrite-strings \
	-Wsign-compare \
	-Wmissing-format-attribute \
	-Wno-endif-labels \
	-Wunreachable-code

# Suppress warning caused by SDK
CFLAGS += -Wno-unused-parameter -Wno-expansion-to-defined

# Nordic Softdevice SDK header files contains inline assembler that has
# broken constraints. As a result the IPA-modref pass, introduced in gcc-11,
# is able to "prove" that arguments to wrapper functions generated with
# the SVCALL() macro are unused and, as a result, the optimizer will remove
# code within the callers that sets up these arguments (which results in
# a broken bootloader). The broken headers come from Nordic-supplied zip
# files and are not trivial to patch so, for now, we'll simply disable the
# new gcc-11 inter-procedural optimizations.
ifeq (,$(findstring unrecognized,$(shell $(CC) $(CFLAGS) -fno-ipa-modref 2>&1)))
CFLAGS += -fno-ipa-modref
endif

# Defined Symbol (MACROS)
CFLAGS += -D__HEAP_SIZE=0
CFLAGS += -DCONFIG_GPIO_AS_PINRESET

# Skip defining CONFIG_NFCT_PINS_AS_GPIOS if the device uses the NFCT.
ifneq ($(USE_NFCT),yes)
  CFLAGS += -DCONFIG_NFCT_PINS_AS_GPIOS
endif

CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DUF2_VERSION='"$(GIT_VERSION) $(GIT_SUBMODULE_VERSIONS)"'
CFLAGS += -DBLEDIS_FW_VERSION='"$(GIT_VERSION) $(SD_NAME) $(SD_VERSION)"'

_VER = $(subst ., ,$(word 1, $(subst -, ,$(GIT_VERSION))))
CFLAGS += -DMK_BOOTLOADER_VERSION='($(word 1,$(_VER)) << 16) + ($(word 2,$(_VER)) << 8) + $(word 3,$(_VER))'

# Debug option use RTT for printf
ifeq ($(DEBUG), 1)
  CFLAGS += -DCFG_DEBUG -DSEGGER_RTT_MODE_DEFAULT=SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL
  RTT_SRC = lib/SEGGER_RTT
  IPATH += $(RTT_SRC)/RTT
  C_SRC += $(RTT_SRC)/RTT/SEGGER_RTT.c
  DFU_APP_DATA_RESERVED = 0

	# expand bootloader address to 28KB of reserved app
  ifeq ($(MCU_SUB_VARIANT),nrf52840)
    CFLAGS += -DBOOTLOADER_REGION_START=0xED000
  else
    CFLAGS += -DBOOTLOADER_REGION_START=0x6D000
  endif
endif

CFLAGS += -DDFU_APP_DATA_RESERVED=$(DFU_APP_DATA_RESERVED)

# https://gcc.gnu.org/bugzilla/show_bug.cgi?id=105523
ifneq ($(findstring 12.,$(shell $(CC) --version 2>/dev/null)),)
	CFLAGS += --param=min-pagesize=0
endif

#------------------------------------------------------------------------------
# Linker Flags
#------------------------------------------------------------------------------

LDFLAGS += \
	$(CFLAGS) \
	-Wl,-L,linker -Wl,-T,$(LD_FILE) \
	-Wl,-Map=$@.map -Wl,-cref -Wl,-gc-sections \
	-specs=nosys.specs -specs=nano.specs

LIBS += -lm -lc

#------------------------------------------------------------------------------
# Assembler flags
#------------------------------------------------------------------------------

ASFLAGS += $(CFLAGS)

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

C_SOURCE_FILE_NAMES = $(notdir $(C_SRC))
C_PATHS = $(call remduplicates, $(dir $(C_SRC) ) )
C_OBJECTS = $(addprefix $(BUILD)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SRC))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SRC) ))
ASM_OBJECTS = $(addprefix $(BUILD)/, $(ASM_SOURCE_FILE_NAMES:.S=.o) )

vpath %.c $(C_PATHS)
vpath %.S $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

INC_PATHS = $(addprefix -I,$(IPATH))

#------------------------------------------------------------------------------
# BUILD TARGETS
#------------------------------------------------------------------------------

.PHONY: all clean flash flash-dfu flash-sd flash-mbr dfu-flash sd mbr gdbflash gdb

# default target to build
all: $(BUILD)/$(OUT_NAME).out $(BUILD)/$(OUT_NAME)_nosd.hex $(BUILD)/update-$(OUT_NAME)_nosd.uf2 $(BUILD)/$(MERGED_FILE).hex $(BUILD)/$(MERGED_FILE).zip

# Print out the value of a make variable.
# https://stackoverflow.com/questions/16467718/how-to-print-out-a-variable-in-makefile
print-%:
	@echo $* = $($*)

#------------------- Compile rules -------------------

# Create build directories
$(BUILD):
	@$(MKDIR) "$@"

clean:
	@$(RM) $(BUILD)
	@$(RM) $(BIN)

# linkermap must be install previously at https://github.com/hathach/linkermap
linkermap: $(BUILD)/$(OUT_NAME).out
	@linkermap -v $<.map

# Create objects from C SRC files
$(BUILD)/%.o: %.c
	@echo CC $(notdir $<)
	@$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(BUILD)/%.o: %.S
	@echo AS $(notdir $<)
	@$(CC) -x assembler-with-cpp $(ASFLAGS) $(INC_PATHS) -c -o $@ $<

# Link
$(BUILD)/$(OUT_NAME).out: $(BUILD) $(OBJECTS)
	@echo LD $(notdir $@)
	@$(CC) -o $@ $(LDFLAGS) $(OBJECTS) -Wl,--start-group $(LIBS) -Wl,--end-group
	@$(SIZE) $@

#------------------- Binary generator -------------------

# Create hex file (no sd, no mbr)
$(BUILD)/$(OUT_NAME).hex: $(BUILD)/$(OUT_NAME).out
	@echo Create $(notdir $@)
	@$(OBJCOPY) -O ihex $< $@

# Hex file with mbr (still no SD)
$(BUILD)/$(OUT_NAME)_nosd.hex: $(BUILD)/$(OUT_NAME).hex
	@echo Create $(notdir $@)
	@python3 tools/hexmerge.py --overlap=replace -o $@ $< $(MBR_HEX)

# Bootolader self-update uf2
$(BUILD)/update-$(OUT_NAME)_nosd.uf2: $(BUILD)/$(OUT_NAME)_nosd.hex
	@echo Create $(notdir $@)
	@python3 lib/uf2/utils/uf2conv.py -f 0xd663823c -c -o $@ $^

# merge bootloader and sd hex together
$(BUILD)/$(MERGED_FILE).hex: $(BUILD)/$(OUT_NAME).hex
	@echo Create $(notdir $@)
	@python3 tools/hexmerge.py -o $@ $< $(SD_HEX)

# Create pkg zip file for bootloader+SD combo to use with DFU CDC
$(BUILD)/$(MERGED_FILE).zip: $(BUILD)/$(OUT_NAME).hex
	@$(NRFUTIL) dfu genpkg --dev-type 0x0052 --dev-revision $(DFU_DEV_REV) --bootloader $< --softdevice $(SD_HEX) $@

#-------------- Artifacts --------------
$(BIN):
	@$(MKDIR) -p $@

copy-artifact: $(BIN)
	@$(CP) $(BUILD)/update-$(OUT_NAME)_nosd.uf2 $(BIN)
	@$(CP) $(BUILD)/$(MERGED_FILE).hex $(BIN)
	@$(CP) $(BUILD)/$(MERGED_FILE).zip $(BIN)

#--------------------------------------
# Flash Target
#--------------------------------------

check_defined = \
    $(strip $(foreach 1,$1, \
    $(call __check_defined,$1,$(strip $(value 2)))))
__check_defined = \
    $(if $(value $1),, \
    $(error Undefined make flag: $1$(if $2, ($2))))

# erase chip
erase:
	@echo Erasing flash
	$(call FLASH_ERASE_CMD)

# Flash the compiled
flash: $(BUILD)/$(OUT_NAME)_nosd.hex
	@echo Flashing: $(notdir $<)
	$(call FLASH_CMD,$<)

# flash SD only
sd: flash-sd
flash-sd:
	@echo Flashing: $(SD_HEX)
	$(call FLASH_NOUICR_CMD,$(SD_HEX))

# flash MBR only
mbr: flash-mbr
flash-mbr:
	@echo Flashing: $(MBR_HEX)
	$(call FLASH_NOUICR_CMD,$(MBR_HEX))

# dfu with adafruit-nrfutil using CDC interface
dfu-flash: flash-dfu
flash-dfu: $(BUILD)/$(MERGED_FILE).zip
	@:$(call check_defined, SERIAL, example: SERIAL=/dev/ttyACM0)
	$(NRFUTIL) --verbose dfu serial --package $< -p $(SERIAL) -b 115200 --singlebank --touch 1200
	
# flash skip crc magic ( app valid = 0x0001, crc = 0x0000 )
#flash-skip-crc:
# nrfjprog --memwr $(BOOT_SETTING_ADDR) --val 0x00000001 -f nrf52
#	nrfjprog --memwr 0xFF000 --val 0x00000001 -f nrf52
#	nrfjprog --memwr 0x7F000 --val 0x00000001 -f nrf52

#------------------- Debugging -------------------

gdbflash: $(BUILD)/$(MERGED_FILE).hex
	@echo Flashing: $<
	@$(GDB_BMP) -nx --batch -ex 'load $<' -ex 'compare-sections' -ex 'kill'

gdb: $(BUILD)/$(OUT_NAME).out
	$(GDB_BMP) $<
