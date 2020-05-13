#------------------------------------------------------------------------------
# CONFIGURE
# - SDK_PATH   : path to SDK directory
#
# - SD_NAME    : e.g s132, s140
# - SD_VERSION : SoftDevice version e.g 6.0.0
# - SD_HEX     : to bootloader hex binary
#------------------------------------------------------------------------------

SDK_PATH     = lib/sdk/components
SDK11_PATH   = lib/sdk11/components
TUSB_PATH    = lib/tinyusb/src
NRFX_PATH    = lib/nrfx
SD_PATH      = lib/softdevice/$(SD_FILENAME)

SD_VERSION   = 6.1.1
SD_FILENAME  = $(SD_NAME)_nrf52_$(SD_VERSION)
SD_HEX       = $(SD_PATH)/$(SD_FILENAME)_softdevice.hex

MBR_HEX			 = lib/softdevice/mbr/hex/mbr_nrf52_2.4.1_mbr.hex

# linker by MCU eg. nrf52840.ld
LD_FILE      = linker/$(MCU_SUB_VARIANT).ld

GIT_VERSION = $(shell git describe --dirty --always --tags)
GIT_SUBMODULE_VERSIONS = $(shell git submodule status | cut -d' ' -f3,4 | paste -s -d" " -)

# compiled file name
OUT_FILE = $(BOARD)_bootloader-$(GIT_VERSION)

# merged file = compiled + sd
MERGED_FILE = $(OUT_FILE)_$(SD_NAME)_$(SD_VERSION)

#------------------------------------------------------------------------------
# Tool configure
#------------------------------------------------------------------------------

# Toolchain commands
# Should be added to your PATH
CROSS_COMPILE ?= arm-none-eabi-
CC      = $(CROSS_COMPILE)gcc
AS      = $(CROSS_COMPILE)as
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE    = $(CROSS_COMPILE)size
GDB     = $(CROSS_COMPILE)gdb

NRFUTIL = adafruit-nrfutil
NRFJPROG = nrfjprog

MK = mkdir -p
RM = rm -rf

# auto-detect BMP on macOS, otherwise have to specify
BMP_PORT ?= $(shell ls -1 /dev/cu.usbmodem????????1 | head -1)
GDB_BMP = $(GDB) -ex 'target extended-remote $(BMP_PORT)' -ex 'monitor swdp_scan' -ex 'attach 1'

#---------------------------------
# Select the board to build
#---------------------------------
BOARD_LIST = $(sort $(subst src/boards/,,$(wildcard src/boards/*)))

ifeq ($(filter $(BOARD),$(BOARD_LIST)),)
  $(info You must provide a BOARD parameter with 'BOARD='. Supported boards are:)
  $(foreach b,$(BOARD_LIST),$(info - $(b)))
  $(error Invalid BOARD specified)
endif

# Build directory
BUILD = _build/build-$(BOARD)

# Board specific
-include src/boards/$(BOARD)/board.mk

# MCU_SUB_VARIANT can be nrf52 (nrf52832), nrf52833, nrf52840
ifeq ($(MCU_SUB_VARIANT),nrf52)
  SD_NAME = s132
  DFU_DEV_REV = 0xADAF
  CFLAGS += -DNRF52 -DNRF52832_XXAA -DS132
else ifeq ($(MCU_SUB_VARIANT),nrf52833)
  SD_NAME = s140
  DFU_DEV_REV = 52840
  CFLAGS += -DNRF52833_XXAA -DS140
else ifeq ($(MCU_SUB_VARIANT),nrf52840)
  SD_NAME = s140
  DFU_DEV_REV = 52840
  CFLAGS += -DNRF52840_XXAA -DS140
else
  $(error Sub Variant $(MCU_SUB_VARIANT) is unknown)
endif

#------------------------------------------------------------------------------
# SOURCE FILES
#------------------------------------------------------------------------------

# all files in src
C_SRC += $(wildcard src/*.c)

# all files in boards
C_SRC += $(wildcard src/boards/*.c)

# all sources files in specific board
C_SRC += $(wildcard src/boards/$(BOARD)/*.c)

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

# USB Application ( MSC + UF2 )
C_SRC += $(wildcard src/usb/*.c)
C_SRC += $(wildcard src/usb/uf2/*.c)

# TinyUSB stack
C_SRC += $(TUSB_PATH)/portable/nordic/nrf5x/dcd_nrf5x.c
C_SRC += $(TUSB_PATH)/common/tusb_fifo.c
C_SRC += $(TUSB_PATH)/device/usbd.c
C_SRC += $(TUSB_PATH)/device/usbd_control.c
C_SRC += $(TUSB_PATH)/class/cdc/cdc_device.c
C_SRC += $(TUSB_PATH)/class/msc/msc_device.c
C_SRC += $(TUSB_PATH)/tusb.c

endif

#------------------------------------------------------------------------------
# Assembly Files
#------------------------------------------------------------------------------
ASM_SRC = $(NRFX_PATH)/mdk/gcc_startup_$(MCU_SUB_VARIANT).S

#------------------------------------------------------------------------------
# INCLUDE PATH
#------------------------------------------------------------------------------

# src
IPATH += src
IPATH += src/boards
IPATH += src/boards/$(BOARD)
IPATH += src/cmsis/include
IPATH += src/usb
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
IPATH += $(SD_PATH)/$(SD_FILENAME)_API/include
IPATH += $(SD_PATH)/$(SD_FILENAME)_API/include/nrf52

#------------------------------------------------------------------------------
# Compiler Flags
#------------------------------------------------------------------------------

# Debug option use RTT for printf
ifeq ($(DEBUG), 1)
	RTT_SRC = lib/SEGGER_RTT
	
	CFLAGS += -ggdb -DCFG_DEBUG -DSEGGER_RTT_MODE_DEFAULT=SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL
	IPATH += $(RTT_SRC)/RTT
  C_SRC += $(RTT_SRC)/RTT/SEGGER_RTT.c
endif

#flags common to all targets
CFLAGS += \
	-mthumb \
	-mabi=aapcs \
	-mcpu=cortex-m4 \
	-mfloat-abi=hard \
	-mfpu=fpv4-sp-d16 \
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

# TinyUSB tusb_hal_nrf_power_event
CFLAGS += -Wno-cast-function-type

# Defined Symbol (MACROS)
CFLAGS += -D__HEAP_SIZE=0
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DCONFIG_NFCT_PINS_AS_GPIOS
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DDFU_APP_DATA_RESERVED=7*4096

CFLAGS += -DUF2_VERSION='"$(GIT_VERSION) $(GIT_SUBMODULE_VERSIONS)"'
CFLAGS += -DBLEDIS_FW_VERSION='"$(GIT_VERSION) $(SD_NAME) $(SD_VERSION)"'

_VER = $(subst ., ,$(word 1, $(subst -, ,$(GIT_VERSION))))
CFLAGS += -DMK_BOOTLOADER_VERSION='($(word 1,$(_VER)) << 16) + ($(word 2,$(_VER)) << 8) + $(word 3,$(_VER))'

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

# Verbose mode (V=). 0: default, 1: print out CFLAG, LDFLAG 2: print all compile command
ifeq ("$(V)","1")
$(info CFLAGS   $(CFLAGS))
$(info )
$(info LDFLAGS  $(LDFLAGS))
$(info )
$(info ASFLAGS $(ASFLAGS))
$(info )
endif

.PHONY: all clean flash dfu-flash sd gdbflash gdb

# default target to build
all: $(BUILD)/$(OUT_FILE).out $(BUILD)/$(OUT_FILE)-nosd.hex $(BUILD)/$(OUT_FILE)-nosd.uf2 $(BUILD)/$(MERGED_FILE).hex $(BUILD)/$(MERGED_FILE).zip

#------------------- Compile rules -------------------

# Create build directories
$(BUILD):
	@$(MK) $@

clean:
	@$(RM) $(BUILD)

# Create objects from C SRC files
$(BUILD)/%.o: %.c
	@echo CC $(notdir $<)
	@$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(BUILD)/%.o: %.S
	@echo AS $(notdir $<)
	@$(CC) -x assembler-with-cpp $(ASFLAGS) $(INC_PATHS) -c -o $@ $<

# Link
$(BUILD)/$(OUT_FILE).out: $(BUILD) $(OBJECTS)
	@echo LD $(notdir $@)
	@$(CC) -o $@ $(LDFLAGS) $(OBJECTS) -Wl,--start-group $(LIBS) -Wl,--end-group
	@$(SIZE) $@

#------------------- Binary generator -------------------

# Create hex file (no sd, no mbr)
$(BUILD)/$(OUT_FILE).hex: $(BUILD)/$(OUT_FILE).out
	@echo Create $(notdir $@)
	@$(OBJCOPY) -O ihex $< $@

# Hex file with mbr (still no SD)
$(BUILD)/$(OUT_FILE)-nosd.hex: $(BUILD)/$(OUT_FILE).hex
	@echo Create $(notdir $@)
	@python3 tools/hexmerge.py --overlap=replace -o $@ $< $(MBR_HEX)

# Bootolader only uf2
$(BUILD)/$(OUT_FILE)-nosd.uf2: $(BUILD)/$(OUT_FILE)-nosd.hex
	@echo Create $(notdir $@)
	@python3 lib/uf2/utils/uf2conv.py -f 0xd663823c -c -o $@ $^

# merge bootloader and sd hex together
$(BUILD)/$(MERGED_FILE).hex: $(BUILD)/$(OUT_FILE).hex
	@echo Create $(notdir $@)
	@python3 tools/hexmerge.py -o $@ $< $(SD_HEX)

# Create pkg zip file for bootloader+SD combo to use with DFU CDC
$(BUILD)/$(MERGED_FILE).zip: $(BUILD)/$(OUT_FILE).hex
	@$(NRFUTIL) dfu genpkg --dev-type 0x0052 --dev-revision $(DFU_DEV_REV) --bootloader $< --softdevice $(SD_HEX) $@

#------------------- Flash target -------------------

check_defined = \
    $(strip $(foreach 1,$1, \
    $(call __check_defined,$1,$(strip $(value 2)))))
__check_defined = \
    $(if $(value $1),, \
    $(error Undefined make flag: $1$(if $2, ($2))))

# Flash the compiled
flash: $(BUILD)/$(OUT_FILE)-nosd.hex
	@echo Flashing: $(notdir $<)
	$(NRFJPROG) --program $< --sectoranduicrerase -f nrf52 --reset

# dfu using CDC interface
dfu-flash: $(BUILD)/$(MERGED_FILE).zip
	@:$(call check_defined, SERIAL, example: SERIAL=/dev/ttyACM0)
	$(NRFUTIL) --verbose dfu serial --package $< -p $(SERIAL) -b 115200 --singlebank --touch 1200

erase:
	@echo Erasing flash
	$(NRFJPROG) -f nrf52 --eraseall

# flash SD only
sd:
	@echo Flashing: $(SD_HEX)
	$(NRFJPROG) --program $(SD_HEX) -f nrf52 --sectorerase --reset

# flash MBR only
mbr:
	@echo Flashing: $(MBR_HEX)
	$(NRFJPROG) --program $(MBR_HEX) -f nrf52 --sectorerase --reset

gdbflash: $(BUILD)/$(MERGED_FILE).hex
	@echo Flashing: $<
	@$(GDB_BMP) -nx --batch -ex 'load $<' -ex 'compare-sections' -ex 'kill'

gdb: $(BUILD)/$(OUT_FILE).out
	$(GDB_BMP) $<
