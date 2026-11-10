#------------------------------------------------------------------------------
# Wrapper over the CMake build (CMakeLists.txt): configures $(BUILD) with CMake, then builds through it.
#
# - BOARD              : board to build, a directory under src/boards
# - BUILD              : build directory, default _build/build-$(BOARD)
# - CMAKE_GENERATOR    : CMake generator, default Unix Makefiles
# - BIN                : copy-artifact's directory, default _bin/$(BOARD)
# - DEBUG              : 1 for a debug build (RTT, *_debug.ld linker script)
# - SD_NAME            : e.g s132, s140
# - SD_VERSION         : SoftDevice version e.g 6.0.0
# - SD_HEX             : SoftDevice hex merged with the bootloader and written by flash-sd
# - SIGNED_FW          : if bootloader will ONLY accept signed firmware
# - SIGNED_FW_QX       : Qx for signed firmware verification
# - SIGNED_FW_QY       : Qy for signed firmware verification
# - DUALBANK_FW        : If bootloader will implement a dual bank feature to allow autorecover from failed
# - FORCE_UF2          : if SIGNED_FW is 1, will force to include UF2 support (UNSECURE, UF2 does NOT validate signature!)
# - DEFAULT_TO_OTA_DFU : if entering DFU, by default enter OTA DFU instead of Serial DFU
# - USE_NFCT           : yes if the board uses the NFC pins as NFC, no if as GPIO (default from board.cmake)
# - ANT_LICENSE_KEY    : ANT license key, with SD_NAME=s340
# - TRACE_ETM          : 1 to set up ETM trace
# - CROSS_COMPILE      : toolchain prefix, default arm-none-eabi-
# - PYTHON             : Python 3 interpreter, default python3; also runs the artifact scripts
# - NRFUTIL            : adafruit-nrfutil command
# - FLASHER            : nrfjprog (default) or pyocd
# - SERIAL             : serial port for flash-dfu e.g /dev/ttyACM0
#------------------------------------------------------------------------------

THIS_MAKEFILE := $(lastword $(MAKEFILE_LIST))

PYTHON = python3

# local customization
-include Makefile.user

BOARD_LIST := $(sort $(patsubst src/boards/%/board.cmake,%,$(wildcard src/boards/*/board.cmake)))

ifeq ($(BOARD),)
  $(info You must provide a BOARD parameter with 'BOARD=')
  $(info Supported boards are:)
  $(foreach b,$(BOARD_LIST),$(info - $(b)))
  $(error BOARD not defined)
else ifeq ($(filter $(BOARD),$(BOARD_LIST)),)
  $(info Unknown board '$(BOARD)'. Supported boards are:)
  $(foreach b,$(BOARD_LIST),$(info - $(b)))
  $(error Invalid BOARD specified)
endif

ifneq ($(filter flash-dfu dfu-flash,$(MAKECMDGOALS)),)
  ifeq ($(SERIAL),)
    $(error Undefined make flag: SERIAL (example: SERIAL=/dev/ttyACM0))
  endif
endif

# Build directory
BUILD = _build/build-$(BOARD)
BIN = _bin/$(BOARD)

CROSS_COMPILE ?= arm-none-eabi-
NRFUTIL ?= adafruit-nrfutil

# Each configure starts from an empty CMake cache (also what lets CROSS_COMPILE change the compiler), so only this
# invocation's variables apply. Flags are on only for 1 and USE_NFCT only for yes, as the make build had them.
flag = -D$(1)=$(if $(filter 1,$($(1))),ON,OFF)
optional = $(if $($(1)),"-D$(1)=$($(1))")
CMAKE_GENERATOR ?= Unix Makefiles
CMAKE_ARGS = -G "$(CMAKE_GENERATOR)" -DBOARD=$(BOARD) "-DCROSS_COMPILE=$(CROSS_COMPILE)" "-DNRFUTIL:STRING=$(NRFUTIL)" \
  -DCMAKE_BUILD_TYPE=$(if $(filter 1,$(DEBUG)),Debug,MinSizeRel) \
  $(foreach v,SIGNED_FW DUALBANK_FW FORCE_UF2 DEFAULT_TO_OTA_DFU,$(call flag,$(v))) \
  $(if $(USE_NFCT),-DUSE_NFCT=$(if $(filter yes,$(USE_NFCT)),ON,OFF)) \
  $(foreach v,SD_NAME SD_VERSION SD_HEX SIGNED_FW_QX SIGNED_FW_QY ANT_LICENSE_KEY TRACE_ETM,$(call optional,$(v)))

# make's DEBUG=1 build kept -Os; CMake's Debug type alone compiles with -O0
CMAKE_ARGS += -DCMAKE_C_FLAGS_DEBUG=-Os -DCMAKE_ASM_FLAGS_DEBUG=-Os

# Extra flags (command line, environment or Makefile.user) reach a fresh configure through the environment, where CMake
# adds them to the toolchain's; a -DCMAKE_C_FLAGS would replace those. As in the make build, CFLAGS also reach the
# assembler and the link. CMake compiles in the build directory, the make build here: tools/abs_flags.py makes their
# checkout-relative paths absolute when configure runs. Left recursive, the flags take Makefile.user's references to
# later variables and target-specific values.
export CFLAGS LDFLAGS
export ASMFLAGS = $(ASFLAGS) $(CFLAGS)

# Flasher utility options
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
  FLASH_CMD = $(PYOCD) flash -t $(MCU_VARIANT) $1
  FLASH_NOUICR_CMD = $(PYOCD) flash -t $(MCU_VARIANT) $1
  FLASH_ERASE_CMD = $(PYOCD) erase -t $(MCU_VARIANT) --chip
else
  $(error Unsupported flash utility: "$(FLASHER)")
endif

# auto-detect BMP on macOS, otherwise have to specify
GDB = $(CROSS_COMPILE)gdb
BMP_PORT ?= $(shell ls -1 /dev/cu.usbmodem????????1 | head -1)
GDB_BMP = $(GDB) -ex 'target extended-remote $(BMP_PORT)' -ex 'monitor swdp_scan' -ex 'attach 1'

# Recipes that need the artifact names, the SoftDevice hex or the MCU run in a sub-make that reads them from a file
# CMake writes: $(BUILD)/make_vars.mk when configuring the build, $(BUILD)/board_vars.mk from cmake/board_vars.cmake
# without configuring it.
SUBMAKE = $(MAKE) --no-print-directory -f $(THIS_MAKEFILE) -f

# one configure and one build at a time in $(BUILD), whatever -j and the goals given
.NOTPARALLEL:

.PHONY: all configure clean flash flash-dfu dfu-flash flash-sd sd flash-mbr mbr flash-uf2 erase copy-artifact \
  linkermap gdbflash gdb

all: configure
	cmake --build "$(BUILD)"

configure:
	@cmake -E rm -f "$(BUILD)/CMakeCache.txt"
	$(PYTHON) tools/abs_flags.py cmake -S . -B "$(BUILD)" $(CMAKE_ARGS)

clean:
	@cmake -E rm -rf "$(BUILD)" "$(BIN)"

# each builds only the artifact it uses
flash: CMAKE_TARGET = nosd-hex
flash-dfu: CMAKE_TARGET = dfu-zip
gdbflash: CMAKE_TARGET = merged-hex
gdb linkermap: CMAKE_TARGET = bootloader
flash flash-dfu gdbflash gdb linkermap: configure
	cmake --build "$(BUILD)" --target $(CMAKE_TARGET)
	@$(SUBMAKE) "$(BUILD)/make_vars.mk" _$@

# need no compiler or submodule, as before the wrapper
flash-sd flash-mbr erase:
	@cmake -DBOARD=$(BOARD) $(foreach v,SD_NAME SD_VERSION SD_HEX,$(call optional,$(v))) \
	  "-DOUT=$(BUILD)/board_vars.mk" -P cmake/board_vars.cmake
	@$(SUBMAKE) "$(BUILD)/board_vars.mk" _$@

# copies what the last build produced
copy-artifact:
	@test -f "$(BUILD)/make_vars.mk" || { echo "$(BUILD) holds no build: run make all first" >&2; exit 1; }
	@$(SUBMAKE) "$(BUILD)/make_vars.mk" _$@

sd: flash-sd
mbr: flash-mbr
dfu-flash: flash-dfu

# deploys bootloader_mbr.uf2, the same bytes as update-*_nosd.uf2
flash-uf2: configure
	cmake --build "$(BUILD)" --target flash-uf2

#------------------- recipes run by the sub-make -------------------

_flash:
	@echo Flashing: $(notdir $(lastword $(NOSD_HEX)))
	$(call FLASH_CMD,"$(NOSD_HEX)")

_flash-sd:
	@echo Flashing: $(SD_HEX)
	$(call FLASH_NOUICR_CMD,"$(SD_HEX)")

_flash-mbr:
	@echo Flashing: $(MBR_HEX)
	$(call FLASH_NOUICR_CMD,"$(MBR_HEX)")

_erase:
	@echo Erasing flash
	$(FLASH_ERASE_CMD)

# dfu with adafruit-nrfutil using CDC interface
_flash-dfu:
	$(NRFUTIL) --verbose dfu serial --package "$(MERGED_ZIP)" -p $(SERIAL) -b 115200 --singlebank --touch 1200

_copy-artifact:
	@cmake -E make_directory "$(BIN)"
	@cmake -E copy "$(UPDATE_UF2)" "$(MERGED_HEX)" "$(MERGED_ZIP)" "$(BIN)"

# linkermap must be install previously at https://github.com/hathach/linkermap
_linkermap:
	@linkermap -v "$(BL_ELF).map"

_gdbflash:
	@echo Flashing: $(MERGED_HEX)
	@$(GDB_BMP) -nx --batch -ex 'load $(MERGED_HEX)' -ex 'compare-sections' -ex 'kill'

_gdb:
	$(GDB_BMP) "$(BL_ELF)"
