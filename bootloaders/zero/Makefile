# Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.
# Copyright (c) 2015 Arduino LLC.  All right reserved.
# Copyright (c) 2017 Justin Mattair.  All right reserved.
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

# Begin Config ###############################

# -----------------------------------------------------------------------------
# SD Card support. This can also be set in the relevant board_definitions file.
# SDCARD_DISABLED, SDCARD_ENABLED
SDCARD?=SDCARD_DISABLED

# -----------------------------------------------------------------------------
# Boards definitions:
# Xeno, Xeno_Mini, MT_D21E_rev_A, MT_D21E_rev_B, MT_D11
# arduino_zero, arduino_mkrzero, arduino_mkr1000, arduino_mkrfox1200, genuino_mkr1000, genuino_zero, arduino_m0, arduino_m0_pro
# Generic_x21E, Generic_x21G, Generic_x21J, Generic_D11D14AM, Generic_D11D14AS, Generic_D11C14A
BOARD_ID?=Xeno_Mini

# -----------------------------------------------------------------------------
# MCU definitions:
# SAMD21J: SAMD21J18A, SAMD21J17A, SAMD21J16A, SAMD21J15A
# SAMD21G: SAMD21G18A, SAMD21G17A, SAMD21G16A, SAMD21G15A
# SAMD21E: SAMD21E18A, SAMD21E17A, SAMD21E16A, SAMD21E15A
# SAML21J: SAML21J18B, SAML21J17B, SAML21J16B
# SAML21G: SAML21G18B, SAML21G17B, SAML21G16B
# SAML21E: SAML21E18B, SAML21E17B, SAML21E16B, SAML21E15B
# SAMC21J: SAMC21J18A, SAMC21J17A, SAMC21J16A, SAMC21J15A
# SAMC21G: SAMC21G18A, SAMC21G17A, SAMC21G16A, SAMC21G15A
# SAMC21E: SAMC21E18A, SAMC21E17A, SAMC21E16A, SAMC21E15A
# SAMD11:  SAMD11D14AM, SAMD11C14A, SAMD11D14AS
# SAMD51G: SAMD51G18A, SAMD51G19A
# SAMD51J: SAMD51J18A, SAMD51J19A, SAMD51J20A
# SAMD51N: SAMD51N19A, SAMD51N20A
# SAMD51P: SAMD51P19A, SAMD51P20A
MCU?=SAMD51G19A

# Note that the makefile can be invoked with values that override the above settings.
# For example: BOARD_ID=Generic_D11C14A MCU=SAMD11C14A SDCARD=SDCARD_DISABLED make

# End Config ###############################

ifeq ($(findstring ENABLED,$(SDCARD)),ENABLED)
NAME=sam_ba_sdcard_$(BOARD_ID)_$(MCU)
else
NAME=sam_ba_$(BOARD_ID)_$(MCU)
endif

ifeq ($(findstring 20,$(MCU)),20)
FLASHSIZE=1MB
endif
ifeq ($(findstring 19,$(MCU)),19)
FLASHSIZE=512KB
endif
ifeq ($(findstring 18,$(MCU)),18)
FLASHSIZE=256KB
endif
ifeq ($(findstring 17,$(MCU)),17)
FLASHSIZE=128KB
endif
ifeq ($(findstring 16,$(MCU)),16)
FLASHSIZE=64KB
endif
ifeq ($(findstring 15,$(MCU)),15)
FLASHSIZE=32KB
endif
ifeq ($(findstring 14,$(MCU)),14)
FLASHSIZE=16KB
endif

ifeq ($(findstring D51,$(MCU)),D51)
LINKER_SCRIPT?=bootloader_sam_m4f_$(FLASHSIZE).ld
MCPU=cortex-m4
else
LINKER_SCRIPT?=bootloader_sam_m0p_$(FLASHSIZE).ld
MCPU=cortex-m0plus
endif

# CORE_VENDOR?=arduino
CORE_VENDOR?=MattairTech_Arduino

# -----------------------------------------------------------------------------
# Paths
ifeq ($(OS),Windows_NT)
  # Are we using mingw/msys/msys2/cygwin?
  ifeq ($(TERM),xterm)
    T=$(shell cygpath -u $(LOCALAPPDATA))
    MODULE_PATH?=$(T)/Arduino15/packages/$(CORE_VENDOR)
    MODULE_PATH_ARDUINO?=$(T)/Arduino15/packages/arduino
    RM=rm
    SEP=/
  else
    MODULE_PATH?=$(LOCALAPPDATA)/Arduino15/packages/$(CORE_VENDOR)
    MODULE_PATH_ARDUINO?=$(LOCALAPPDATA)/Arduino15/packages/arduino
    RM=del
    SEP=\\
  endif
else
  UNAME_S := $(shell uname -s)

  ifeq ($(UNAME_S),Linux)
    MODULE_PATH?=$(HOME)/.arduino15/packages/$(CORE_VENDOR)
    MODULE_PATH_ARDUINO?=$(HOME)/.arduino15/packages/arduino
    RM=rm
    SEP=/
  endif

  ifeq ($(UNAME_S),Darwin)
    MODULE_PATH?=$(HOME)/Library/Arduino15/packages/$(CORE_VENDOR)/
    MODULE_PATH_ARDUINO?=$(HOME)/Library/Arduino15/packages/arduino/
    RM=rm
    SEP=/
  endif
endif

PATHSEP=$(strip $(SEP))
ARM_GCC_PATH?=$(MODULE_PATH_ARDUINO)/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-
BUILD_DIR=build

# -----------------------------------------------------------------------------
# Tools
CC=$(ARM_GCC_PATH)gcc
OBJCOPY=$(ARM_GCC_PATH)objcopy
NM=$(ARM_GCC_PATH)nm
SIZE=$(ARM_GCC_PATH)size

# -----------------------------------------------------------------------------
# Compiler options
# eliminate PARANOIA define to save a tiny amount of code (~40B)
CFLAGS_EXTRA=-D$(SDCARD) -D__$(MCU)__ -DBOARD_ID_$(BOARD_ID) -DPARANOIA
CFLAGS=-mthumb -mcpu=$(MCPU) -Wall -c -std=gnu99 -ffunction-sections -fdata-sections -nostdlib -nostartfiles --param max-inline-insns-single=500 -Wa,-adhlns=$(<:%.c=$(BUILD_DIR)/%.lst)

ifdef DEBUG
  CFLAGS+=-g3 -O1 -DDEBUG=1
else
  CFLAGS+=-Os -DDEBUG=0
endif

ELF=$(NAME).elf
BIN=$(NAME).bin
HEX=$(NAME).hex


INCLUDES=-I"$(MODULE_PATH_ARDUINO)/tools/CMSIS/4.5.0/CMSIS/Include/" -I"$(MODULE_PATH)/tools/CMSIS-Atmel/1.0.0-mattairtech-2/CMSIS/Device/ATMEL/"

# -----------------------------------------------------------------------------
# Linker options
LDFLAGS=-mthumb -mcpu=$(MCPU) -Wall -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--unresolved-symbols=report-all
LDFLAGS+=-Wl,--warn-common -Wl,--warn-section-align -Wl,--warn-unresolved-symbols --specs=nano.specs --specs=nosys.specs

# -----------------------------------------------------------------------------
# Source files and objects
ifeq ($(findstring SAMC21,$(MCU)),SAMC21)
SOURCES= \
  board_driver_led.c \
  board_driver_serial.c \
  board_init.c \
  board_startup.c \
  util.c \
  SDCard/sdBootloader.c \
  SDCard/mmcbbp.c \
  SDCard/pff.c \
  main.c \
  sam_ba_monitor.c \
  sam_ba_serial.c

else
SOURCES= \
  board_driver_led.c \
  board_driver_serial.c \
  board_driver_usb.c \
  board_init.c \
  board_startup.c \
  util.c \
  SDCard/sdBootloader.c \
  SDCard/mmcbbp.c \
  SDCard/pff.c \
  main.c \
  sam_ba_usb.c \
  sam_ba_cdc.c \
  sam_ba_monitor.c \
  sam_ba_serial.c
endif

OBJECTS=$(addprefix $(BUILD_DIR)/, $(SOURCES:.c=.o))
DEPS=$(addprefix $(BUILD_DIR)/, $(SOURCES:.c=.d))

ifneq "test$(AVRSTUDIO_EXE_PATH)" "test"
  AS_BUILD=copy_for_atmel_studio
  AS_CLEAN=clean_for_atmel_studio
else
  AS_BUILD=
  AS_CLEAN=
endif


all: print_info $(SOURCES) $(BIN) $(HEX) $(AS_BUILD)

$(ELF): Makefile $(BUILD_DIR) $(OBJECTS)
	@echo ----------------------------------------------------------
	@echo Creating ELF binary
	"$(CC)" -L. -L$(BUILD_DIR) $(LDFLAGS) -Os -Wl,--gc-sections -save-temps -Tlinker_scripts/$(LINKER_SCRIPT) -Wl,-Map,"$(BUILD_DIR)/$(NAME).map" -o "$(BUILD_DIR)/$(ELF)" -Wl,--start-group $(OBJECTS) -lm -Wl,--end-group
	"$(NM)" "$(BUILD_DIR)/$(ELF)" >"$(BUILD_DIR)/$(NAME)_symbols.txt"
	"$(SIZE)" --format=sysv -t -x $(BUILD_DIR)/$(ELF)

$(BIN): $(ELF)
	@echo ----------------------------------------------------------
	@echo Creating flash binary
	"$(OBJCOPY)" -O binary $(BUILD_DIR)/$< $@

$(HEX): $(ELF)
	@echo ----------------------------------------------------------
	@echo Creating flash binary
	"$(OBJCOPY)" -O ihex $(BUILD_DIR)/$< $@

$(BUILD_DIR)/%.o: %.c
	@echo ----------------------------------------------------------
	@echo Compiling $< to $@
	"$(CC)" $(CFLAGS) $(CFLAGS_EXTRA) $(INCLUDES) $< -o $@
	@echo ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

$(BUILD_DIR):
	@echo ----------------------------------------------------------
	@echo Creating build folder
	-mkdir $(BUILD_DIR)
	-mkdir $(BUILD_DIR)$(PATHSEP)SDCard

print_info:
	@echo ----------------------------------------------------------
	@echo Compiling bootloader using
	@echo BASE PATH = $(MODULE_PATH)
	@echo GCC  PATH = $(ARM_GCC_PATH)
#	@echo OS        = $(OS)
#	@echo SHELL     = $(SHELL)
#	@echo TERM      = $(TERM)
#	"$(CC)" -v
#	env

copy_for_atmel_studio: $(BIN) $(HEX)
	@echo ----------------------------------------------------------
	@echo Atmel Studio detected, copying ELF to project root for debug
	cp $(BUILD_DIR)$(PATHSEP)$(ELF) .

clean_for_atmel_studio:
	@echo ----------------------------------------------------------
	@echo Atmel Studio detected, cleaning ELF from project root
	-$(RM) .$(PATHSEP)$(ELF)

clean: $(AS_CLEAN)
	@echo ----------------------------------------------------------
	@echo Cleaning project
	-$(RM) $(BIN)
	-$(RM) $(HEX)
	-$(RM) $(BUILD_DIR)$(PATHSEP)SDCard$(PATHSEP)*.*
	-$(RM) $(BUILD_DIR)$(PATHSEP)*.*
	-rmdir $(BUILD_DIR)$(PATHSEP)SDCard
	-rmdir $(BUILD_DIR)

mostly_clean: $(AS_CLEAN)
	@echo ----------------------------------------------------------
	@echo Cleaning project except bin
	-$(RM) $(HEX)
	-$(RM) $(BUILD_DIR)$(PATHSEP)SDCard$(PATHSEP)*.*
	-$(RM) $(BUILD_DIR)$(PATHSEP)*.*
	-rmdir $(BUILD_DIR)$(PATHSEP)SDCard
	-rmdir $(BUILD_DIR)

.phony: print_info $(BUILD_DIR)
