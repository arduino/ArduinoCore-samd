# Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.
# Copyright (c) 2015 Arduino LLC.  All right reserved.
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

# -----------------------------------------------------------------------------
# Paths
ifeq ($(OS),Windows_NT)

  # Are we using mingw/msys/msys2/cygwin?
  ifeq ($(TERM),xterm)
		T=$(shell cygpath -u $(LOCALAPPDATA))
    MODULE_PATH?=$(T)/Arduino15/packages/arduino
    ARM_GCC_PATH?=$(MODULE_PATH)/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-
    RM=rm
    SEP=/
  else
    MODULE_PATH?=$(LOCALAPPDATA)/Arduino15/packages/arduino
    ARM_GCC_PATH?=$(MODULE_PATH)/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-
    RM=rm
    SEP=\\
  endif
else
    UNAME_S := $(shell uname -s)

    ifeq ($(UNAME_S),Linux)
			MODULE_PATH?=$(HOME)/.arduino15/packages/arduino
			ARM_GCC_PATH?=$(MODULE_PATH)/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-
      RM=rm
	    SEP=/
    endif

    ifeq ($(UNAME_S),Darwin)
			MODULE_PATH?=$(HOME)/Library/Arduino15/packages/arduino/
			ARM_GCC_PATH?=$(MODULE_PATH)/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-
      RM=rm
	    SEP=/
    endif
endif

BUILD_PATH=build

# -----------------------------------------------------------------------------
# Tools
CC=$(ARM_GCC_PATH)gcc
OBJCOPY=$(ARM_GCC_PATH)objcopy
NM=$(ARM_GCC_PATH)nm
SIZE=$(ARM_GCC_PATH)size

# -----------------------------------------------------------------------------
# Compiler options
CFLAGS=-mthumb -mcpu=cortex-m0plus -Wall -c -std=gnu99 -ffunction-sections -fdata-sections -nostdlib -nostartfiles --param max-inline-insns-single=500
ifdef DEBUG
CFLAGS+=-g3 -O1 -DDEBUG=1
else
CFLAGS+=-Os -DDEBUG=0
endif

# Adafruit Feather Zero (0x239A 0x000B)
#CFLAGS_EXTRA?=-D__SAMD21G18A__ -DUSB_PID_HIGH=0x00 -DUSB_PID_LOW=0x0B -DUSB_VID_LOW=0x9A -DUSB_VID_HIGH=0x23
# Adafruit Feather SAMR (0x239A 0x0014)
CFLAGS_EXTRA?=-D__SAMR21G18A__ -DUSB_PID_HIGH=0x00 -DUSB_PID_LOW=0x14 -DUSB_VID_LOW=0x9A -DUSB_VID_HIGH=0x23
# Arduino Zero (PID == 0x004D)
# CFLAGS_EXTRA?=-D__SAMD21G18A__ -DUSB_PID_HIGH=0x00 -DUSB_PID_LOW=0x4D -DUSB_VID_LOW=0x41 -DUSB_VID_HIGH=0x23
# Genuino Zero (PID == 0x024D)
# CFLAGS_EXTRA?=-D__SAMD21G18A__ -DUSB_PID_HIGH=0x02 -DUSB_PID_LOW=0x4D -DUSB_VID_LOW=0x41 -DUSB_VID_HIGH=0x23
# Arduino MKR1000 (PID == 0x004E)
# CFLAGS_EXTRA?=-D__SAMD21G18A__ -DUSB_PID_HIGH=0x00 -DUSB_PID_LOW=0x4E -DUSB_VID_LOW=0x41 -DUSB_VID_HIGH=0x23
# Genuino MKR1000 (PID == 0x024E)
# CFLAGS_EXTRA?=-D__SAMD21G18A__ -DUSB_PID_HIGH=0x02 -DUSB_PID_LOW=0x4E -DUSB_VID_LOW=0x41 -DUSB_VID_HIGH=0x23

# samd or samr
NAME=samr21_sam_ba

INCLUDES=-I"$(MODULE_PATH)/tools/CMSIS/4.0.0-atmel/CMSIS/Include/" -I"$(MODULE_PATH)/tools/CMSIS/4.0.0-atmel/Device/ATMEL/"

# -----------------------------------------------------------------------------
# Linker options
LDFLAGS=-mthumb -mcpu=cortex-m0plus -Wall -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--unresolved-symbols=report-all
LDFLAGS+=-Wl,--warn-common -Wl,--warn-section-align -Wl,--warn-unresolved-symbols --specs=nano.specs --specs=nosys.specs

# -----------------------------------------------------------------------------
# Source files and objects
SOURCES= \
board_driver_led.c \
board_driver_serial.c \
board_driver_usb.c \
board_init.c \
board_startup.c \
main.c \
sam_ba_usb.c \
sam_ba_cdc.c \
sam_ba_monitor.c \
sam_ba_serial.c

OBJECTS=$(addprefix $(BUILD_PATH)/, $(SOURCES:.c=.o))
DEPS=$(addprefix $(BUILD_PATH)/, $(SOURCES:.c=.d))

ELF=$(NAME).elf
BIN=$(NAME).bin
HEX=$(NAME).hex

ifneq "test$(AVRSTUDIO_EXE_PATH)" "test"
AS_BUILD=copy_for_atmel_studio
AS_CLEAN=clean_for_atmel_studio
else
AS_BUILD=
AS_CLEAN=
endif


all: print_info $(SOURCES) $(BIN) $(HEX) $(AS_BUILD)

$(ELF): Makefile $(BUILD_PATH) $(OBJECTS)
	@echo ----------------------------------------------------------
	@echo Creating ELF binary
	"$(CC)" -L. -L$(BUILD_PATH) $(LDFLAGS) -Os -Wl,--gc-sections -save-temps -Tbootloader_samd21x18.ld -Wl,-Map,"$(BUILD_PATH)/$(NAME).map" -o "$(BUILD_PATH)/$(ELF)" -Wl,--start-group $(OBJECTS) -lm -Wl,--end-group
	"$(NM)" "$(BUILD_PATH)/$(ELF)" >"$(BUILD_PATH)/$(NAME)_symbols.txt"
	"$(SIZE)" --format=sysv -t -x $(BUILD_PATH)/$(ELF)

$(BIN): $(ELF)
	@echo ----------------------------------------------------------
	@echo Creating flash binary
	"$(OBJCOPY)" -O binary $(BUILD_PATH)/$< $@

$(HEX): $(ELF)
	@echo ----------------------------------------------------------
	@echo Creating flash binary
	"$(OBJCOPY)" -O ihex $(BUILD_PATH)/$< $@

$(BUILD_PATH)/%.o: %.c
	@echo ----------------------------------------------------------
	@echo Compiling $< to $@
	"$(CC)" $(CFLAGS) $(CFLAGS_EXTRA) $(INCLUDES) $< -o $@
	@echo ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

$(BUILD_PATH):
	@echo ----------------------------------------------------------
	@echo Creating build folder
	-mkdir $(BUILD_PATH)

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
	cp $(BUILD_PATH)/$(ELF) .

clean_for_atmel_studio:
	@echo ----------------------------------------------------------
	@echo Atmel Studio detected, cleaning ELF from project root
	-$(RM) ./$(ELF)

clean: $(AS_CLEAN)
	@echo ----------------------------------------------------------
	@echo Cleaning project
	-$(RM) $(BIN)
	-$(RM) $(HEX)
	-$(RM) $(BUILD_PATH)/*.*
	-rmdir $(BUILD_PATH)

.phony: print_info $(BUILD_PATH)
