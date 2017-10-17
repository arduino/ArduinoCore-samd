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
    RM=rm
    SEP=/
  else
    MODULE_PATH?=$(LOCALAPPDATA)/Arduino15/packages/arduino
    RM=rm
    SEP=\\
  endif
else
  UNAME_S := $(shell uname -s)

  ifeq ($(UNAME_S),Linux)
    MODULE_PATH?=$(HOME)/.arduino15/packages/arduino
    RM=rm
    SEP=/
  endif

  ifeq ($(UNAME_S),Darwin)
    MODULE_PATH?=$(HOME)/Library/Arduino15/packages/arduino/
    RM=rm
    SEP=/
  endif
endif

ARM_GCC_PATH?=$(MODULE_PATH)/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-
BUILD_PATH=build

# -----------------------------------------------------------------------------
# Tools
CC=$(ARM_GCC_PATH)gcc
OBJCOPY=$(ARM_GCC_PATH)objcopy
NM=$(ARM_GCC_PATH)nm
SIZE=$(ARM_GCC_PATH)size

# -----------------------------------------------------------------------------
# Compiler options
CFLAGS_EXTRA=-D$(BOARD_CHIP) -DBOARD_ID_$(BOARD_ID)
CFLAGS=-mthumb -mcpu=cortex-m4 -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -ffunction-sections -fdata-sections -nostdlib -nostartfiles --param max-inline-insns-single=500
ifdef DEBUG
  CFLAGS+=-g3 -O1 -DDEBUG=1
else
  CFLAGS+=-Os -DDEBUG=0
endif

ELF=$(BOARD_ID)_sam_ba.elf
BIN=$(BOARD_ID)_sam_ba.bin
HEX=$(BOARD_ID)_sam_ba.hex


INCLUDES=-I"$(MODULE_PATH)/tools/CMSIS/4.5.0/CMSIS/Include/" -I"$(MODULE_PATH)/tools/CMSIS-Atmel/1.1.0/CMSIS/Device/ATMEL/"

# -----------------------------------------------------------------------------
# Linker options
LDFLAGS=-mthumb -mcpu=cortex-m4 -Wall -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--unresolved-symbols=report-all
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
	"$(CC)" -L. -L$(BUILD_PATH) $(LDFLAGS) -Os -Wl,--gc-sections -save-temps -Tbootloader_$(BOARD_ID).ld -Wl,-Map,"$(BUILD_PATH)/$(NAME).map" -o "$(BUILD_PATH)/$(ELF)" -Wl,--start-group $(OBJECTS) -lm -Wl,--end-group
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
