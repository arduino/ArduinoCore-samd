#
#  Copyright (c) 2014 Arduino.  All right reserved.
#
#  This library is free software; you can redistribute it and/or
#  modify it under the terms of the GNU Lesser General Public
#  License as published by the Free Software Foundation; either
#  version 2.1 of the License, or (at your option) any later version.
#
#  This library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#  See the GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with this library; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#

# Makefile for compiling validation application
.SUFFIXES: .o .a .c .cpp .s

# putting default variant
ifeq ("$(VARIANT)", "")
VARIANT=arduino_zero
endif

ifeq ("$(VARIANT)", "arduino_zero")
DEVICE=__SAMD21G18__
endif

ifeq ($(DEVICE), __SAMD21G18__)
DEVICE_NAME=samd21g18
DEVICE_SERIE=samd21
else ifeq ($(DEVICE), __SAMD21J18__)
DEVICE_NAME=samd21j18
DEVICE_SERIE=samd21
endif

#-------------------------------------------------------------------------------
# Path
#-------------------------------------------------------------------------------
HARDWARE_PATH=../../../../../../
ARCH_PATH=$(HARDWARE_PATH)/arduino/samd
ARDUINO_CORE_PATH=$(ARCH_PATH)/cores/arduino
ARDUINO_USB_PATH=$(ARDUINO_CORE_PATH)/USB
VARIANT_PATH = $(ARCH_PATH)/variants/$(VARIANT)
TOOLS_PATH = $(HARDWARE_PATH)/tools
CMSIS_ROOT_PATH=$(TOOLS_PATH)/CMSIS
CMSIS_ARM_PATH=$(CMSIS_ROOT_PATH)/CMSIS/Include
CMSIS_ATMEL_PATH=$(CMSIS_ROOT_PATH)/Device/ATMEL
#CMSIS_DEVICE_PATH=$(CMSIS_ROOT_PATH)/Device/ATMEL/$(DEVICE_SERIE)

PROJECT_BASE_PATH = ..

TOOLCHAIN=gcc


# Output directories
OUTPUT_PATH = debug_$(VARIANT)

#-------------------------------------------------------------------------------
# Files
#-------------------------------------------------------------------------------

vpath %.cpp $(PROJECT_BASE_PATH)

#VPATH+=$(PROJECT_BASE_PATH)

INCLUDES = -I$(ARDUINO_CORE_PATH)
INCLUDES += -I$(ARDUINO_CORE_PATH)/USB
INCLUDES += -I$(VARIANT_PATH)
INCLUDES += -I$(SYSTEM_PATH)/libsam
INCLUDES += -I$(CMSIS_ARM_PATH)
INCLUDES += -I$(CMSIS_ATMEL_PATH)
INCLUDES += -I$(CMSIS_DEVICE_PATH)

#-------------------------------------------------------------------------------
ifdef DEBUG
include debug.mk
else
include release.mk
endif

#-------------------------------------------------------------------------------
# Tools
#-------------------------------------------------------------------------------

include $(TOOLCHAIN).mk

CFLAGS += -DUSB_VID=0x2341 -DUSB_PID=0x004d
CPPFLAGS += -DUSB_VID=0x2341 -DUSB_PID=0x004d

#-------------------------------------------------------------------------------
ifdef DEBUG
OUTPUT_OBJ=debug
LIBS_POSTFIX=dbg
else
OUTPUT_OBJ=release
LIBS_POSTFIX=rel
endif

OUTPUT_BIN=test_$(TOOLCHAIN)_$(LIBS_POSTFIX)
LIBS=-Wl,--start-group -lgcc -lc -lstdc++ -Wl,--end-group

LIB_PATH =-L$(PROJECT_BASE_PATH)/..
LIB_PATH+=-L=/lib/thumb2
#LIB_PATH+=-L=/../lib/gcc/arm-none-eabi/4.5.2/thumb2

#-------------------------------------------------------------------------------
# CPP source files and objects
#-------------------------------------------------------------------------------
CPP_SRC=$(wildcard $(PROJECT_BASE_PATH)/*.cpp)

CPP_OBJ_TEMP = $(patsubst %.cpp, %.o, $(notdir $(CPP_SRC)))

# during development, remove some files
CPP_OBJ_FILTER=

CPP_OBJ=$(filter-out $(CPP_OBJ_FILTER), $(CPP_OBJ_TEMP))

#-------------------------------------------------------------------------------
# Rules
#-------------------------------------------------------------------------------
all: test

test: create_output $(OUTPUT_BIN)


.PHONY: create_output
create_output:
	@echo ------------------------------------------------------------------------------------
	@echo --- Preparing $(VARIANT) files in $(OUTPUT_PATH) $(OUTPUT_BIN)
#	@echo -------------------------
#	@echo *$(INCLUDES)
#	@echo -------------------------
#	@echo *$(C_SRC)
#	@echo -------------------------
#	@echo *$(C_OBJ)
#	@echo -------------------------
#	@echo *$(addprefix $(OUTPUT_PATH)/, $(C_OBJ))
#	@echo -------------------------
#	@echo *$(CPP_SRC)
#	@echo -------------------------
#	@echo *$(CPP_OBJ)
#	@echo -------------------------
#	@echo *$(addprefix $(OUTPUT_PATH)/, $(CPP_OBJ))
#	@echo -------------------------
#	@echo *$(A_SRC)
#	@echo -------------------------

	-@mkdir $(OUTPUT_PATH) 1>NUL 2>&1
	@echo ------------------------------------------------------------------------------------

$(addprefix $(OUTPUT_PATH)/,$(CPP_OBJ)): $(OUTPUT_PATH)/%.o: %.cpp
	@echo *** Current folder is $(shell cd)
#	@"$(CXX)" -c $(CPPFLAGS) $< -o $@
	"$(CXX)" -v -c $(CPPFLAGS) $< -o $@

$(OUTPUT_BIN): $(addprefix $(OUTPUT_PATH)/, $(C_OBJ)) $(addprefix $(OUTPUT_PATH)/, $(CPP_OBJ)) $(addprefix $(OUTPUT_PATH)/, $(A_OBJ))
	@"$(CC)" $(LIB_PATH) $(LDFLAGS) -T"$(VARIANT_PATH)/linker_scripts/gcc/flash.ld" -Wl,-Map,$(OUTPUT_PATH)/$@.map -o $(OUTPUT_PATH)/$@.elf $^ $(LIBS)
	@"$(NM)" $(OUTPUT_PATH)/$@.elf >$(OUTPUT_PATH)/$@.elf.txt
	@"$(OBJCOPY)" -O binary $(OUTPUT_PATH)/$@.elf $(OUTPUT_PATH)/$@.bin
	$(SIZE) $^ $(OUTPUT_PATH)/$@.elf

.PHONY: clean
clean:
	@echo ------------------------------------------------------------------------------------
	@echo --- Cleaning test files for $(VARIANT)
	-@$(RM) $(OUTPUT_PATH) 1>NUL 2>&1
	@echo ------------------------------------------------------------------------------------

#	-$(RM) $(OUTPUT_PATH)/test.o
#	-$(RM) $(OUTPUT_PATH)/$(OUTPUT_BIN).elf
#	-$(RM) $(OUTPUT_PATH)/$(OUTPUT_BIN).elf.txt
#	-$(RM) $(OUTPUT_PATH)/$(OUTPUT_BIN).bin
#	-$(RM) $(OUTPUT_PATH)/$(OUTPUT_BIN).map

debug: test
	@"$(GDB)" -x "$(VARIANT_PATH)/debug_scripts/gcc/$(VARIANT)_flash.gdb" -ex "reset" -readnow -se $(OUTPUT_PATH)/$(OUTPUT_BIN).elf

