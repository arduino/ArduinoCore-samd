TOOLS_PATH="../../../arduino15/packages/arduino/tools"
ARM_GCC_PATH=$(TOOLS_PATH)/arm-none-eabi-gcc/4.8.3-2014q1/bin
CC=$(ARM_GCC_PATH)/arm-none-eabi-gcc
CFLAGS=-mthumb -mcpu=cortex-m0plus -Wall -c -g -Os -w -std=gnu99 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500
LDFLAGS=-mthumb -mcpu=cortex-m0plus -Wall -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,--warn-unresolved-symbols
BLD_EXTA_FLAGS=-D__SAMD21E18A__
BUILD_PATH=build
INCLUDES=-I$(TOOLS_PATH)/CMSIS/4.0.0-atmel/CMSIS/Include/ -I$(TOOLS_PATH)/CMSIS/4.0.0-atmel/Device/ATMEL/ -I./drivers/ -I./utils/ -I./utils/preprocessor/ -I./utils/interrupt 
SOURCES=main.c sam_ba_monitor.c startup_samd21.c usart_sam_ba.c drivers/cdc_enumerate.c drivers/uart_driver.c utils/interrupt/interrupt_sam_nvic.c 
OBJECTS=$(addprefix $(BUILD_PATH)/, $(SOURCES:.c=.o))

NAME=samd21e18a_sam_ba
EXECUTABLE=$(NAME).bin

SLASH=/
BSLASH=$(EMPTY)\$(EMPTY)

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS) 
	$(CC) -L$(BUILD_PATH) $(LDFLAGS) -Os -Wl,--gc-sections -save-temps  -Tsamd21e18a_flash.ld -Wl,-Map,$(BUILD_PATH)/$(NAME).map --specs=nano.specs --specs=nosys.specs -o $(BUILD_PATH)/$(NAME).elf $(OBJECTS) -Wl,--start-group -lm -Wl,--end-group
	$(ARM_GCC_PATH)/arm-none-eabi-objcopy -O binary $(BUILD_PATH)/$(NAME).elf $@

$(BUILD_PATH)/%.o: %.c
	-@mkdir -p $(@D)
	$(CC) $(CFLAGS) $(BLD_EXTA_FLAGS) $(INCLUDES) $< -o $@
	
clean:
#	del $(EXECUTABLE) $(subst /,\,$(OBJECTS)) $(subst /,\,$(BUILD_PATH)/$(NAME).*)
	rm -f $(EXECUTABLE) $(OBJECTS) $(BUILD_PATH)/$(NAME).*
