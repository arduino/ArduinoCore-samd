#!/bin/sh -x

ARDUINO=arduino
BOOTLOADER_DIR=" ../../../bootloaders"
OUTPUT_PATH="../src/bootloaders"

if [[ "$OSTYPE" == "darwin"* ]]; then
	ARDUINO="/Applications/Arduino.app/Contents/MacOS/Arduino"
fi

ls $BOOTLOADER_DIR/mkrvidor4000
cat $BOOTLOADER_DIR/mkrvidor4000/samd21_sam_ba_arduino_mkrvidor4000.bin | xxd -i > $OUTPUT_PATH/mkrvidor4000.h
