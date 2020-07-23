#!/bin/sh -x

#Sara U201 
ARDUINO=arduino
SKETCH_NAME="SSUBoot.ino"
SKETCH="$PWD/$SKETCH_NAME"
BUILD_PATH="$PWD/build"
OUTPUT_PATH="../../src/boot"

if [[ "$OSTYPE" == "darwin"* ]]; then
	ARDUINO="/Applications/Arduino.app/Contents/MacOS/Arduino"
fi

buildSSUBootSketch() {
	BOARD=$1
	DESTINATION=$2

	$ARDUINO --verify --board $BOARD --preserve-temp-files --pref build.path="$BUILD_PATH" $SKETCH
	cat "$BUILD_PATH/$SKETCH_NAME.bin" | xxd -include > $DESTINATION
	rm -rf "$BUILD_PATH"
}

buildSSUBootSketch "arduino:samd:mkrgsm1400" "$OUTPUT_PATH/mkrgsm1400.h"
