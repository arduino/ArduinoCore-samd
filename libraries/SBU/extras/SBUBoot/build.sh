#!/bin/sh -x

#Sara R410M 
ARDUINO=arduino
SKETCH_NAME="SBUBoot.ino"
SKETCH="$PWD/$SKETCH_NAME"
BUILD_PATH="$PWD/build"
OUTPUT_PATH="../../src/boot"

if [[ "$OSTYPE" == "darwin"* ]]; then
	ARDUINO="/Applications/Arduino.app/Contents/MacOS/Arduino"
fi

buildSBUBootSketch() {
	BOARD=$1
	DESTINATION=$2

	$ARDUINO --verify --board $BOARD --preserve-temp-files --pref build.path="$BUILD_PATH" $SKETCH
	cat "$BUILD_PATH/$SKETCH_NAME.bin" | xxd -include > $DESTINATION
	rm -rf "$BUILD_PATH"
}

buildSBUBootSketch "arduino:samd:mkrnb1500" "$OUTPUT_PATH/mkrnb1500.h"
