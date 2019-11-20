#!/bin/sh -x

ARDUINO=arduino
SKETCH_NAME="NiNaBoot.ino"
SKETCH="$PWD/$SKETCH_NAME"
BUILD_PATH="$PWD/build"
OUTPUT_PATH="../../src/boot"

if [[ "$OSTYPE" == "darwin"* ]]; then
	ARDUINO="/Applications/Arduino.app/Contents/MacOS/Arduino"
fi

buildSDUBootSketch() {
	BOARD=$1
	DESTINATION=$2

	$ARDUINO --verify --board $BOARD --preserve-temp-files --pref build.path="$BUILD_PATH" $SKETCH
	cat "$BUILD_PATH/$SKETCH_NAME.bin" | xxd -i > $DESTINATION
	rm -rf "$BUILD_PATH"
}

mkdir -p "$OUTPUT_PATH"

buildSDUBootSketch "arduino:samd:mkrwifi1010" "$OUTPUT_PATH/mkrwifi1010.h"
buildSDUBootSketch "arduino:samd:mkrvidor" "$OUTPUT_PATH/mkrvidor4000.h"
