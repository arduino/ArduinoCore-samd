#!/bin/sh -x

ARDUINO=arduino
SKETCH_NAME="SFUBoot.ino"
SKETCH="$PWD/$SKETCH_NAME"
BUILD_PATH="$PWD/build"
OUTPUT_PATH="../../src/boot"

if [[ "$OSTYPE" == "darwin"* ]]; then
	ARDUINO="/Applications/Arduino.app/Contents/MacOS/Arduino"
fi

buildSFUBootSketch() {
	BOARD=$1
	DESTINATION=$2

	$ARDUINO --verify --board $BOARD --preserve-temp-files --pref build.path="$BUILD_PATH" $SKETCH
	cat "$BUILD_PATH/$SKETCH_NAME.bin" | xxd -include > $DESTINATION
	rm -rf "$BUILD_PATH"
}

mkdir -p "$OUTPUT_PATH"

buildSFUBootSketch "arduino:samd:mkrzero" "$OUTPUT_PATH/mkrzero.h"
buildSFUBootSketch "arduino:samd:mkr1000" "$OUTPUT_PATH/mkr1000.h"
buildSFUBootSketch "arduino:samd:mkrwifi1010" "$OUTPUT_PATH/mkrwifi1010.h"
buildSFUBootSketch "arduino:samd:mkrgsm1400" "$OUTPUT_PATH/mkrgsm1400.h"
buildSFUBootSketch "arduino:samd:mkrnb1500" "$OUTPUT_PATH/mkrnb1500.h"
buildSFUBootSketch "arduino:samd:mkrvidor4000" "$OUTPUT_PATH/mkrvidor4000.h"
buildSFUBootSketch "arduino:samd:mkrwan1310" "$OUTPUT_PATH/mkrwan1310.h"
buildSFUBootSketch "arduino:samd:mkrfox1200" "$OUTPUT_PATH/mkrfox1200.h"
