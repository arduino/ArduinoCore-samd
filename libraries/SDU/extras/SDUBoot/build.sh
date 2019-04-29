#!/bin/sh -x

ARDUINO=arduino
SKETCH_NAME="SDUBoot.ino"
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

buildSDUBootSketch "arduino:samd:arduino_zero_edbg" "$OUTPUT_PATH/zero.h"
buildSDUBootSketch "arduino:samd:mkr1000" "$OUTPUT_PATH/mkr1000.h"
buildSDUBootSketch "arduino:samd:mkrzero" "$OUTPUT_PATH/mkrzero.h"
buildSDUBootSketch "arduino:samd:mkrfox1200" "$OUTPUT_PATH/mkrfox1200.h"
buildSDUBootSketch "arduino:samd:mkrgsm1400" "$OUTPUT_PATH/mkrgsm1400.h"
buildSDUBootSketch "arduino:samd:mkrwan1300" "$OUTPUT_PATH/mkrwan1300.h"
buildSDUBootSketch "arduino:samd:mkrwifi1010" "$OUTPUT_PATH/mkrwifi1010.h"
buildSDUBootSketch "arduino:samd:mkrnb1500" "$OUTPUT_PATH/mkrnb1500.h"
buildSDUBootSketch "arduino:samd:mzero_bl" "$OUTPUT_PATH/mzero.h"
