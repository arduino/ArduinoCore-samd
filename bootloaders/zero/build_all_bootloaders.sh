#!/bin/bash -ex

BOARD_ID=arduino_zero NAME=samd21_sam_ba make clean all

BOARD_ID=genuino_zero NAME=samd21_sam_ba_genuino make clean all

BOARD_ID=arduino_mkr1000 NAME=samd21_sam_ba_arduino_mkr1000 make clean all
mv -v samd21_sam_ba_arduino_mkr1000.* ../mkr1000/

BOARD_ID=genuino_mkr1000 NAME=samd21_sam_ba_genuino_mkr1000 make clean all
mv -v samd21_sam_ba_genuino_mkr1000.* ../mkr1000/

BOARD_ID=arduino_mkrzero NAME=samd21_sam_ba_arduino_mkrzero make clean all
mv -v samd21_sam_ba_arduino_mkrzero.* ../mkrzero/

BOARD_ID=arduino_mkrfox1200 NAME=samd21_sam_ba_arduino_mkrfox1200 make clean all
mv -v samd21_sam_ba_arduino_mkrfox1200.* ../mkrfox1200/

BOARD_ID=arduino_mkrgsm1400 NAME=samd21_sam_ba_arduino_mkrgsm1400 make clean all
mv -v samd21_sam_ba_arduino_mkrgsm1400.* ../mkrgsm1400/

BOARD_ID=arduino_mkrwan1300 NAME=samd21_sam_ba_arduino_mkrwan1300 make clean all
mv -v samd21_sam_ba_arduino_mkrwan1300.* ../mkrwan1300/

BOARD_ID=arduino_mkrwifi1010 NAME=samd21_sam_ba_arduino_mkrwifi1010 make clean all
mv -v samd21_sam_ba_arduino_mkrwifi1010.* ../mkrwifi1010/

echo Done building bootloaders!

