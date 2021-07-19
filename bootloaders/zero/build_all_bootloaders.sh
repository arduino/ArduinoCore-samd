#!/bin/bash -ex

BOARD_ID=arduino_zero NAME=samd21_sam_ba make clean all

BOARD_ID=arduino_mkr1000 NAME=samd21_sam_ba_arduino_mkr1000 make clean all
mv -v samd21_sam_ba_arduino_mkr1000.* ../mkr1000/

BOARD_ID=arduino_mkrzero NAME=samd21_sam_ba_arduino_mkrzero make clean all
mv -v samd21_sam_ba_arduino_mkrzero.* ../mkrzero/

BOARD_ID=arduino_mkrfox1200 NAME=samd21_sam_ba_arduino_mkrfox1200 make clean all
mv -v samd21_sam_ba_arduino_mkrfox1200.* ../mkrfox1200/

BOARD_ID=arduino_mkrgsm1400 NAME=samd21_sam_ba_arduino_mkrgsm1400 make clean all
mv -v samd21_sam_ba_arduino_mkrgsm1400.* ../mkrgsm1400/

BOARD_ID=arduino_mkrwan1300 NAME=samd21_sam_ba_arduino_mkrwan1300 make clean all
mv -v samd21_sam_ba_arduino_mkrwan1300.* ../mkrwan1300/

BOARD_ID=arduino_mkrwan1310 NAME=samd21_sam_ba_arduino_mkrwan1310 make clean all
mv -v samd21_sam_ba_arduino_mkrwan1310.* ../mkrwan1300/

BOARD_ID=arduino_mkrwifi1010 NAME=samd21_sam_ba_arduino_mkrwifi1010 make clean all
mv -v samd21_sam_ba_arduino_mkrwifi1010.* ../mkrwifi1010/

BOARD_ID=arduino_mkrvidor4000 SAM_BA_INTERFACES=SAM_BA_USBCDC_ONLY NAME=samd21_sam_ba_arduino_mkrvidor4000 make clean all
mv -v samd21_sam_ba_arduino_mkrvidor4000.* ../mkrvidor4000/

BOARD_ID=arduino_mkrnb1500 NAME=samd21_sam_ba_arduino_mkrnb1500 make clean all
mv -v samd21_sam_ba_arduino_mkrnb1500.* ../mkrnb1500/

BOARD_ID=arduino_nano_33_iot NAME=samd21_sam_ba_arduino_nano_33_iot make clean all
mv -v samd21_sam_ba_arduino_nano_33_iot.* ../nano_33_iot/

echo Done building bootloaders!

