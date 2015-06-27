1- Prerequisites

gcc-arm-none-eabi-4.8.3-2014q1

2- Selecting between USB and UART interface

Set the define SAM_BA_INTERFACE to
SAM_BA_UART_ONLY for only UART interface
SAM_BA_USBCDC_ONLY for only USB CDC interface
SAM_BA_BOTH_INTERFACES for enabling both the interfaces

SAM_BA_INTERFACE value should be modified in
Project Options -> C/C++ Compiler -> Preprocessor -> Defined symbols
Project Options -> Assembler -> Preprocessor -> Defined symbols
The default value of SAM_BA_BOTH_INTERFACES is defined in sam_ba_monitor.h

Additionally, you must select the cpu in the makefile (3 locations)
as well as the TOOLS_PATH variable (and possibly other include paths).
On Windows, change 'rm -f' to 'del' in the makefile (uncomment line 29, comment line 30).

3- Start application check

Bootloader checks for the state of BOOT_LOAD_PIN (configurable by the user from main.h). If BOOT_LOAD_PIN is pulled low, bootloader execution is resumed.
Else, the first location of application is fetched and checked. If it is empty (0xFFFFFFFF), then bootloader execution is resumed. Else it jumps to application and starts execution from there.

BOOT_LOAD_PIN is PA27 of SAMD21ExxA, Arduino pin 27 of MT-D21E board.
