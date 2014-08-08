1- Prerequisites

IAR Embedded Workbench for ARM 7.30

2- Selecting between USB and UART interface

Set the define SAM_BA_INTERFACE to
SAM_BA_UART_ONLY for only UART interface
SAM_BA_USBCDC_ONLY for only USB CDC interface
SAM_BA_BOTH_INTERFACES for enabling both the interfaces

SAM_BA_INTERFACE value should be modified in
Project Options -> C/C++ Compiler -> Preprocessor -> Defined symbols
Project Options -> Assembler -> Preprocessor -> Defined symbols

3- Start application check

Bootloader checks for the state of BOOT_LOAD_PIN (configurable by the user from main.h). If BOOT_LOAD_PIN is pulled low, bootloader execution is resumed.
Else, the first location of application is fetched and checked. If it is empty (0xFFFFFFFF), then bootloader execution is resumed. Else it jumps to application and starts execution from there.

Currently, BOOT_LOAD_PIN is PA15 of SAMD21G18A, pin 5 of Arduino Zero board.
