1- Prerequisites

IAR Embedded Workbench for ARM 7.10

2- Selecting between USB and UART interface

Set the define SAM_BA_INTERFACE to
SAM_BA_UART_ONLY for only UART interface
SAM_BA_USBCDC_ONLY for only USB CDC interface
SAM_BA_BOTH_INTERFACES for enabling both the interfaces

SAM_BA_INTERFACE value should be modified in
Project Options -> C/C++ Compiler -> Preprocessor -> Defined symbols
Project Options -> Assembler -> Preprocessor -> Defined symbols