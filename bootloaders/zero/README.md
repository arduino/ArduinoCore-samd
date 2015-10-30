# Arduino Zero Bootloader
# -----------------------

## 1- Prerequisites

The project build is based on Makefile system.
Makefile is present at project root and try to handle multi-platform cases.

Multi-plaform GCC is provided by ARM here: https://launchpad.net/gcc-arm-embedded/+download

### Windows

Native command line:
Make binary can be obtained here: http://gnuwin32.sourceforge.net/packages/make.htm

* Cygwin/MSys/MSys2/Babun/etc...:
It is available natively in all distributions.

* Atmel Studio:
An Atmel Studio Makefile-based project is present at project root, just open samd21_sam_ba.atsln file.

### Linux

Make is usually available by default.

### OS X

Make is available through XCode package.


## 2- Selecting available SAM-BA interfaces

By default both USB and UART are made available, but this parameter can be modified in sam_ba_monitor.h, line 31:

Set the define SAM_BA_INTERFACE to
* SAM_BA_UART_ONLY for only UART interface
* SAM_BA_USBCDC_ONLY for only USB CDC interface
* SAM_BA_BOTH_INTERFACES for enabling both the interfaces


## 3- Behaviour

This bootloader implements the double-tap on Reset button.
By quickly pressing this button two times, the board will reset and stay in bootloader, waiting for communication on either USB or USART.

The USB port in use is the USB Native port, close to the Reset button.
The USART in use is the one available on pins D0/D1, labelled respectively RX/TX.
