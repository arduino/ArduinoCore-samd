# Arduino Zero Bootloader

## 1- Prerequisites

The project build is based on Makefile system.
Makefile is present at project root and try to handle multi-platform cases.

Multi-plaform GCC is provided by ARM here: https://launchpad.net/gcc-arm-embedded/+download

Atmel Studio contains both make and ARM GCC toolchain. You don't need to install them in this specific use case.

For all builds and platforms you will need to have the Arduino IDE installed and the board support
package for "Arduino SAMD Boards (32-bits ARM Cortex-M0+)". You can install the latter
from the former's "Boards Manager" UI.

This version of the bootloader requires bossac (1.6.1-arduino-mattairtech-1)
from the package "MattairTech SAM M0+ Boards" or download bossac directly:
* https://www.mattairtech.com/software/arduino/bossac-1.6.1-arduino-mattairtech-1-mingw32.zip (Windows 32 bit and 64 bit)
* https://www.mattairtech.com/software/arduino/bossac-1.6.1-arduino-mattairtech-1-x86_64-linux-gnu.tar.bz2 (Linux 64 bit)
* https://www.mattairtech.com/software/arduino/bossac-1.6.1-arduino-mattairtech-1-i686-linux-gnu.tar.bz2 (Linux 32 bit)
* https://www.mattairtech.com/software/arduino/bossac-1.6.1-arduino-mattairtech-1-x86_64-apple-darwin.tar.gz (OS X 64 bit)

### Windows

* Native command line
Make binary can be obtained here: http://gnuwin32.sourceforge.net/packages/make.htm

* Cygwin/MSys/MSys2/Babun/etc...
It is available natively in all distributions.

* Atmel Studio
An Atmel Studio **7** Makefile-based project is present at project root, just open samd21_sam_ba.atsln file in AS7.

### Linux

Make is usually available by default.

### OS X

Make is available through XCode package.


## 2- Makefile Configuration

The section between 'Begin Configuration' and 'End Configuration' in the Makefile should be edited. Set MCU to a chip listed in the comments.
Set FLASH to a size listed in the comments that matches your MCU. Both SAM_BA_INTERFACE and ARDUINO_EXTENDED_CAPABILITIES will then be set automatically.
By default, for chips with 32KB of FLASH or more, an 8KB bootloader is built with SAM_BA_INTERFACE=SAM_BA_BOTH_INTERFACES and ARDUINO_EXTENDED_CAPABILITIES=1.
For chips with 16KB of FLASH (SAMD11), a 4KB bootloader is built with SAM_BA_INTERFACE=SAM_BA_USBCDC_ONLY and ARDUINO_EXTENDED_CAPABILITIES=0.
This can be changed in the Makefile:

Set SAM_BA_INTERFACE to
* SAM_BA_UART_ONLY for only UART interface
* SAM_BA_USBCDC_ONLY for only USB CDC interface
* SAM_BA_BOTH_INTERFACES for enabling both the interfaces

If you want to build a 4KB bootloader, you will need to:
* Choose only one interface from above
* Set ARDUINO_EXTENDED_CAPABILITIES=0
* Disable BOOT_DOUBLE_TAP (board_definitions.h) for SAMD21 only (the SAMD11 has enough room)

Note that both bossac (Devices.h) and the MattairTech core (boards.txt) are configured to use a 4KB bootloader for the SAMD11 only.
All other SAM M0+ chips use the 8KB bootloader. When using a different bootloader size, Devices.h and boards.txt must be modified as well.

## 3- Behaviour / Board Configuration

Board configuration is available in board_definitions.h.

First, if BOOT_DOUBLE_TAP_ADDRESS is defined, the bootloader checks if the reset button was pressed twice in quick succession (within 500ms).
If so, bootloader execution is resumed, waiting for communication on either USB or USART.

Second, the bootloader checks the state of BOOT_LOAD_PIN, which will be pulled high internally.
After a 10ms. delay, if BOOT_LOAD_PIN is low, bootloader execution is resumed.

Finally, the first location of the sketch is fetched and checked. If it is empty (0xFFFFFFFF), then bootloader execution is resumed.
Note that when Arduino auto-reset (into bootloader) is initiated, the first flash row is erased, so the bootloader will always run after reset until a new sketch is transferred.

Otherwise, with no bootloader entry condition present, it jumps to the application and starts execution from there.
The LED (configured with BOARD_LED_PORT and BOARD_LED_PIN) will light during bootloader execution.

The USART by default is available on pins PA10 (USART TX) and PA11 (USART RX). The baudrate is 115200, with 8bits of data, no parity and 1 stop bit (8N1).


## 4- Details

**Pinmap**

The following pins are used by the program :
PA25 : input/output (USB DP)
PA24 : input/output (USB DM)
PA11 : input (USART RX)
PA10 : output (USART TX)

The application board shall avoid driving these pins externally while the boot program is running (after a POR for example).

**Clock system**

CPU runs at 48MHz from Generic Clock Generator 0 on DFLL48M.

Generic Clock Generator 1 is using external 32kHz oscillator and is the source of DFLL48M.

USB and USART are using Generic Clock Generator 0 also.

**Memory Mapping**

Bootloader code will be located at 0x0 and executed before any application code.

Applications compiled to be executed along with the bootloader will start at 0x2000 for 8KB bootloaders and 0x1000 for 4KB bootloaders (see linker_scripts directory).

Before jumping to the application, the bootloader changes the VTOR register to use the interrupt vectors of the application (0x2000 or 0x1000 depending on bootloader size).

**SRAM Usage**

When modifying this bootloader, pay close attention to SRAM usage. On the 4KB SRAM versions,
DATA + BSS must be less than 1KB, so avoid increasing buffers and watch stack usage. This
bootloader places the stack (grows down) in the middle of the SRAM (normally, the stack
starts at the end of SRAM). This is so that an applet can be placed immediately after the top
of the system stack. The applet in this case is a very simple word copy function. However,
1KB is reserved for the applet, and there are two 64 byte data buffers placed after it. The
applet has its own stack at the top of RAM, but the word copy applet uses little/none of this.
The bossac tool is responsible for loading the applet. See Devices.h from the Bossa source.
