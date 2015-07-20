1- Prerequisites

gcc-arm-none-eabi 4.8.3-2014q1 from Arduino samd package
CMSIS 4.0.0-atmel from Arduino samd package
bossac 1.5-arduino-mattairtech-1 from MattairTech SAMD package or:
   * https://www.mattairtech.com/software/arduino/bossac-1.5-arduino-mattairtech-1-mingw32.zip (Windows 32 bit and 64 bit)
   * https://www.mattairtech.com/software/arduino/bossac-1.5-arduino-mattairtech-1-x86_64-linux-gnu.tar.bz2 (Linux 64 bit)
   * https://www.mattairtech.com/software/arduino/bossac-1.5-arduino-mattairtech-1-i686-linux-gnu.tar.bz2 (Linux 32 bit)
   * Use the bossac command from the Arduino SAMD package for OS X support. Only the 256 KB chip versions are supported

2- Selecting bootloader options

Set the define SAM_BA_INTERFACE in sam_ba_monitor.h to:
   * SAM_BA_UART_ONLY for only UART interface
   * SAM_BA_USBCDC_ONLY for only USB CDC interface
   * SAM_BA_BOTH_INTERFACES for enabling both the interfaces

All of these options will build an 8KB bootloader.
If you want to build a 4KB bootloader, you will need to:
   * Choose only one interface from above
   * Disable ARDUINO_EXTENDED_CAPABILITIES (sam_ba_monitor.h)
   * Disable BOOT_DOUBLE_TAP (main.h)

3- Makefile configuration

In the makefile, you must modify BLD_EXTA_FLAGS, NAME,
LINKER_SCRIPT, and STARTUP to match your cpu, as well as
the TOOLS_PATH variable (and possibly other include paths).
On Windows, you may need to change 'rm -f' to 'del' at the bottom of the makefile.

4- Boot Process

First, if BOOT_DOUBLE_TAP is defined, the bootloader checks if the reset button was pressed
twice in quick succession. If so, bootloader execution is resumed Second, the bootloader
checks the state of BOOT_LOAD_PIN (configurable by the user from main.h). If BOOT_LOAD_PIN
is pulled low, bootloader execution is resumed. Finally, the first location of application
is fetched and checked. If it is empty (0xFFFFFFFF), then bootloader execution is resumed.
Otherwise, it jumps to application and starts execution from there. The LED will light
during bootloader execution.

BOOT_LOAD_PIN is PA27 of SAMD21ExxA (arduino pin 27), which corresponds to Button A on the
MT-D21E board. LED_PIN is PA28 of SAMD21ExxA (arduino pin 28), which corresponds to the LED
on the MT-D21E board. It will turn on during bootloader operation (after the boot process).
If BOOT_DOUBLE_TAP is defined (default), the bootloader can be started by quickly tapping
two times on the reset button. BOOT_DOUBLE_TAP_ADDRESS must point to a free SRAM cell that
must not be touched from the loaded application.

5- Useful Info

When modifying this bootloader, pay close attention to SRAM usage. On the 4KB SRAM versions,
DATA + BSS must be less than 1KB, so avoid increasing buffers and watch stack usage. This
bootloader places the stack (grows down) in the middle of the SRAM (normally, the stack
starts at the end of SRAM). This is so that an applet can be placed immediately after the top
of the system stack. The applet in this case is a very simple word copy function. However,
1KB is reserved for the applet, and there are two 64 byte data buffers placed after it. The
applet has its own stack at the top of RAM, but the word copy applet uses little/none of this.
The bossac tool is responsible for loading the applet. See Devices.h from the Bossa source.
