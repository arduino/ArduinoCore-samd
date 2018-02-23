# Arduino Zero Bootloader

This bootloader is based on the Arduino Zero bootloader which is a part of the Arduino SAMD core. It
provides a USB-CDC and/or TTL serial communications interface to a host running the bossac command
line firmware programming utility (or the Arduino IDE) running on Windows, Linux, or OS X. Optionally,
SD Card firmware loading is supported, using SDSC or SDHC cards with a FAT16 or FAT32 filesystem.
This version adds support for the D11, L21, C21, and D51 microcontrollers. It also adds support for
four different clock sources (two external crystals and two internal oscillator options). There are
additional board definitions added, and binaries for most board/chip combinations are pre-built.

## Features

* SAM-BA USB CDC and UART interfaces with optional terminal mode
* SD Card interface (both USB CDC and SD Card support fits in 8KB)
* Four different clock sources (two external crystals and two internal oscillator options)
* Arduino IDE auto-reset and double-tap reset button support
* Arduino extended commands for faster firmware loading
* Supports the D21, L21, C21, and D11 SAM M0+ chips. Also supports D51 M4F chips.
* Bossac command line utility for Windows, Linux, and OS X


## SD Card Support

When SDCARD_ENABLED is defined, SD card bootloader support is compiled in. Support for
the other interfaces (USB CDC and/or UART) can be compiled in as well. External pins
can be defined to control the run-time behavior. If the SD bootloader is configured to
run, it always runs before the other intarfaces. If the SD bootloader does not find
the appropriate binary file on the SD card (or if the SD card does not exist), then
the other interface(s), if configured, will activate.

The SD bootloader supports MMCv3, SDv1 (SDSC), and SDv2 (SDSC and SDHC) cards with
FAT16 and FAT32 filesystem support. The filesystem is mounted read only, with no directory
support, so the files must be placed in the root directory. Only 8.3 format filenames are
supported (no long filenames, LFN). After reset, the SPI bus operates at 250KHz. After
successful sd card initialization, the speed is increased to 6MHz. While the sd bootloader
is running, the LED, if configured, will PWM fade quickly (~4Hz).

1. The bootloader can be run by Arduino IDE auto-reset, double-tap reset, and/or external
   pin state. It will run automatically if the first four bytes of FLASH is erased.
   SDCARD_ENABLED must be defined to compile in SD card bootloader support.
2. If SDCARD_USE_PIN1 or SDCARD_USE_PIN2 defined, read the pin(s) to determine if the sd
   bootloader should run (either pin enabled), and which file to load (UPDATE.BIN when
   PIN1 is enabled, UPDATE2.BIN if PIN2 is enabled). If neither pin is enabled, the
   enabled SAM-BA interface, if configured, will run. If a SAM-BA interface is not
   enabled, the LED will blink with status code LED_STATUS_NO_SAM_BA_INTERFACE. If the
   pins are not defined, then the SD bootloader will always run. The LED, if configured,
   will PWM fade quickly while the SD bootloader is running.
3. The SD bootloader initializes the SD card. If successful, it mounts the FAT16 or FAT32
   volume, if present. It then looks for the appropriate file (UPDATE.BIN or UPDATE2.BIN)
   in the root directory. If there is no SD card, no FAT16/FAT32 volume, or no file, the
   SD bootloader will exit, and the enabled SAM-BA interface, if configured, will run. If
   a SAM-BA interface is not enabled, the LED will blink with status code
   LED_STATUS_FILE_NOT_FOUND.
4. If SDCARD_VERIFICATION_DISABLED is not defined, the SD bootloader compares the update
   file on the SD card with the already installed firmware. If they already match, then
   the bootloader will jump to the firmware, unless SDCARD_AUTORUN_DISABLED is defined,
   in which case the LED will blink with status code LED_STATUS_FILE_ALREADY_MATCHES.
   If the update file differs from the installed firmware, continue to step 5.
5. The application section of the FLASH is erased. The selected update file is written
   to the FLASH. If the file exceeds the size of the FLASH, the LED will blink with
   status code LED_STATUS_FILE_TOO_LARGE.
6. If SDCARD_VERIFICATION_DISABLED is not defined, the SD bootloader verifies the FLASH
   by re-reading the update file. If the files differ, the application section of the
   FLASH is erased (so it will never be executed; the bootloader will still run) and the
   LED will blink with LED_STATUS_VERIFICATION_FAILURE. If verification is successful,
   the bootloader will jump to the firmware, unless SDCARD_AUTORUN_DISABLED is defined,
   in which case the LED will blink with status code LED_STATUS_SUCCESS.


### SD Card External Pins

If only SDCARD_USE_PIN1 is defined, then the SD Card bootloader will run depending
on the state of an external pin and the value of SDCARD_PIN1_POLARITY. The
SPI peripheral and SPI pins will only be setup if the SD card bootloader runs.
This is the default setting with the precompiled binaries that include SD Card support.

*Hint: When doing development with an SD card installed, and thus probably using the*
*SAM-BA interface, either ensure there is no UPDATE.BIN, or use pin 1 to skip the SD card*

Pin1            | Action
----------------|---------------------
Inactive        | Skip SD bootloader
Active          | Run SD bootloader

If both SDCARD_USE_PIN1 and SDCARD_USE_PIN2 are defined, then the SD Card bootloader
will run depending on the state of two external pins and the values of
SDCARD_PIN1_POLARITY and SDCARD_PIN1_POLARITY. The SPI peripheral and pins
will only be setup if the SD card bootloader runs. Note that if SDCARD_USE_PIN2
is defined, then SDCARD_USE_PIN1 must also be defined.

Pin1            | Pin2          | Action
----------------|---------------|-----------------------------------
Inactive        | Inactive      | Skip SD bootloader
Active          | Inactive      | Run SD bootloader w/ UPDATE.BIN
Inactive        | Active        | Run SD bootloader w/ UPDATE2.BIN
Active          | Active        | Skip SD bootloader

SDCARD_PIN1_POLARITY and SDCARD_PIN2_POLARITY can be set to PIN_POLARITY_ACTIVE_LOW
or PIN_POLARITY_ACTIVE_HIGH. If polarity is undefined, PIN_POLARITY_ACTIVE_HIGH
is used.

If neither SDCARD_USE_PIN1 or SDCARD_USE_PIN2 are defined, then the SD Card
bootloader will always run.


## Status LED

When the bootloader is running, and if both BOARD_LED_PIN and BOARD_LED_FADE_ENABLED are
defined, the LED will PWM fade in an "M-wave" pattern. When the SD bootloader is running,
the fading will be twice as fast (~4Hz) as the SAM-BA interface (USB CDC or UART at ~2Hz).
If BOARD_LED_FADE_ENABLED is not defined, the LED will simply turn on.

**Status codes**

Status Code                     | Blink Period  | Description
--------------------------------|---------------|---------------------------------------
LED_STATUS_SUCCESS              | 1Hz           | FLASH write success.
LED_STATUS_FILE_ALREADY_MATCHES | 2Hz           | File already the same.
LED_STATUS_FILE_NOT_FOUND       | 4Hz           | SD Card or update file not present.
LED_STATUS_NO_SAM_BA_INTERFACE  | 8Hz           | No SAM-BA interface is available (nothing to do).
LED_STATUS_FILE_TOO_LARGE       | 16Hz          | Update file exceeds size of application FLASH. Application FLASH erased.
LED_STATUS_VERIFICATION_FAILURE | 32Hz          | Application FLASH verification failure. Application FLASH erased.


## SAM-BA monitor commands

The SAM-BA interface, which is used with both USB CDC and UART,
can operate in either binary (default) or terminal mode. If
TERMINAL_MODE_ENABLED is defined, additional terminal handling code
(add prompt, add \n\r to EOL, format numbers, etc.) will be compiled
in. To switch to terminal mode, type 'T#' (you should then see a
prompt). Then, type 'V#' to show version information.

Command | Action                | Argument(s)           | Example
--------|-----------------------|-----------------------|------------------------
N       | Set Normal Mode       | No argument           | N#
T       | Set Terminal Mode     | No argument           | T#
O       | Write a byte          | Address, Value#       | O200001,CA#
o       | Read a byte           | Address,#             | o200001,#
H       | Write a half word     | Address, Value#       | H200002,CAFE#
h       | Read a half word      | Address,#             | h200002,#
W       | Write a word          | Address, Value#       | W200000,CAFEDECA#
w       | Read a word           | Address,#             | w200000,#
S       | Send a file           | Address,#             | S200000,#
R       | Receive a file        | Address, NbOfBytes#   | R200000, 1234#
G       | Go                    | Address#              | G200200#
V       | Display version       | No argument           | V#

For more information on SAM-BA, see (especially pages 10 and 11): 
http://www.atmel.com/Images/Atmel-42438-SAM-BA-Overview-and-Customization-Process_ApplicationNote_AT09423.pdf


## D51 Clock Configuration

The D51 can run at either 120MHz or 48MHz. When running at 120MHz,
two or three additional clock generators are used. Two of these
generate 48MHz and 96MHz. The USB peripheral can only run at 48MHz,
and many peripherals (ie: SERCOM) have a limit of 100MHz, so these
generators are used for them. Because 120MHz cannot be divided down
to 48MHz or 96MHz using the GCLK dividers, the second PLL is used.
Thus, with all clock source configurations, when the cpu runs at
120MHz both PLLs are enabled. When the cpu runs at 48MHz, only the
first PLL is enabled and only when using an external crystal. Use
48MHz to reduce power consumption.


## Bootloader Binaries

The bootloaders/zero/binaries directory contains the SAM-BA m0+
bootloaders built by the build_all_bootloaders.sh script from
the 'MattairTech SAM M0+ Boards' Arduino core, which is available
at https://github.com/mattairtech/ArduinoCore-samd. Each board
and chip combination has two bootloaders available:

* SAM-BA interface only
  * This is the bootloader that is installed by the Arduino IDE
  * USB CDC only for all MattairTech boards
  * Both USB CDC and UART for most Arduino boards
  * The Generic board variants minimize external pin usage
    * Only the SAM-BA interface pins are used (no crystal, LED, etc.)
  * Filename is: sam_ba_$(BOARD_ID)_$(MCU)

* SAM-BA interface and SD Card interface
  * USB CDC only for all Arduino and most MattairTech boards
  * No SAM-BA interface for the D11 chips
  * All board variants define SDCARD_USE_PIN1 (except D11)
  * The Generic board variants use the LED
  * SDCARD_AUTORUN_DISABLED is defined
  * Filename is: sam_ba_sdcard_$(BOARD_ID)_$(MCU)

Please see the appropriate board_definitions file to see which pins
are used for the SD card. Note that the D51 uses different pins.


### MattairTech Boards

MattairTech boards are all configured with only one interface:
SAM_BA_USBCDC_ONLY (except C21, which uses SAM_BA_UART_ONLY).
CLOCKCONFIG_CLOCK_SOURCE is set to CLOCKCONFIG_INTERNAL_USB
(CLOCKCONFIG_INTERNAL for the C21). Only the main LED is defined.
BOOT_LOAD_PIN is not defined, but BOOT_DOUBLE_TAP_ENABLED is.
When the SD Card interface is enabled, SDCARD_AUTORUN_DISABLED and
SDCARD_USE_PIN1 are defined.

### MattairTech/Generic D11 Boards

All boards are configured with only the USB CDC interface, except
when SDCARD_ENABLED is defined, then only the SD Card interface is
enabled. ARDUINO_EXTENDED_CAPABILITIES is set to 0 (disabled).
TERMINAL_MODE_ENABLED is not defined. As of 1.6.8-beta-b2,
USB_VENDOR_STRINGS_ENABLED is now defined. BOOT_LOAD_PIN is not
defined, but BOOT_DOUBLE_TAP_ENABLED is. When the SD Card interface is
enabled, SDCARD_AUTORUN_DISABLED is defined (but not SDCARD_USE_PIN1).

### Arduino/Genuino Boards

Most Arduino/Genuino boards are configured with both interfaces,
except when SDCARD_ENABLED is defined, then only USB CDC is enabled.
CLOCKCONFIG_CLOCK_SOURCE is set to CLOCKCONFIG_32768HZ_CRYSTAL.
All LEDs that are installed for each board are defined (and some
have LED_POLARITY_LOW_ON set). BOOT_LOAD_PIN is not defined, but
BOOT_DOUBLE_TAP_ENABLED is. When the SD Card interface is enabled,
SDCARD_AUTORUN_DISABLED and SDCARD_USE_PIN1 are defined.

### Generic Boards

The generic boards are all configured to minimize external hardware
requirements. Only one interface is enabled: SAM_BA_USBCDC_ONLY
(except C21, which uses SAM_BA_UART_ONLY). CLOCKCONFIG_CLOCK_SOURCE
is set to CLOCKCONFIG_INTERNAL_USB (CLOCKCONFIG_INTERNAL for the C21),
so no crystal is required. No LEDs are defined. BOOT_LOAD_PIN is not
defined, but BOOT_DOUBLE_TAP_ENABLED is, since it uses the reset pin.
When the SD Card interface is enabled, SDCARD_AUTORUN_DISABLED and
SDCARD_USE_PIN1 are defined.


## Installation

### Driver Installation

The bootloader requires a USB CDC driver to be installed when using the SAM-BA USB CDC interface.
This driver is the same as the one used by the MattairTech SAM M0+ Core.

**See the MattairTech SAM M0+ Core [README.md](https://github.com/mattairtech/ArduinoCore-samd/tree/master/README.md) "Driver Installation" for installation instructions.**


### Bootloader Firmware Installation

#### Bootloader Installation Using the Arduino IDE

1. If you do not already have the MattairTech SAM M0+ core installed, see SAM M0+ Core Installation above.
2. Plug in the SAM M0+ board. The bootloader must be running to (press reset twice within 500ms).
3. Plug an Atmel ICE into USB, then connect it to the powered SAM M0+ board. A green LED should light on the Atmel ICE.
4. Click Tools->Programmer->Atmel ICE.
5. Click Tools->Board->MattairTech MT-D21E (or whichever board you are using).
6. Click Tools->Microcontroller and select your MCU (if menu present).
7. Click Tools->Burn Bootloader. Ignore any messages about not supporting shutdown or reset.
8. Continue with driver installation above.

**A running sketch *may* interfere with the bootloader installation process. Be sure you are running the existing bootloader or using a blank chip.**

#### Bootloader Installation Using Another Tool (ie: Atmel Studio, openocd)

1. Download the bootloader from **https://www.mattairtech.com/software/arduino/SAM-BA-bootloaders-zero-mattairtech.zip**.
2. Unzip to any directory. Be sure that a bootloader is available for your particular MCU.
3. Follow the procedures for your upload tool to upload the firmware.
   * Perform a chip erase first. Be sure no BOOTPROT bits are set.
   * Install the binary file to 0x00000000 of the FLASH.
   * You can optionally set the BOOTPROT bits to 8KB (or 4KB for the MT-D11). The Arduino installation method does not set these.
   * You can optionally set the EEPROM bits or anything else. The Arduino installation method uses factory defaults.
4. Continue with driver installation above.


### Bossac Utility

This version of the bootloader requires bossac (1.7.0-mattairtech-2) or above.

**See the MattairTech SAM M0+ Core [README.md](https://github.com/mattairtech/ArduinoCore-samd/tree/master/README.md) "Driver Installation" for installation instructions.).**

#### Bossac Utility Installation

If using the Arduino IDE to upload firmware, then this will be installed automatically when intalling the core.
If using Bossac standalone, download bossac directly at:

* https://www.mattairtech.com/software/arduino/bossac-1.7.0-mattairtech-2-mingw32.tar.gz (Windows 32 bit and 64 bit)
* https://www.mattairtech.com/software/arduino/bossac-1.7.0-mattairtech-2-x86_64-linux-gnu.tar.gz (Linux 64 bit)
* https://www.mattairtech.com/software/arduino/bossac-1.7.0-mattairtech-2-i686-linux-gnu.tar.gz (Linux 32 bit)
* https://www.mattairtech.com/software/arduino/bossac-1.7.0-mattairtech-2-x86_64-apple-darwin.tar.gz (OS X 64 bit)

Linux 64 bit users can also download Bossa (GUI) and bossash (shell) from:

* https://www.mattairtech.com/software/arduino/Bossa-1.7.0-mattairtech-2-x86_64-linux-gnu.tar.gz (Linux 64 bit)

Note that the SAM-BA tools from Atmel will not work, and the version of bossac from the Arduino
SAMD Core currently only supports the D21.


#### Using Bossac Standalone

TODO: Update https://www.mattairtech.com/software/SAM-BA-bootloader-test-firmware.zip with new chips (L21, C21 and D51).

When using Bossac standalone, you will need to ensure that your application starts at 0x00002000 for 8 KB bootloaders,
and 0x00001000 for 4 KB bootloaders. This is because the bootloader resides at 0x00000000. This can be accomplished
by passing the following flag to the linker (typically LDFLAGS in your makefile; adjust for your bootloader size):

```
Wl,sectionstart=.text=0x2000
```

You can also use a linker script. See the MattairTech SAM M0+ package for examples.
Be sure to generate and use a binary file. Many makefiles are set up to generate an elf, hex, and bin already.

As an example, bossac will be used to upload the test firmware (blink sketch):

1. Download firmware from https://www.mattairtech.com/software/SAM-BA-bootloader-test-firmware.zip and unzip.
2. If you have not already installed the bootloader driver, see Driver Installation above.
3. Be sure there is a binary that matches your chip. On the command line (change the binary to match yours):

```
bossac.exe -d --port=COM5 -U true -i -e -w -v Blink_Demo_ATSAMD21E18A.bin -R
```
4. On Linux --port might be /dev/ttyACM0. If the device is not found, remove the --port argument for auto-detection.
5. See http://manpages.ubuntu.com/manpages/vivid/man1/bossac.1.html for details.
6. The board should reset automatically and the sketch should be running.


## Building Bootloader

### Prerequisites for Building

For all builds and platforms you will need to have the Arduino IDE installed as well as the packages
for both Arduino SAMD Boards and for MattairTech SAM M0+ Boards, which provides the needed dependencies
(CMSIS, CMSIS-Atmel, and the compiler toolchain: arm-none-eabi-gcc), which can be installed using the
Arduino IDE Boards Manager. If you do not wish to install the MattairTech SAM M0+ Boards core, then the
arm-none-eabi-gcc, CMSIS, and openocd packages are included with the stock Arduino SAMD. However, you
will still need to download bossac (see above) and CMSIS-Atmel from MattairTech:

* https://www.mattairtech.com/software/arduino/CMSIS-Atmel-1.0.0-mattairtech-2.tar.gz

Then install to ~/arduino15/packages/MattairTech_Arduino/tools/CMSIS-Atmel (or similar based on your OS)
and rename the folder in CMSIS-Atmel from CMSIS to 1.0.0-mattairtech-2.

This project uses a Makefile, which is in the root zero directory. However, you will need a make program:

#### Windows

* Native command line
Make binary can be obtained here: http://gnuwin32.sourceforge.net/packages/make.htm
Be sure that no other version of make is in your PATH (ie: MinGW).

* Cygwin/MSys/MSys2/Babun/etc...
It is available natively in all distributions.

#### Linux

Make is usually available by default.

#### OS X

Make is available through XCode package.


### Makefile Configuration

The section between 'Begin Configuration' and 'End Configuration' in the Makefile should be edited.
Set SDCARD, BOARD_ID, and MCU to one of the values listed in the comments.

#### SD Card support

  * SDCARD_DISABLED, SDCARD_ENABLED

This can also be set in the relevant board_definitions file. It is present in the makefile to
allow the build_all_bootloaders.sh script to select SD Card support.

#### Boards definitions:

* Xeno, MT_D21E_rev_A, MT_D21E_rev_B, MT_D11, MT_D21J
* arduino_zero, arduino_mkrzero, arduino_mkr1000, genuino_mkr1000, genuino_zero
* Generic_x21E, Generic_x21G, Generic_x21J, Generic_D11D14AM, Generic_D11D14AS, Generic_D11C14A

#### MCU definitions:

* SAMD21J: SAMD21J18A, SAMD21J17A, SAMD21J16A, SAMD21J15A
* SAMD21G: SAMD21G18A, SAMD21G17A, SAMD21G16A, SAMD21G15A
* SAMD21E: SAMD21E18A, SAMD21E17A, SAMD21E16A, SAMD21E15A
* SAML21J: SAML21J18B, SAML21J17B, SAML21J16B
* SAML21G: SAML21G18B, SAML21G17B, SAML21G16B
* SAML21E: SAML21E18B, SAML21E17B, SAML21E16B, SAML21E15B
* SAMC21J: SAMC21J18A, SAMC21J17A, SAMC21J16A, SAMC21J15A
* SAMC21G: SAMC21G18A, SAMC21G17A, SAMC21G16A, SAMC21G15A
* SAMC21E: SAMC21E18A, SAMC21E17A, SAMC21E16A, SAMC21E15A
* SAMD11:  SAMD11D14AM, SAMD11C14A, SAMD11D14AS
* SAMD51G: SAMD51G18A, SAMD51G19A
* SAMD51J: SAMD51J18A, SAMD51J19A, SAMD51J20A
* SAMD51N: SAMD51N19A, SAMD51N20A
* SAMD51P: SAMD51P19A, SAMD51P20A


### Board Configuration

Configuration for each board is available in the board_definitions directory.
Each board has a file named board_definitions_BOARD_NAME.h, where BOARD_NAME is
listed above in the makefile configuration. The following options are available:

#### Sizes of Options

Compilation Option                      | Size (Bytes)
----------------------------------------|--------------------------
*Core*                                  | ~1200*
SAM_BA_CDC                              | 1148*
SAM_BA_UART                             | 1108*
*SAM-BA Monitor*                        | 1200*
SDCARD_ENABLED                          | 2788*
SDCARD_VERIFICATION_DISABLED            | 284
USB_VENDOR_STRINGS_ENABLED              | 228
TERMINAL_MODE_ENABLED                   | 228
BOARD_LED_FADE_ENABLED                  | 160
SAM_BA_INTERFACE_USE_PIN                | 100
BOOT_DOUBLE_TAP_ENABLED                 | 96
BOOT_LOAD_PIN                           | 84
SDCARD_USE_PIN1 & SDCARD_USE_PIN2       | 92
SDCARD_USE_PIN1                         | 60
ARDUINO_EXTENDED_CAPABILITIES (X/Y/Z)   | 904*
*X (Chip Erase) & Y (Write FLASH)*      | 120*
*Z (CRC Verification)*                  | 248*
*crc16Table*                            | 512*

* SAM_BA_CDC and SAM_BA_UART automatically pull in SAM-BA Monitor
* ARDUINO_EXTENDED_CAPABILITIES and SDCARD_ENABLED include X and Y functions
* ARDUINO_EXTENDED_CAPABILITIES and SAM_BA_UART include crc16Table
* ARDUINO_EXTENDED_CAPABILITIES includes Z function

#### Example Compiled Binary Size

Precompiled Bootloader                  | Size (Bytes)
----------------------------------------|--------------------------
Default 8KB with CDC only               | 5196
8KB with CDC and SDCARD                 | 8052
Default 4KB with CDC only               | 4036
4KB with SDCARD only                    | 4044


### TERMINAL_MODE_ENABLED

The SAM-BA interface, which is used with both USB CDC and UART (TTL
serial), can operate in either binary (default) or terminal mode. If
TERMINAL_MODE_ENABLED is defined, additional terminal handling code
(add prompt, add \n\r to EOL, number formatting, etc.) will be compiled
in. To switch to terminal mode, type 'T#' (you should then see a prompt).
Then, type 'V#' to show version information. See SAM-BA monitor commands.
Size: ~228B. Enabled by default. Disable with 4KB bootloader.

### SDCARD_ENABLED

If SDCARD_ENABLED is defined, SD card bootloader support is compiled in.
See "SD Card Bootloader" section. This define can also be set from the
makefile (so it can be used with the build_all_bootloaders.sh script).
Size: ~2788B. Disabled by default. Available with 4KB bootloader.

### SDCARD_SPI_SERCOM_INSTANCE
#### SDCARD_SPI_PAD_SETTINGS, SDCARD_SPI_PAD0, SDCARD_SPI_PAD1, SDCARD_SPI_PAD2, SDCARD_SPI_PAD3

If SDCARD_ENABLED is defined, then all SDCARD_SPI_* defines must also be set.
When setting SDCARD_SPI_PADx defines, consult the appropriate header file
from CMSIS-Atmel (ie: ~/arduino15/packages/MattairTech_Arduino/tools/CMSIS-
Atmel/1.0.0-mattairtech-2/CMSIS/Device/ATMEL/sam<d21|d51|c21|l21|d11>/include/
<YOUR_CHIP>.h). SDCARD_SPI_PAD_SETTINGS values are in SDCard/diskio.h.
When using SDCARD_USE_PIN1 or SDCARD_USE_PIN2, the SPI peripheral and
associated pins are only initialized if either pin is active.

### SDCARD_SPI_CS_PORT, SDCARD_SPI_CS_PIN

If SDCARD_ENABLED is defined, then SDCARD_SPI_CS_PORT and SDCARD_SPI_CS_PIN
must also be defined. PORT can be 0 (Port A) or 1 (Port B).

### SDCARD_USE_PIN1
#### SDCARD_PIN1_POLARITY, SDCARD_PIN1_PORT, SDCARD_PIN1_PIN, SDCARD_PIN1_CONFIG

### SDCARD_USE_PIN2
#### SDCARD_PIN2_POLARITY, SDCARD_PIN2_PORT, SDCARD_PIN2_PIN, SDCARD_PIN2_CONFIG

If SDCARD_ENABLED is defined, then SDCARD_USE_PIN1 and SDCARD_USE_PIN2 can
optionally be defined. When SDCARD_USE_PIN2 is defined, SDCARD_USE_PIN1 must
also be defined. See "SD Card External Pins" section for more information. PORT
can be 0 (Port A) or 1 (Port B). Polarity can be PIN_POLARITY_ACTIVE_LOW or
PIN_POLARITY_ACTIVE_HIGH. Config can be INPUT, INPUT_PULLUP, or INPUT_PULLDOWN.
Size: ~60B for SDCARD_USE_PIN1, ~92B for both pins. By default, only pin1 used.

### SDCARD_VERIFICATION_DISABLED

If SDCARD_VERIFICATION_DISABLED is defined, then verification of the FLASH
after programming will not occur, nor will the initial check to see if the
FLASH contents are already the same as the file.
Size: ~284B. By default, this is not defined, so verification will be enabled.

### SDCARD_AUTORUN_DISABLED

If SDCARD_AUTORUN_DISABLED is defined, then the SD card bootloader will not
automatically run the firmware that was just installed. Instead, the LED will
blink with status code LED_STATUS_SUCCESS. This option also applies when the
binary file on the SD card already matches the installed firmware. In this
case, the LED will blink with status code LED_STATUS_FILE_ALREADY_MATCHES.
By default, SDCARD_AUTORUN_DISABLED is defined.

### SDCARD_FILENAME_PRIMARY
### SDCARD_FILENAME_SECONDARY

Two different binary files can be loaded, depending on external pin settings.
By default, the filenames are UPDATE.BIN and UPDATE2.BIN, but these can be
overridden by defining SDCARD_FILENAME_PRIMARY and SDCARD_FILENAME_SECONDARY.
If both pins are configured, SDCARD_FILENAME_PRIMARY (UPDATE.BIN) will be
loaded when PIN1 is enabled, and SDCARD_FILENAME_PRIMARY (UPDATE2.BIN) is
loaded when PIN2 is enabled. If only one pin or no pin is configured, only
SDCARD_FILENAME_PRIMARY is loaded.

### SAM_BA_INTERFACE

Set SAM_BA_INTERFACE to SAM_BA_USBCDC_ONLY, SAM_BA_UART_ONLY, SAM_BA_NONE, or
SAM_BA_BOTH_INTERFACES. With 4KB bootloaders, select only one interface (except
when using SDCARD_ENABLED, then set SAM_BA_INTERFACE to SAM_BA_NONE). The C21
lacks USB, so set to SAM_BA_UART_ONLY in this case. By default,
SAM_BA_USBCDC_ONLY is set (SAM_BA_UART_ONLY with the C21).

### SAM_BA_INTERFACE_USE_PIN
#### SAM_BA_INTERFACE_PIN_POLARITY, SAM_BA_INTERFACE_PIN_PORT, SAM_BA_INTERFACE_PIN_PIN, SAM_BA_INTERFACE_PIN_CONFIG

If SAM_BA_INTERFACE_USE_PIN is defined, then the associated pin controls which
SAM-BA interface is used (if SAM_BA_BOTH_INTERFACES is defined). If only one
interface is used, then the pin acts as an enable. In both cases, the value of
SAM_BA_INTERFACE_PIN_POLARITY controls the polarity, with values of
PIN_POLARITY_ACTIVE_LOW or PIN_POLARITY_ACTIVE_HIGH for a single interface, and
PIN_POLARITY_USBCDC_LOW or PIN_POLARITY_USBCDC_HIGH when both interfaces are
enabled. PORT can be 0 (Port A) or 1 (Port B). Config can be INPUT, INPUT_PULLUP,
or INPUT_PULLDOWN.The USB/UART peripheral and pins will not be setup if the
device is not selected/enabled. If no interface is selected by the pin, the LED
will blink with status code LED_STATUS_NO_SAM_BA_INTERFACE.
Size: ~100B. By default, SAM_BA_INTERFACE_USE_PIN is not defined.

### ARDUINO_EXTENDED_CAPABILITIES

If ARDUINO_EXTENDED_CAPABILITIES is defined and set to 1, 3 additional commands
will become available which will speed up programming when using the Arduino
IDE or the bossac tool standalone. Set to 0 with 4KB bootloaders.
Size: ~904B. This is defined and set to 1 by default (except with 4KB).

Arduino Extended Capabilities:

* X: Erase the flash memory starting from ADDR to the end of flash.
* Y: Write the content of a buffer in SRAM into flash memory.
* Z: Calculate the CRC for a given area of memory.

### CLOCKCONFIG_CLOCK_SOURCE

The clock source must be chosen by setting CLOCKCONFIG_CLOCK_SOURCE to
CLOCKCONFIG_32768HZ_CRYSTAL, CLOCKCONFIG_HS_CRYSTAL, CLOCKCONFIG_INTERNAL,
or CLOCKCONFIG_INTERNAL_USB. If CLOCKCONFIG_32768HZ_CRYSTAL or
CLOCKCONFIG_HS_CRYSTAL is defined, then the PLL will be used. If
CLOCKCONFIG_HS_CRYSTAL is defined, then HS_CRYSTAL_FREQUENCY_HERTZ must
also be defined with the crystal frequency in Hertz. CLOCKCONFIG_INTERNAL
uses the DFLL in open-loop mode, except with the C21 which lacks a DFLL, so
the internal 48MHz RC oscillator is used instead. CLOCKCONFIG_INTERNAL_USB
can be defined for the D21, D11, L21, or D51. It will also use the DFLL in
open-loop mode, except when connected to a USB port with data lines (and
not suspended), where it will calibrate against the USB SOF signal.

### HS_CRYSTAL_FREQUENCY_HERTZ

If CLOCKCONFIG_HS_CRYSTAL is defined, then HS_CRYSTAL_FREQUENCY_HERTZ
must also be defined with the external crystal frequency in Hertz.
Current MattairTech boards use 16MHz (12MHz or 24MHz on future boards).

### PLL_FRACTIONAL_ENABLED

If the PLL is used (CLOCKCONFIG_32768HZ_CRYSTAL, or CLOCKCONFIG_HS_CRYSTAL
defined), then PLL_FRACTIONAL_ENABLED can be defined, which will result in
a more accurate 48MHz output frequency at the expense of increased jitter.

### PLL_FAST_STARTUP

If both PLL_FAST_STARTUP and CLOCKCONFIG_HS_CRYSTAL are defined, the crystal
will be divided down to 1MHz - 2MHz, rather than 32KHz - 64KHz, before being
multiplied by the PLL. This will result in a faster lock time for the PLL,
however, it will also result in a less accurate PLL output frequency if the
crystal is not divisible (without remainder) by 1MHz. In this case, define
PLL_FRACTIONAL_ENABLED as well. By default, this is disabled. This mode is
also useful for USB host mode applications. See datasheet USB electrical
characteristics.

### VARIANT_MCK

Master clock frequency (also Fcpu frequency), set to 48000000ul for all MCUs,
except the D51, which can be either 48000000ul or 120000000ul.

### NVM_SW_CALIB_DFLL48M_FINE_VAL

The fine calibration value for DFLL open-loop mode is defined here.
The coarse calibration value is loaded from NVM OTP (factory calibration values).

### USB_VENDOR_STRINGS_ENABLED
#### STRING_MANUFACTURER, STRING_PRODUCT

If USB_VENDOR_STRINGS_ENABLED is defined, then STRING_MANUFACTURER and
STRING_PRODUCT will be sent to the host.
Size: ~228B. By default, USB_VENDOR_STRINGS_ENABLED is defined (including 4KB).

### USB_VID_HIGH
#### USB_VID_LOW, USB_PID_HIGH, USB_PID_LOW

If USB CDC is used, then the USB vendor ID (VID) and product ID (PID) must be set.

### BOOT_USART_SERCOM_INSTANCE
#### BOOT_USART_PAD_SETTINGS, BOOT_USART_PAD3, BOOT_USART_PAD2, BOOT_USART_PAD1, BOOT_USART_PAD0

BOOT_USART_SERCOM_INSTANCE must be a single digit representing the SERCOM number.
See board_driver_serial.h for BOOT_USART_PAD_SETTINGS values. When setting
BOOT_USART_PADx defines, consult the appropriate header file from CMSIS-Atmel (ie:
~/arduino15/packages/MattairTech_Arduino/tools/CMSIS-Atmel/1.0.0-mattairtech-2/
CMSIS/Device/ATMEL/sam<d21|d51|c21|l21|d11>/include/<YOUR_CHIP>.h). Use PINMUX_UNUSED
if not used. By default, this interface is not enabled (except with the C21).

### BOOT_DOUBLE_TAP_ENABLED

If BOOT_DOUBLE_TAP_ENABLED is defined the bootloader is started by quickly
tapping two times on the reset button (within 1/2 second).
Size: ~96B. Enabled by default.

### BOOT_LOAD_PIN_ENABLED
#### BOOT_LOAD_PIN, BOOT_LOAD_PIN_PORT, BOOT_LOAD_PIN_POLARITY, BOOT_LOAD_PIN_CONFIG

If BOOT_LOAD_PIN_ENABLED is defined, the bootloader is started if the selected
pin is active after reset. There is a 10ms delay before testing the pin to
allow time for debouncing capacitors to charge (ie: button use). PORT can be 0
(Port A) or 1 (Port B). Polarity can be PIN_POLARITY_ACTIVE_LOW or
PIN_POLARITY_ACTIVE_HIGH. Config can be INPUT, INPUT_PULLUP, or INPUT_PULLDOWN.
Size: ~84B. Disabled by default.

### BOARD_LED_FADE_ENABLED

If BOARD_LED_FADE_ENABLED is defined, then the main LED produces a PWM fade in an
"M-wave" pattern, otherwise, it simply turns on (if enabled). When the SD bootloader
is running, the fading will be twice as fast as the SAM-BA interface (USB CDC or UART).
Size: ~160B. Enabled by default.

### BOARD_LED_PORT, BOARD_LED_PIN, BOARD_LED_POLARITY
#### BOARD_LEDRX_PORT, BOARD_LEDRX_PIN, BOARD_LEDRX_POLARITY
#### BOARD_LEDTX_PORT, BOARD_LEDTX_PIN, BOARD_LEDTX_POLARITY

If the LED PORT is defined, then the LED on the associated pin is enabled.
Polarity can be either LED_POLARITY_HIGH_ON or LED_POLARITY_LOW_ON.
By default, only BOARD_LED is enabled.


### Building a 4KB bootloader

Only one interface (USB CDC, UART, or SD Card) should be selected.
ARDUINO_EXTENDED_CAPABILITIES should be set to 0. All external pins
should be undefined, as well as TERMINAL_MODE_ENABLED. As of
1.6.8-beta-b2, USB_VENDOR_STRINGS_ENABLED can now be defined.
BOOT_DOUBLE_TAP_ENABLED can be defined.


### Building

If not specified the makefile builds for **MT_D21E_rev_B**:

```
make
```

If you want to make a custom bootloader for a derivative board you must supply all the
necessary information in a `board_definitions_xxx.h` file, and add the corresponding case in
`board_definitions.h`. For example, for the **Generic_x21J** board with a **SAMD21J18A** MCU,
use `board_definitions_Generic_x21J.h` and build with the following command:

```
BOARD_ID=Generic_x21J MCU=SAMD21J18A SDCARD=SDCARD_ENABLED make clean all
```

which will produce a binary named sam_ba_sdcard_Generic_x21J_SAMD21J18A.bin


## Possible Future Additions/Changes

* IP Protection / Security options (control/eliminate direct register access, applet loading/executing, and reading/checksumming of flash contents)
* Variable bootloader sizes for each chip


## Technical Details

**TTL Serial**

The TX and RX pins are defined in the relevant board_definitions_* file. The baud rate is 115200 (8N1).

**Arduino IDE Auto-Reset**

When the Arduino IDE initiates the bootloader, the following procedure is used:

1. The IDE opens and closes the USB serial port at a baud rate of 1200bps. This triggers a “soft erase” procedure.
2. The first row of application section flash memory is erased by the MCU. If it is interrupted for any reason, the erase procedure will likely fail.
3. The board is reset. The bootloader (which always runs first) detects the blank flah row, so bootloader operation resumes.
4. Opening and closing the port at a baud rate other than 1200bps will not erase or reset the SAM M0+.

**Boot Condition Test Sequence**

First, the start location of the sketch is fetched and checked. If it
is empty (0xFFFFFFFF), then bootloader execution is resumed. Note that
when Arduino auto-reset (into bootloader) is initiated, the first flash
row is erased, so the bootloader will always run after reset until a
new sketch is transferred. Next, it checks for the double-tap reset
condition. Then, it checks the boot pin state (after a 10ms delay to
allow debounce capacitor charging). If no bootloader entry condition
is present, it jumps to the application and starts execution from there.

**Pinmap**

The following pins are used by the program :
PA25 : input/output (USB DP)
PA24 : input/output (USB DM)
The serial pins used are defined in each board_definitions file.
The application board shall avoid driving these pins externally
while the boot program is running (after a POR for example).

**Memory Mapping**

Bootloader code will be located at 0x0 and executed before any
application code. Applications compiled to be executed along with the
bootloader will start at 0x2000 for 8KB bootloaders and 0x1000 for 4KB
bootloaders (see linker_scripts directory). Before jumping to the
application, the bootloader changes the VTOR register to use the interrupt
vectors of the application (0x2000 or 0x1000 depending on bootloader size).

**SRAM Usage**

When modifying this bootloader, pay close attention to SRAM usage. On the 4KB SRAM versions,
DATA + BSS must be less than 1KB, so avoid increasing buffers and watch stack usage. This
bootloader places the stack (grows down) in the middle of the SRAM (normally, the stack
starts at the end of SRAM). This is so that an applet can be placed immediately after the top
of the system stack. The applet in this case is a very simple word copy function. However,
1KB is reserved for the applet, and there are two 64 byte data buffers placed after it. The
applet has its own stack at the top of RAM, but the word copy applet uses little/none of this.
The bossac tool is responsible for loading the applet. See Devices.h from the Bossa source.

**A running sketch *may* interfere with the bootloader installation process. Be sure you are running the existing bootloader or using a blank chip.**


## License

Copyright (c) 2015 Arduino LLC.  All right reserved.
Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.
Copyright (c) 2017 MattairTech LLC. All right reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

### Petit FatFS

Petit FatFs module is an open source software to implement FAT file system to
small embedded systems. This is a free software and is opened for education,
research and commercial developments under license policy of following trems.

Copyright (C) 2014, ChaN, all right reserved.

* The Petit FatFs module is a free software and there is NO WARRANTY.
* No restriction on use. You can use, modify and redistribute it for
  personal, non-profit or commercial use UNDER YOUR RESPONSIBILITY.
* Redistributions of source code must retain the above copyright notice.
