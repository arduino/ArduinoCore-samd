# Arduino Zero Bootloader

## Prerequisites for Building

The project build is based on Makefile system.
Makefile is present at project root and try to handle multi-platform cases.

Multi-plaform GCC is provided by ARM here: https://launchpad.net/gcc-arm-embedded/+download

Atmel Studio contains both make and ARM GCC toolchain. You don't need to install them in this specific use case.

For all builds and platforms you will need to have the Arduino IDE installed and the board support
package for "Arduino SAMD Boards (32-bits ARM Cortex-M0+)". You can install the latter
from the former's "Boards Manager" UI.

This version of the bootloader requires bossac (1.7.0-mattairtech-1)
from the package "MattairTech SAM M0+ Boards" or download bossac directly:
* https://www.mattairtech.com/software/arduino/bossac-1.7.0-mattairtech-1-mingw32.tar.gz (Windows 32 bit and 64 bit)
* https://www.mattairtech.com/software/arduino/bossac-1.7.0-mattairtech-1-x86_64-linux-gnu.tar.gz (Linux 64 bit)
* https://www.mattairtech.com/software/arduino/bossac-1.7.0-mattairtech-1-i686-linux-gnu.tar.gz (Linux 32 bit)
* https://www.mattairtech.com/software/arduino/bossac-1.7.0-mattairtech-1-x86_64-apple-darwin.tar.gz (OS X 64 bit)

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


## Makefile Configuration

The section between 'Begin Configuration' and 'End Configuration' in the Makefile should be edited.
Set BOARD_ID and MCU to a chip listed in the comments.

### Boards definitions:
* MT_D21E_rev_A, MT_D21E_rev_B, MT_D11, MT_D21J
* arduino_zero, arduino_mkrzero, arduino_mkr1000, genuino_mkr1000, genuino_zero
* Generic_x21E, Generic_x21G, Generic_x21J, Generic_D11D14AM, Generic_D11D14AS, Generic_D11C14A

### MCU definitions:
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


## Board Configuration

Configuration for each board is available in the board_definitions directory.
Each board has a file named board_definitions_BOARD_NAME.h, where BOARD_NAME is
listed above in the makefile configuration. The following options are available:

### SAM_BA_INTERFACE
Set SAM_BA_INTERFACE to SAM_BA_USBCDC_ONLY, SAM_BA_UART_ONLY, or
SAM_BA_BOTH_INTERFACES. Select only one interface with 4KB bootloaders.
The C21 lacks USB, so set to SAM_BA_UART_ONLY in this case.

### ARDUINO_EXTENDED_CAPABILITIES
If ARDUINO_EXTENDED_CAPABILITIES is defined and set to 1, 3 additional commands
will become available which will speed up programming when using the Arduino
IDE or the bossac tool standalone. Set to 0 with 4KB bootloaders.

### CLOCKCONFIG_CLOCK_SOURCE
The clock source must be chosen by setting CLOCKCONFIG_CLOCK_SOURCE to
CLOCKCONFIG_32768HZ_CRYSTAL, CLOCKCONFIG_HS_CRYSTAL, CLOCKCONFIG_INTERNAL,
or CLOCKCONFIG_INTERNAL_USB. If CLOCKCONFIG_32768HZ_CRYSTAL or
CLOCKCONFIG_HS_CRYSTAL is defined, then the PLL will be used. If
CLOCKCONFIG_HS_CRYSTAL is defined, then HS_CRYSTAL_FREQUENCY_HERTZ must
also be defined with the crystal frequency in Hertz. CLOCKCONFIG_INTERNAL
uses the DFLL in open-loop mode, except with the C21 which lacks a DFLL, so
the internal 48MHz RC oscillator is used instead. CLOCKCONFIG_INTERNAL_USB
can be defined for the D21, D11, or L21. It will also use the DFLL in
open-loop mode, except when connected to a USB port with data lines (and
not suspended), where it will calibrate against the USB SOF signal.

### HS_CRYSTAL_FREQUENCY_HERTZ
If CLOCKCONFIG_HS_CRYSTAL is defined, then HS_CRYSTAL_FREQUENCY_HERTZ
must also be defined with the external crystal frequency in Hertz.

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
Master clock frequency (also Fcpu frequency), set to 48000000ul only for now.

### NVM_SW_CALIB_DFLL48M_FINE_VAL
The fine calibration value for DFLL open-loop mode is defined here.
The coarse calibration value is loaded from NVM OTP (factory calibration values).

### USB_VENDOR_STRINGS_ENABLED
If USB_VENDOR_STRINGS_ENABLED is defined, then STRING_MANUFACTURER and
STRING_PRODUCT will be sent to the host. Do not enable with 4KB bootloaders.

### STRING_MANUFACTURER
Manufacturer Name

### STRING_PRODUCT
Product Name

### USB_VID_HIGH
High byte of USB VID

### USB_VID_LOW
Low byte of USB VID

### USB_PID_HIGH
High byte of USB PID

### USB_PID_LOW
Low byte of USB PID

### BOOT_USART_SERCOM_INSTANCE
This must be a single digit representing the SERCOM number

### BOOT_USART_PAD_SETTINGS
UART_RX_PAD3_TX_PAD2

### BOOT_USART_PADx
where x is 0, 1, 2, or 3. Consult CMSIS defines to determine what
goes here (ie:PINMUX_PA10C_SERCOM0_PAD2). Use PINMUX_UNUSED if not used.

### BOOT_DOUBLE_TAP_ENABLED
If BOOT_DOUBLE_TAP_ENABLED is defined the bootloader is started by
quickly tapping two times on the reset button.

### BOOT_LOAD_PIN
If BOOT_LOAD_PIN is defined (ie: PIN_PA27), the bootloader is started
if the selected pin is tied LOW during startup. An internal pullup
resistor will be enabled. There will be a 10ms delay before testin the
pin to allow time for debouncing capacitors to charge (button use).

### BOARD_LED_PORT, BOARD_LEDRX_PORT, BOARD_LEDTX_PORT
### BOARD_LED_PIN, BOARD_LEDRX_PIN, BOARD_LEDTX_PIN
If the PORT is defined, then the LED on the associated pin is enabled.

### BOARD_LED_POLARITY
This can be either LED_POLARITY_HIGH_ON or LED_POLARITY_LOW_ON.

### BOARD_LED_FADE_ENABLED
If BOARD_LED_FADE_ENABLED is defined, then the main LED produces
a PWM pulse or heartbeat, otherwise, it simply turns on if enabled.


## Generic Boards
The generic boards are all configured to minimize external hardware
requirements. Only one interface is enabled: SAM_BA_USBCDC_ONLY
(except C21, which uses SAM_BA_UART_ONLY). CLOCKCONFIG_CLOCK_SOURCE
is set to CLOCKCONFIG_INTERNAL_USB (CLOCKCONFIG_INTERNAL for the C21),
so no crystal is required. No LEDs are defined. BOOT_LOAD_PIN is not
defined, but BOOT_DOUBLE_TAP_ENABLED is, since it uses the reset pin.


## Boot Condition Test Sequence
First, the start location of the sketch is fetched and checked. If it
is empty (0xFFFFFFFF), then bootloader execution is resumed. Note that
when Arduino auto-reset (into bootloader) is initiated, the first flash
row is erased, so the bootloader will always run after reset until a
new sketch is transferred. Next, it checks for the double-tap reset
condition. Then, it checks the boot pin state (after a 10ms delay to
allow debounce capacitor charging). If no bootloader entry condition
is present, it jumps to the application and starts execution from there.


## Building a 4KB bootloader
Only one interface should be selected. ARDUINO_EXTENDED_CAPABILITIES
should not be defined. USB_VENDOR_STRINGS_ENABLED also should not be
defined in most cases, as well as BOOT_LOAD_PIN.


## Building

If not specified the makefile builds for **MT_D21E_rev_B**:

```
make
```

if you want to make a custom bootloader for a derivative board you must supply all the
necessary information in a `board_definitions_xxx.h` file, and add the corresponding case in
`board_definitions.h`. For example for the **Generic_x21J** board with a **SAMD21J18A** MCU,
we use `board_definitions_Generic_x21J.h` and it is build with the following command:

```
BOARD_ID=Generic_x21J MCU=SAMD21J18A make clean all
```

which will produce a binary named sam_ba_Generic_x21J_SAMD21J18A.bin


## Technical Details

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
