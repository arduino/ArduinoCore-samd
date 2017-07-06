# MattairTech SAM M0+ Core for Arduino

The MattairTech SAM M0+ Core is a fork from arduino/ArduinoCore-samd on GitHub, which will
be used to maintain Arduino support for MattairTech branded boards (see
https://www.mattairtech.com/) as well as "Generic" boards.

* Supports the SAMD21, SAMD11, SAML21, and SAMC21.
* Supports four clock sources (two crystals, internal oscillator, and USB calibrated).
* USB CDC Bootloader with optional SDCard support

*This core is intended to be installed using Boards Manager (see below). To update from a*
*previous version, click on MattairTech SAM M0+ Boards in Boards Manager, then click Update.*

**Differences from Arduino in Versioning**  The MattairTech version number does not
correspond to either the IDE version or to the upstream ArduinoCore-samd version. See the
CHANGELOG for details on which upstream commits have been merged in to the MattairTech core.


## What's New Beta (1.6.8-beta)
**See Beta Builds section for installation instructions.**

**1.6.8-beta-b2:**
* Added SD Card firmware loading support to the bootloader (4KB and 8KB)
* Removed SDU library, as the bootloader now supports SD cards directly
* Removed automatic page writes from bootloader (may have caused bricked board during development)
* Fixed bootloader compilation on Windows
* Added more Serial, SPI, and WIRE instances to MT-D21E (rev A and B)
* Added support for up to 6 SERCOM on the L21E (32-pin)
* Merged in changes from upstream SAMD CORE 1.6.16 (not released yet):
  * USB CDC: fixed issue of available() getting stuck when receiving ZLP's
* Merged in changes from upstream SAMD CORE 1.6.15 2017.04.27 (not relevant)
* Merged in changes from upstream SAMD CORE 1.6.14 2017.04.04:
  * Added lowpower function on USB subsystem
* Documentation updates

**1.6.8-beta-b1:**
* Fixed auto-reset not working on some versions of Windows
* Documentation updates

**1.6.8-beta-b0:**
* Added L21 and C21 support. Improved D11D and D11C support.
  * Use Tools->Microcontroller menu to select mcu.
* Both the core and bootloader have added support for:
  * external high-speed crystal (400KHz - 32MHz) using PLL
  * external 32.768KHz crystal using PLL
  * internal oscillator with USB calibration using DFLL
  * internal oscillator using DFLL in open-loop mode (or 48MHz RC oscillator with C21)
  * PLL_FRACTIONAL_ENABLED and PLL_FAST_STARTUP options
  * The clock source is selectable in the Tools->Clock Source menu
* New Tools->Serial Config menu for selecting different combinations of serial peripherals
* New Tools->Bootloader Size menu allows selection of bootloader size
* New Tools->USB Config menu simplifies USB configuration compared to previous core
* Updated variant.cpp table format for future CCL and GCLK use. See VARIANT_COMPLIANCE_CHANGELOG.
* Updated bootloader.
* Updated bossac upload tool (fixed support for SAML and SAMC)
* New CMSIS-Atmel package (this is different than from Arduino)
* Merged in all changes from upstream through SAMD CORE 1.6.14 (April 2017)


## What's New Release (1.6.6)
**This is out of date, use the beta for now.**

* 1.6.6-mt3:
  * Fixes compilation with CDC_UART and CDC_ONLY settings

* 1.6.6-mt2:
  * Changes the default Communication setting to CDC_UART (from CDC_HID_UART)

* 1.6.6-mt1:
  * New documentation section 'Special Notes'. Please read!
  * Updated ASCII pinouts to be more readable and less ambiguous.
  * Updated the Signed driver for Windows (extras directory) (see CHANGELOG for details)
  * Merged in changes from upstream (see CHANGELOG for details)
  * Fix warnings about deprecated recipe.ar.pattern
  * Merged in changes from upstream SAMD CORE 1.6.2 2015.11.03 (see CHANGELOG for details)


## Features Summary

Feature		| 21J (64 pin)				| 21G (48 pin)				| 21E (32 pin)				| D11 (24, 20, or 14 pin)
----------------|---------------------------------------|---------------------------------------|---------------------------------------|---------------------------------------
Board Variants	| New board coming June, Generic 21J	| Arduino Zero, Arduino M0, Generic 21G	| MT-D21E, Generic 21E			| MT-D11, Generic D11D14AM, Generic D11D14AS, Generic D11C14A
Processor	| 48 MHz 32-bit ARM Cortex M0+		| 48 MHz 32-bit ARM Cortex M0+		| 48 MHz 32-bit ARM Cortex M0+		| 48 MHz 32-bit ARM Cortex M0+
Flash Memory	| Up to 256KB (L21/C21 have RWW)	| Up to 256KB (L21/C21 have RWW)	| Up to 256KB (L21/C21 have RWW)	| 16 KB (4KB used by bootloader)
SRAM		| Up to 32KB (plus <=8KB LPSRAM on L21)	| Up to 32KB (plus <=8KB LPSRAM on L21)	| Up to 32KB (plus <=8KB LPSRAM on L21)	| 4 KB
Digital Pins	| 52 (51 for L21)			| 38 (37 for L21)			| 26 (25 for L21)			| 24-pin: 21, 20-pin: 17, 14-pin: 11
Analog Inputs	| 20 channels, 12-bit			| 14 channels, 12-bit			| 10 channels, 12-bit			| 24-pin: 10, 20-pin: 8, 14-pin: 5 (12-bit)
Analog Outputs	| One 10-bit (two 12-bit on L21)	| One 10-bit (two 12-bit on L21)	| One 10-bit (two 12-bit on L21)	| One 10-bit
PWM Outputs	| 18					| 14					| 14					| 8 (6 for 14-pin)
Interrupts	| 16					| 16					| 16					| 8 (7 for 14-pin)
USB		| Full Speed Device and Host (not C21)	| Full Speed Device and Host (not C21)	| Full Speed Device and Host (not C21)	| Full Speed Device
SERCOM*		| 6					| 6					| 4 (6 for L21)				| 3 (2 for 14-pin)
UART (Serial)*	| Up to 6				| Up to 6				| Up to 4 (up to 6 for L21)		| Up to 2
SPI*		| Up to 3				| Up to 2				| Up to 2				| Up to 1
I2C (WIRE)*	| Up to 3				| Up to 2				| Up to 2				| Up to 1
I2S		| Present on the D21 only		| Present on the D21 only		| Present on the D21 only		| Not present
Voltage		| 1.62V-3.63V (2.7V-5.5V for the C21)	| 1.62V-3.63V (2.7V-5.5V for the C21)	| 1.62V-3.63V (2.7V-5.5V for the C21)	| 1.62V-3.63V
I/O Pin	Current	| D21: 7mA, L21: 5mA, C21: 6mA@5V	| D21: 7mA, L21: 5mA, C21: 6mA@5V	| D21: 7mA, L21: 5mA, C21: 6mA@5V	| 7 mA

*Note that the maximum number of UART/SPI/I2C is the number of SERCOM. The number listed above for UART/SPI/I2C indicated how many are configurable through the Arduino IDE menu.*



## Board Variants

Pin configuration and peripheral assignment information is now in the README.md for each board variant.
README.md also now includes technical information on the new PinDescription table format.

* [MattairTech MT-D21E Rev B (SAMx21Exxx)](https://github.com/mattairtech/ArduinoCore-samd/tree/master/variants/MT_D21E_revB/README.md)

* [MattairTech MT-D21E Rev A (SAMD21ExxA)](https://github.com/mattairtech/ArduinoCore-samd/tree/master/variants/MT_D21E/README.md)

* [MattairTech MT-D11 (SAMD11D14AM)](https://github.com/mattairtech/ArduinoCore-samd/tree/master/variants/MT_D11/README.md)

* [MattairTech Generic D11C14A](https://github.com/mattairtech/ArduinoCore-samd/tree/master/variants/Generic_D11C14A/README.md)

* MattairTech x21J based board (coming July)

* MattairTech Generic D11D14AS (coming soon)

* MattairTech Generic D11D14AM (coming soon)

* MattairTech Generic x21E (coming soon)

* MattairTech Generic x21G (coming soon)

* MattairTech Generic x21J (coming soon)

* [Arduino Zero (arduino.cc)](https://github.com/mattairtech/ArduinoCore-samd/tree/master/variants/arduino_zero/README.md)

* [Arduino M0 (arduino.org)](https://github.com/mattairtech/ArduinoCore-samd/tree/master/variants/arduino_mzero/README.md)



## Tools Menu Additions

Depending on the board variant, different menu options will appear in the Tools menu. 


### Microcontroller Menu

This menu will appear with boards that have multiple microcontroller options.


### Clock Source Menu

There are up to four clock source choices, depending on board variant and microcontroller. They are:

* 32KHZ_CRYSTAL (default)
* HIGH_SPEED_CRYSTAL
* INTERNAL_OSCILLATOR
* INTERNAL_USB_CALIBRATED_OSCILLATOR

See Clock Source section for more information.


### Bootloader Size Menu

With the D21, L21, and C21, the bootloader size can be configured as:

* 8KB_BOOTLOADER (default)
* 16KB_BOOTLOADER
* NO_BOOTLOADER

With the D11, the bootloader size can be configured as:

* 4KB_BOOTLOADER (default)
* NO_BOOTLOADER

Choose NO_BOOTLOADER if not using a bootloader (an external programmer will be used for sketch upload).


### Serial Config Menu

This menu is used to select different combinations of serial peripherals. It adds additional UART, SPI,
and WIRE instances. This is also useful for the D11, which has a reduced pin count and number of SERCOMs.
It can also be used to reduce FLASH and SRAM usage by selecting fewer UART peripherals, which are
instantiated in the core, rather than only when including a library (like SPI and WIRE). Note that with
options where there is more than one SPI or WIRE, the additional instances will consume a small amount
of RAM, but neither the peripheral nor the pins are configured until begin() method is called (thus, the
pins can be used for other purposes).

Use the ASCII art rendering at the top of the README.md file of the board variant used in order to
determine the mapping of instances to pins. When USB CDC is enabled, Serial refers to SerialUSB,
otherwise it refers to Serial1 (TX1/RX1).


### USB Config Menu

This menu will appear with all microcontrollers except the C21, which does not have USB. The options are:

* CDC_ONLY (default)
* CDC_HID
* WITH_CDC
* HID_ONLY
* WITHOUT_CDC
* USB_DISABLED

Choose an option that best matches your code and library usage. Each option results in a different USB PID.
Choose an option with CDC if you want auto-reset to function, or the serial monitor over USB. If CDC is
not enabled, Serial will refer to Serial1 instead of SerialUSB. These options can be used to optimize FLASH
and SRAM usage by allowing CDC to be disabled (or USB completely disabled).


## Clock Source

There are up to four clock source choices, depending on board features and microcontroller. Since currently
the cpu must run at 48MHz, the PLL or DFLL must be used (the SAMC can use OSC48M).

### 32KHZ_CRYSTAL (default)
* Uses both XOSC32K and FDPLL96M
* High long-term accuracy, slow startup, medium current (PLL)

### HIGH_SPEED_CRYSTAL
* Uses both XOSC and FDPLL96M
* High accuracy, medium startup, high current (XOSC and PLL)

### INTERNAL_OSCILLATOR
* Uses DFLL48M in open-loop mode (SAMC uses OSC48M)
* Low accuracy, fast startup, medium-low current (low current with SAMC)

### INTERNAL_USB_CALIBRATED_OSCILLATOR (not available with SAMC)
* Uses DFLL48M in closed-loop mode
* High accuracy, medium-fast startup, medium current

### SAMD

Source          | Frequency Range                       | Supply Current (max.) | Startup Time typ. (max.)      | Notes, jitter, accuracy, other differences
----------------|---------------------------------------|-----------------------|-------------------------------|--------------------------------------------------------
XOSC            | 0.4MHz-32MHz crystal                  | 307uA (552uA) AGC on  | 5K-14K cycles (10K-48K)       | up to 32MHz digital clock input, Supply current based on 16MHz crystal
XOSC32K         | 32.768KHz typical crystal             | 1.22uA (2.19uA)       | 28K cycles (30K)              | 32.768KHz typical digital clock input
OSC32K          | 32.27-33.26KHz (28.50-34.74KHz)       | 0.67uA (1.32uA)       | 1 cycle (2 cycles)            | 
OSCULP32K       | 31.29-34.57KHz (25.55-38.01KHz)       | 0.125uA max.          | 10 cycles                     | 
OSC8M           | 7.94-8.06MHz (7.80-8.16MHz)           | 64uA                  | 2.1us (3us)                   | 
FDPLL96M        | 32KHz-2MHz in, 48MHz-96MHz out        | 500uA (700uA)         | Lock: 25us (50us) @ 2MHz in   | 1.5% (2%) period jitter (32KHz in, 48MHz out), Lock: 1.3ms (2ms) @ 32KHz in
DFLL48M open    | 47MHz-49MHz out                       | 403uA (453uA)         | 8us (9us)                     | 
DFLL48M closed  | 0.73-33KHz in, 47.96-47.98MHz out     | 425uA (482uA)         | Lock: 200us (500us)           | 0.42ns max. jitter

### SAML

Source          | Frequency Range                       | Supply Current (max.) | Startup Time typ. (max.)      | Notes, jitter, accuracy, other differences
----------------|---------------------------------------|-----------------------|-------------------------------|--------------------------------------------------------
XOSC            | 0.4MHz-32MHz crystal                  | 293uA (393uA) AGC on  | 5K-14K cycles (10K-48K)       | up to 24MHz digital clock input, Supply current based on 16MHz crystal
XOSC32K         | 32.768KHz typical crystal             | 0.311uA (2.19uA)      | 25K cycles (82K)              | 32.768KHz typical digital clock input (1MHz max.)
OSC32K          | 32.57-33.05KHz (28.58-34.72KHz)       | 0.54uA (1.10uA)       | 1 cycle (2 cycles)            | 
OSCULP32K       | 31.77-34.03KHz (26.29-38.39KHz)       | Not Specified.        | Not Specified                 | 
OSC16M          | 15.75-16.24MHz                        | 141uA (169uA)         | 1.4us (3.1us)                 | Wake up time: 0.12us (0.25us)
FDPLL96M        | 32KHz-2MHz in, 48MHz-96MHz out        | 454uA (548uA)         | Lock: 25us (35us) @ 2MHz in   | 1.9% (4%) period jitter (32KHz in, 48MHz out), Lock: 1ms (2ms) @ 32KHz in
DFLL48M open    | 46.6MHz-49MHz out                     | 286uA                 | 8.3us (9.1us)                 | 
DFLL48M closed  | 0.73-33KHz in, 47.96-47.98MHz out     | 362uA                 | Lock: 200us (700us)           | 0.51ns max. jitter

### SAMC

Source          | Frequency Range                       | Supply Current (max.) | Startup Time typ. (max.)      | Notes, jitter, accuracy, other differences
----------------|---------------------------------------|-----------------------|-------------------------------|--------------------------------------------------------
XOSC            | 0.4MHz-32MHz crystal                  | 429uA (699uA) AGC on  | 6K-12K cycles (20K-48K)       | up to 48MHz digital clock input, Supply current based on 16MHz crystal
XOSC32K         | 32.768KHz typical crystal             | 1.53uA (2.84uA)       | 16K cycles (24K)              | 32.768KHz typical digital clock input
OSC32K          | 32.11-33.43KHz (25.55-37.36KHz)       | 0.864uA (1.08uA)      | 1 cycle (2 cycles)            | 
OSCULP32K       | 30.96-34.57KHz (22.93-38.99KHz)       | Not Specified.        | Not Specified                 | 
OSC48M          | 47.04-48.96MHz                        | 87uA (174uA)          | 22.5us (25.5us)               | 
FDPLL96M        | 32KHz-2MHz in, 48MHz-96MHz out        | 536uA (612uA)         | Not Yet Specified             | 


### External 32.768KHz Crystal

The PLL will be used with the 32.768KHz crystal. PLL_FRACTIONAL_ENABLED can be defined,
which will result in a more accurate 48MHz output frequency at the expense of increased
jitter.

### External High-Speed Crystal

HS_CRYSTAL_FREQUENCY_HERTZ must be defined with the external crystal frequency in Hertz.
The crystal frequency must be between 400000Hz and 32000000Hz. The PLL will be used.
PLL_FRACTIONAL_ENABLED can be defined, which will result in a more accurate 48MHz output
frequency at the expense of increased jitter. If PLL_FAST_STARTUP is defined, the crystal
will be divided down to 1MHz - 2MHz, rather than 32KHz - 64KHz, before being multiplied
by the PLL. This will result in a faster lock time for the PLL, however, it will also
result in a less accurate PLL output frequency if the crystal is not divisible (without
remainder) by 1MHz. In this case, define PLL_FRACTIONAL_ENABLED as well. By default,
PLL_FAST_STARTUP is disabled. PLL_FAST_STARTUP is also useful for USB host mode
applications. See datasheet USB electrical characteristics. The crystal frequency must
be at least 1000000Hz when PLL_FAST_STARTUP is defined.

### Internal Oscillator

The DFLL will be used in open-loop mode, except with the C21 which lacks a DFLL, so the
internal 48MHz RC oscillator is used instead. NVM_SW_CALIB_DFLL48M_FINE_VAL is the fine
calibration value for DFLL open-loop mode. The coarse calibration value is loaded from
NVM OTP (factory calibration values).

### Internal Oscillator with USB Calibration

This is available for the D21, D11, or L21. It will also use the DFLL in open-loop mode,
except when connected to a USB port with data lines (and not suspended), then it
will calibrate against the USB SOF signal. NVM_SW_CALIB_DFLL48M_FINE_VAL is the fine
calibration value for DFLL open-loop mode. The coarse calibration value is loaded from
NVM OTP (factory calibration values).

### Clock Generators Currently Used

0. MAIN (mcu)
1. XOSC (high speed crystal)
2. OSCULP32K (initialized at reset for WDT on D21 and D11)
3. OSC_HS (the reset default internal RC oscillator is put here at 8MHz, except with C21)



## Analog Reference

### D21
* AR_DEFAULT uses 1/2X gain on each input.
* The external reference should be between 1.0V and VDDANA-0.6V.

### L21
* AR_DEFAULT = AR_INTERNAL_INTVCC2
* Both AR_INTREF and AR_INTERNAL1V0 has the same effect as AR_INTREF_1V0.
* The external reference should be between 1v and VDDANA-0.6v=2.7v.

### C21
* AR_DEFAULT = AR_INTERNAL_INTVCC2
* Both AR_INTREF and AR_INTERNAL1V0 has the same effect as AR_INTREF_1V024.
* The external reference should be between 1v and VDDANA-0.6v=2.7v.

**Warning : The maximum reference voltage is Vcc (up to 3.6 volts for the SAMD/SAML, 5V for the SAMC)**

### Reference Selection Table

D21 / D11               | Volts      | L21                   | Volts     | C21                   | Volts
------------------------|------------|-----------------------|-----------|-----------------------|--------
AR_DEFAULT              | 1/2 VCC    | AR_DEFAULT            | VCC       | AR_DEFAULT            | VCC
AR_INTERNAL1V0          | 1.00V      | AR_INTREF             | 1.00V     | AR_INTREF             | 1.024V
AR_INTERNAL_INTVCC0     | 1/1.48 VCC | AR_INTREF_1V0         | 1.00V     | AR_INTREF_1V024       | 1.024V
AR_INTERNAL_INTVCC1     | 1/2 VCC    | AR_INTREF_1V1         | 1.10V     | AR_INTREF_2V048       | 2.048V
AR_EXTERNAL_REFA        | REFA       | AR_INTREF_1V2         | 1.20V     | AR_INTREF_4V096       | 4.096V
AR_EXTERNAL_REFB        | REFB       | AR_INTREF_1V25        | 1.25V     | AR_INTERNAL1V0        | 1.024V
---                     |            | AR_INTREF_2V0         | 2.00V     | AR_INTERNAL_INTVCC0   | 1/1.6 VCC
---                     |            | AR_INTREF_2V2         | 2.20V     | AR_INTERNAL_INTVCC1   | 1/2 VCC
---                     |            | AR_INTREF_2V4         | 2.40V     | AR_INTERNAL_INTVCC2   | VCC
---                     |            | AR_INTREF_2V5         | 2.50V     | AR_EXTERNAL_REFA      | REFA
---                     |            | AR_INTERNAL1V0        | 1.00V     | AR_EXTERNAL_DAC       | DAC
---                     |            | AR_INTERNAL_INTVCC0   | 1/1.6 VCC |                       |
---                     |            | AR_INTERNAL_INTVCC1   | 1/2 VCC   |                       |
---                     |            | AR_INTERNAL_INTVCC2   | VCC       |                       |
---                     |            | AR_EXTERNAL_REFA      | REFA      |                       |
---                     |            | AR_EXTERNAL_REFB      | REFB      |                       |

### Common Settings

AR_INTERNAL = AR_INTERNAL_INTVCC0
AR_INTERNAL2V23 = AR_INTERNAL_INTVCC0
AR_INTERNAL1V65 = AR_INTERNAL_INTVCC1
AR_EXTERNAL = AR_EXTERNAL_REFA

*When using AR_INTERNAL2V23 or AR_INTERNAL1V65, these voltages are correct only when Vcc = 3.3V)*



## Chip Specific Notes

### SAMD21

* When USB is disabled, pullups will be enabled on PA24 and PA24 to avoid excessive current consumption (<1mA) due to floating pins.
  Note that it is not necessary to enable pull resistors on any other pins that are floating.
  Errata: Disable pull resistors on PA24 and PA25 manually before switching to a peripheral.


### SAML21

* There are two DACs, DAC0 and DAC1. Both are supported. Because changing the configuration of one DAC requires disabling both,
  there will be about a 40us period when the second DAC is disabled. Most of this time is due to an errata that requires a delay of
  at least 30us when turning off the DAC while refresh is on. The L21 DACs have a refresh setting which must be enabled in this core.
* The analog reference has additional options on the L21 and C21. See Analog Reference section.
* On the L21, SERCOM5 is in a low power domain. The Fm+ and HS modes of I2C (wire) are not supported.
* The SAML and SAMC have double-buffered TCs, which are supported in the core.
* The CHANGE and RISING interrupt modes on pin A31 do not seem to work properly on the L21.
* The L21 has two performance levels that affect power consumption. During powerup, the L21 starts at the lowest performance level (PL0).
  The startup code changes to the highest performance level (PL2) in order to support 48MHz and USB (among other things).
* Two Flash Wait States are inserted for the L21 and C21 (the D21/D11 use one wait state).
* pinPeripheral now handles disabling the DAC (if active). Note that on the L21, the DAC output would
  interfere with other peripherals if left enabled, even if the anaolog peripheral is not selected.


### SAMC21

* There are two SAR ADCs. Both are supported. The PinDescription table determines the peripheral instance and pin mapping.
* The analog reference has additional options on the L21 and C21. See Analog Reference section.
* The SAML and SAMC have double-buffered TCs, which are supported in the core.
* Two Flash Wait States are inserted for the L21 and C21 (the D21/D11 use one wait state).
* The C21 requires internal pull resistors to be activated on floating pins to minimize power consumption (not needed on D21/D11 or L21).
* The C21 uses the minimum sampling time so that rail-to-rail and offset compensation works. Offset compensation adds 3 ADC clock cycles,
  so the total is 4 clock cycles. The D21, D11, and L21 use the maximum sampling time.


### SAMD11

* The D11D has three SERCOM. The D11C has two sercom (no sercom2).
* TONE: TC5 does not exist on the D11. Using TC2 instead (TC1 on the D11C14 as TC2 is not routed to pins). It will conflict with the 2 associated TC analogWrite() pins.
* When USB is disabled, pullups will be enabled on PA24 and PA24 to avoid excessive current consumption (<1mA) due to floating pins.
  Note that it is not necessary to enable pull resistors on any other pins that are floating.
  Errata: Disable pull resistors on PA24 and PA25 manually before switching to a peripheral.

#### Reducing SRAM/FLASH Usage on the D11

TODO


## Differences Between MattairTech and Arduino Cores (TODO)

* Table summarizing which core files are modified and by how much
* Communications interfaces are mostly unchanged, including USB
* Changes due to adding/changing features vs porting to new chip
* All pins (digital and analog) setup in STARTUP mode (enable INEN and set default pull direction to pullup (pullup will not be enabled))
* INEN enabled for both input and output (but not analog)
* pinPeripheral now handles disabling the DAC (if active). Note that on the L21, the DAC output would
  interfere with other peripherals if left enabled, even if the anaolog peripheral is not selected.
* Pull resistors enabled only if pin attributes allow and only if pin is not configured as output.
* Pull direction (pullup or pulldown) is now set with pinMode only (defaults to pullup if pinMode never called).
* At least on the L21, pin A31 must be set as an input. It is possible that debugger probe detection is being falsely
  detected (even with a pullup on A31 (SWCLK)), which would change the peripheral mux of A31 to COM.
  This might not normally be a problem, but one strange effect is that Serial2 loses characters if pin A31 is not set as INPUT.
  So, the startup code calls pinMode(31, INPUT).


## Serial Monitor

To print to the Serial Monitor over USB, use 'Serial'. Serial refers to SerialUSB (Serial1 and Serial2 are UARTs).
Unlike most Arduino boards (ie. Uno), SAM M0+ boards do not automatically reset when the serial monitor is opened.
To see what your sketch outputs to the serial monitor from the beginning, the sketch must wait for the SerialUSB
port to open first. Add the following to setup():

```
while (!Serial) ;
```

Remember that if the sketch needs to run without SerialUSB connected, another approach must be used.
You can also reset the board manually with the Reset button if you wish to restart your sketch. However, pressing
the Reset button will reset the SAM M0+ chip, which in turn will reset USB communication. This interruption means
that if the serial monitor is open, it will be necessary to close and re-open it to restart communication.

When USB CDC is not enabled, Serial will instead refer to Serial1, which is the first UART.


## Code Size and RAM Usage (1.6.5-mt2)

TODO: Update this. Maybe just for D11 and move to D11 Chip Specific Notes.

Sketch and Configuration    | MT-D21E (Flash + RAM) | MT-D11 (Flash + RAM)
----------------------------|-----------------------|-----------------------
Blink (CDC + HID + UART)    |     7564 + 1524       |     7452 + 1424
Blink (CDC + UART)          |     6588 + 1496       |     6484 + 1396
Blink (CDC Only)            |     5248 + 1304       |     5192 + 1300
Blink (UART Only)           |     3828 + 336        |     3716 + 236
Blink (No USB or UART)      |     2472 + 144        |     2416 + 140
Datalogger (No USB or UART) |     10340 + 948       |     10260 + 944

* 180 bytes of flash can be saved on the MT-D11 by using PIN_MAP_COMPACT (see 'New PinDescription Table' below).
* Datalogger compiled without USB or UART support, but with SPI and SD (with FAT filesystem) support. Serial output was disabled.
* Note that USB CDC is required for auto-reset into the bootloader to work (otherwise, manually press reset twice in quick succession).
* USB uses primarily 3 buffers totaling 1024 bytes. The UART uses a 96 byte buffer. The banzai() function (used for auto-reset) resides in RAM and uses 72 bytes.
* Any combination of CDC, HID, or UART can be used (or no combination), by using the Tools->Communication menu.


### Detailed Memory Usage Output After Compilation

The flash used message at the end of compilation is not correct. The number shown
represents the .text segment only. However, Flash usage = .text + .data segments
(RAM usage = .data + .bss segments). In this release, two programs are run at the
end of compilation to provide more detailed memory usage. To enable this output, go
to File->Preferences and beside "Show verbose output during:", check "compilation".

Just above the normal flash usage message, is the output from the size utility.
However, this output is also incorrect, as it shows .text+.data in the .text field,
but 0 in the .data field. However, the .text field does show the total flash used.
The .data field can be determined by subtracting the value from the normal flash
usage message (.text) from the value in the .text field (.text+.data). The .bss
field is correct.

Above the size utility output is the output from the nm utility. The values on the
left are in bytes. The letters stand for: T(t)=.text, D(d)=.data, B(b)=.bss, and
everything else (ie: W) resides in flash (in most cases).


## Installation

### Driver Installation

#### Windows

Prior to core version 1.6.6-mt1, sketches compiled with both CDC and HID USB code by default, thus requiring a CDC
driver for the bootloader and a CDC-HID driver for sketches. Now that PluggableUSB is supported, sketches compile
with only CDC code by default. Thus, only one driver is needed. Since HID and MIDI are currently supported (and
MSD potentially in the future), driver installation will be required for each different combination of USB devices.
There are currently four USB composite device combinations that include CDC as well as a CDC only device. Each
supported combination has a unique USB VID:PID pair, and these are listed in the .inf file. Once the first device
is installed (the CDC only device), future installations *might* be automatic, otherwise, you may direct the
installer to the same .inf file. The drivers are signed and support both 32 and 64 bit versions of Windows XP(SP3),
Vista, 7, 8, and 10. Note that the Windows 10 generic CDC drivers work as well.


1. If you do not already have the SAM-BA bootloader installed, see below.
2. Download https://www.mattairtech.com/software/MattairTech_CDC_Driver_Signed.zip and unzip into any folder.
   Note that the Windows 10 generic CDC drivers work as well.
3. Plug in the board. The LED should fade when the bootloader is running (or blink if the test sketch is running).
4. Windows will detect the board. Point the installer to the folder from above to install the bootloader driver.
5. If you don't intend on using Arduino, you can skip the rest of this list. See Using Bossac Standalone below.
6. If you do not already have the test firmware installed (comes preinstalled), see Using Bossac Standalone below.
7. Press the reset button to run the test firmware (if needed). The LED will blink.
8. Windows will detect the board. Point the installer to the above folder to install the sketch driver (if needed).
9. Continue with SAM M0+ Core Installation below.

#### Linux

0. No driver installation is needed.
1. On some distros, you may need to add your user to the same group as the port (ie: dialout) or set udev rules:
   * See the file https://github.com/mattairtech/ArduinoCore-samd/tree/master/drivers/99-mattairtech-USB-CDC.rules.
2. You MAY have to install and use Arduino as the root user in order to get reliable access to the serial port.
   * This is true even when group permissions are set correctly, and it may fail after previously working.
   * You can also create/modify a udev rule to set permissions on the port so *everyone* can read / write.
3. If you are running modemmanager (ie: Ubuntu), disable it, or use the udev rules file above.
4. Continue with SAM M0+ Core Installation below.

#### OS X

*OS X support was added in version 1.6.7-beta-b0.*

0. No driver installation is needed.
5. Plug in the board. You may get a dialog box asking if you wish to open the “Network Preferences”:
   * Click the "Network Preferences..." button, then click "Apply".
   * The board will show up as “Not Configured”, but it will work fine.
5. Continue with SAM M0+ Core Installation below.


### SAM M0+ Core Installation

**See Beta Builds section below to install the beta, as it uses a different json file**

* To update from a previous version, click on MattairTech SAM M0+ Boards in Boards Manager, then click Update.

1. The MattairTech SAM M0+ Core requires Arduino 1.6.7 or above (including 1.8.x).
2. In the Arduino IDE, click File->Preferences.
3. Click the button next to Additional Boards Manager URLs.
4. Add https://www.mattairtech.com/software/arduino/package_MattairTech_index.json.
5. Save preferences, then open the Boards Manager.
6. Install the Arduino SAMD Boards package. Use version 1.6.2 or higher.
7. Install the MattairTech SAM M0+ Boards package.
8. Close Boards Manager, then click Tools->Board->MattairTech MT-D21E (or MT-D11).
9. Select the MCU with the now visible Tools->Microcontroller menu (if present).
10. If you do not already have the bootloader or blink sketch installed, see SAM-BA USB CDC Bootloader below.
11. Plug in the board. The blink sketch should be running.
12. Click Tools->Port and choose the COM port. Note that the board indicated may not match the chosen board*
13. You can now upload your own sketch.

*Currently, with MattairTech boards, USB PIDs are shared across boards (but they are different based on Tools->USB Config).*
*This will result in Tools->Port showing "MattairTech MT-D21E (rev B)" for all MattairTech boards.*

### Uploading the First Sketch

1. In the Arduino IDE (1.6.7 or above), open File->Examples->01.Basics->Blink.
2. Change the three instances of '13' to 'LED_BUILTIN'.
3. Be sure the correct options are selected in the Tools menu (see Core Installation above).
4. With the board plugged in, select the correct port from Tools->Port.
5. Click the Upload button. After compiling, the sketch should be transferred to the board.
6. Once the bootloader exits, the blink sketch should be running.


## Beta Builds

Periodically, a beta is released for testing.

The beta builds are available through Boards Manager. If you want to install them:
  1. Open the **Preferences** of the Arduino IDE.
  2. Add this URL `https://www.mattairtech.com/software/arduino/beta/package_MattairTech_index.json` in the **Additional Boards Manager URLs** field, and click OK.
  3. Open the **Boards Manager** (menu Tools->Board->Board Manager...)
  4. Install **MattairTech SAM M0+ Boards - Beta build**
  5. Select one of the boards under **MattairTech SAM M0+ Beta Build XX** in Tools->Board menu
  6. Compile/Upload as usual

The Arduino IDE will notify the user if an update to the beta is available, which can then be installed automatically.
Alternatively, if a particular beta is needed, replace the url in step 2 with:
  `https://www.mattairtech.com/software/arduino/beta/package_MattairTech_sam_m0p-${VERSION}-beta-b${BUILD_NUMBER}_index.json`
where ${VERSION} and ${BUILD_NUMBER} match the beta name as shown in the CHANGELOG (ie: package_MattairTech_sam_m0p-1.6.7-beta-b0_index.json).
In this case, the IDE will not notify the user of updates.


## SAM-BA USB CDC Bootloader (Arduino compatible)

This bootloader is based on the Arduino Zero bootloader which is a part of the Arduino SAMD core. It
provides a USB-CDC and/or TTL serial communications interface to a host running the bossac command
line firmware programming utility (or the Arduino IDE) running on Windows, Linux, or OS X. Optionally,
SD Card firmware loading is supported, using SDSC or SDHC cards with a FAT16 or FAT32 filesystem.
This version adds support for the D11, L21, and C21 microcontrollers. It also adds support for four
different clock sources (two external crystals and two internal oscillator options). There are
additional board definitions added, and binaries for most board/chip combinations are pre-built.

**See [bootloaders/zero/README.md](https://github.com/mattairtech/ArduinoCore-samd/tree/master/bootloaders/zero/README.md) for more technical information on the bootloader.**


### Features

* SAM-BA USB CDC and UART interfaces with optional terminal mode
* SD Card interface (both USB CDC and SD Card support fits in 8KB)
* Four different clock sources (two external crystals and two internal oscillator options)
* Arduino IDE auto-reset and double-tap reset button support
* Arduino extended commands for faster firmware loading
* Supports the D21, L21, C21, and D11 SAM M0+ chips
* Bossac command line utility for Windows, Linux, and OS X

The bootloader can be started by:

   * Tapping reset twice in quick succession (BOOT_DOUBLE_TAP).
   * Holding down button A (BOOT_LOAD_PIN) while powering up.
   * Clicking 'Upload Sketch' in the Arduino IDE, which will automatically start the bootloader (when CDC is enabled).
   * If the application (sketch) area is blank, the bootloader will run.

Otherwise, it jumps to application and starts execution from there. The LED will PWM fade during bootloader execution.


### Bossac

Bossac is a command line utility for uploading firmware to SAM-BA bootloaders. It runs on Windows. Linux, and OS X.
It is used by Arduino to upload firmware to SAM and SAM M0+ boards. The version described here adds to the
Arduino version (https://github.com/shumatech/BOSSA, Arduino branch), which in turn is a fork from the original
Bossa (http://www.shumatech.com/web/products/bossa). It adds support for more SAM M0+ chips (D21, L21, C21, and D11).
Note that only the Arduino or Mattairtech versions of bossac are currently supported for SAM M0+ chips.
Neither the stock bossac (or Bossa) nor the Atmel SAM-BA upload tool will work.


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

A running sketch *may* interfere with the bootloader installation process. Be sure you are running the existing bootloader or using a blank chip.

#### Bootloader Installation Using Another Tool (ie: Atmel Studio, openocd)

**See [bootloaders/zero/README.md](https://github.com/mattairtech/ArduinoCore-samd/tree/master/bootloaders/zero/README.md) for information.**


### Bootloader Binaries

The bootloaders/zero/binaries directory contains the SAM-BA m0+
bootloaders built by the build_all_bootloaders.sh script from
the 'MattairTech SAM M0+ Boards' Arduino core, which is available
at https://github.com/mattairtech/ArduinoCore-samd. Each board
and chip combination has two bootloaders available:

* SAM-BA interface only
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


### Using Bossac Standalone

**See [bootloaders/zero/README.md](https://github.com/mattairtech/ArduinoCore-samd/tree/master/bootloaders/zero/README.md) for information on using Bossac standalone.**


## New PinDescription Table

Technical information on the new PinDescription table format is now in the README.md
that accompanies each board variant. See board variants above.

### Note that a new column (GCLKCCL) was added for 1.6.8-beta-b0.
MATTAIRTECH_ARDUINO_SAMD_VARIANT_COMPLIANCE in variant.h is used to track versions.
If using board variant files with the old format, the new core will still read the
table the old way, losing any new features introduced by the new column. Additionally,
new definitions have been added for L21 and C21 support.

### Each pin can have multiple functions.
The PinDescription table describes how each of the pins can be used by the Arduino
core. Each pin can have multiple functions (ie: ADC input, digital output, PWM,
communications, etc.), and the PinDescription table configures which functions can
be used for each pin. This table is mainly accessed by the pinPeripheral function in
wiring_private.c, which is used to attach a pin to a particular peripheral function.
The communications drivers (ie: SPI, I2C, and UART), analogRead(), analogWrite(),
analogReference(), attachInterrupt(), and pinMode() all call pinPeripheral() to
verify that the pin can perform the function requested, and to configure the pin for
that function. Most of the contents of pinMode() are now in pinPeripheral().

### Pin Mapping
There are different ways that pins can be mapped. Typically, there is no relation
between the arduino pin number used, and the actual port pin designator. Thus, the 
pcb must be printed with the arduino numbering, otherwise, if the port pin is printed,
a cross reference table is needed to find the arduino pin number. However, this results
in the least amount of space used by the table. Another method, used by default by the
MT-D21E and MT-D11, maps Arduino pin numbers to the actual port pin number (ie: Arduino
pin 28 = Port A28). This works well when there is only one port (or if the PORTB pins
are used for onboard functions and not broken out). PIO_NOT_A_PIN entries must be added
for pins that are used for other purposes or for pins that do not exist (especially the
D11), so some FLASH space may be wasted. For an example of both types, see variant.cpp
from the MT-D11 variant.

**See Board Variants above for more technical information on the PinDescription table.**

**See [WVariant.h](https://github.com/mattairtech/ArduinoCore-samd/tree/master/cores/arduino/WVariant.h) for the definitions used in the table.**



## MattairTech Libraries

### Available Now

*Use Libraries Manager to install*

* SRAM_23LC - Library for Microchip Technology Inc. 23LC (23LCV, 23A, 23K) SPI SRAM chips
  * Byte and block transfers

* EEPROM_CAT25 - Library for On Semiconductor CAT25 SPI EEPROM chips
  * Byte, block, and page transfers

### Under Development

* ZeroTimers - 8/16/24/32 bit timer library with API based on TimerOne
  * PWM
  * interrupt
  * Input capture

* POWER_PAC1921 - Library for Microchip Technologies high-side power/current/voltage monitor with I2C and analog out
* SENSOR_LPS22HB - Library for ST MEMS nano pressure sensor / temperature sensor with I2C
* SENSOR_LSM6DS3H - Library for ST iNemo inertial module: 3D accelerometer / 3D gyroscope with I2C and interrupt
* FLASH_AT25 - Library for Adesto Technologies AT25 SPI serial FLASH devices

### Possible Future

* Several I2C (Wire) sensor devices
* TFT LCD (CFAF128128B-0145T)
* IR decoder
* I2S DAC/AMP and I2S MEMS microphone
* Battery management IC
* XBee/Xbee Pro devices?


## Core Future Additions/Changes

### Under Development

* PlatformIO support
* Fix programming port for Arduino Zero and M0 board variants
* Reduce SRAM usage by USB endpoint buffers by only allocating endpoints actually used (D11 especially)

### Possible Future

* Features for lower power consumption (library?) Summer 2017?
* Reliability and security enhancements
* USB Host mode CDC ACM (partially complete; BSD-like license?)
* SD card library? Port of FatFS and/or Petit FatFS?
* Optional use of single on-board LED as USB activity LED
* MSC (Mass Storage) USB Device Class
* Polyphonic tone
* Wired-AND, Wired-OR for port pins
* High-speed port pin access (IOBUS)

### Feature Requests

Please use the GitHub Issue Tracker if you would like to request a feature.



## ChangeLog

The Changelog has moved to a separate file named CHANGELOG. The most recent changes are still in the 'What's New' section above.


## Troubleshooting

* **Tools->Port shows wrong board**
  * Currently, with MattairTech boards, USB PIDs are shared across boards (but they are different based on Tools->USB Config).
    This will result in Tools->Port showing "MattairTech MT-D21E (rev B)" for all MattairTech boards.

* **Tools->USB Config menu**
  * Currently, the Tools->USB Config menu (was Tools->Communications) must be used to select the communications configuration.
    This configuration must match the included libraries. For example, when including the HID and Keyboard libraries, you must
    select an option that includes HID (all options except CDC_ONLY or USB_DISABLED). This menu is currently needed to select
    the USB PID that matches the USB device configuration (needed for some versions of Windows). It is also used to control
    if CDC support is compiled (CDC is always enabled in the stock Arduino core). Auto reset requires CDC to be enabled.

    * Be sure that the Tools->Communications menu matches the sketch and libraries you are compiling.
    * Different combinations of USB devices will result in different COM port assingments in Windows.

* **Incude platform specific libraries**
  * You may need to manually include platform specific libraries such as SPI.h, Wire.h, and HID.h.

* **Errors when compiling, uploading, or burning the bootloader**
  * Be sure to install the Arduino samd core before installing the MattairTech sam m0+ core. If you have problems upgrading
    the IDE to 1.6.6, you may need to uninstall both the Arduino and MattairTech cores, then re-install in the proper order.
    Use Arduino core 1.6.2 or above.

* **On Linux, disable modem manager (Ubuntu)**

* Do not perform a manual auto-reset (using a terminal program to change baud to 1200)

* Boards Manager must be opened twice to see some updates (only applies to some old IDE versions)

* **Boards manager might not install/uninstall the core or tools properly if the contents of the arduino15 directory has been manually modified**
  * Be sure to delete all manually installed folders (not just files)


## Bugs or Issues

If you find a bug you can submit an issue here on github:

https://github.com/mattairtech/ArduinoCore-samd/issues

Before posting a new issue, please check if the same problem has been already reported by someone else to avoid duplicates.


## Contributions

Contributions are always welcome. The preferred way to receive code cotribution is by submitting a Pull Request on github.


## License and Credits

This core has been developed by Arduino LLC in collaboration with Atmel.
This fork developed by Justin Mattair of MattairTech LLC.

```
  Copyright (c) 2015 Arduino LLC.  All right reserved.
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
```

### Petit FatFS

Petit FatFs module is an open source software to implement FAT file system to
small embedded systems. This is a free software and is opened for education,
research and commercial developments under license policy of following trems.

Copyright (C) 2014, ChaN, all right reserved.

* The Petit FatFs module is a free software and there is NO WARRANTY.
* No restriction on use. You can use, modify and redistribute it for
  personal, non-profit or commercial use UNDER YOUR RESPONSIBILITY.
* Redistributions of source code must retain the above copyright notice.
