# MattairTech MT-D11 (ATsamD11D14AM)

```
=========================== MattairTech MT-D11 (ATsamD11D14AM) ==========================
Other  COM    PWM   Analog  INT  Arduino*             Arduino*  INT   PWM     COM   Other
=========================================================================================
                                    -------------------
DAC                   *            | A2   | USB |  Gnd |
REF                   *            | A3   |     |  Vcc |
             TCC00    *      *     | A4    -----   A31 | 31     *    TC21     RX1   SWDIO
             TCC01    *      *     | A5            A30 | 30          TC20     TX1  SWDCLK
             TCC02    *            | A6            A27 | 27     *
             TCC03    *            | A7            A23 | 23                   SCL
   MOSI / TX2         *      *     | A10           A22 | 22     *             SDA
   SCK / RX2          *            | A11           A17 | 17          TC11
      MISO            *     NMI    | A14           A16 | 16     *    TC10             LED
BTN   SS              *      *     | A15           RST |                             BOOT
                                    -------------------

* Most pins can be used for more than one function. When using PIN_MAP_STANDARD, the port
  pin number printed on the board is also used in Arduino (but without the 'A') for all
  of the supported functions (ie: digitalRead(), analogRead(), analogWrite(), etc.). When
  using PIN_MAP_COMPACT, the Arduino numbering is sequential starting from 0 at the top
  left pin (A2). PIN_MAP_COMPACT uses less RAM.
* When USB CDC is enabled, Serial refers to SerialUSB, otherwise it refers to Serial1.
* Leave pin A30 floating (or use external pullup) during reset.
* Tone available on TC2. DO NOT connect voltages higher than 3.3V!
```


# Pins descriptions for the MattairTech MT-D11
## PIN_MAP_STANDARD
============================================================================================================================================
Arduino	| Silk	| Port	| Alternate Function	| Comments (! means not used with this peripheral assignment)
--------|-------|-------|-----------------------|-------------------------------------------------------------------------------------------
0	| ---	| ----	| NOT A PIN		| NOT A PIN
1	| ---	| ----	| NOT A PIN		| NOT A PIN
2	| A2	| PA02	| DAC			| !EIC/EXTINT[2] ADC/AIN[0] PTC/Y[0] DAC/VOUT
3	| A3	| PA03	| REFA			| !EIC/EXTINT[3] REF/ADC/VREFA REF/DAC/VREFA ADC/AIN[1] PTC/Y[1]
4	| A4	| PA04	| REFB / VM		| EIC/EXTINT[4] REF/ADC/VREFB ADC/AIN[4] AC/AIN[0] PTC/Y[2] !SERCOM0/PAD[2] !SERCOM0/PAD[0] !TC1/WO[0] TCC0/WO[0]
5	| A5	| PA05	| 			| EIC/EXTINT[5] ADC/AIN[3] AC/AIN[1] PTC/Y[3] !SERCOM0/PAD[3] !SERCOM0/PAD[1] !TC1/WO[1] TCC0/WO[1]
6	| A6	| PA06	| 			| !EIC/EXTINT[6] ADC/AIN[4] AC/AIN[2] PTC/Y[4] !SERCOM0/PAD[0] !SERCOM0/PAD[2] !TC2/WO[0] TCC0/WO[2]
7	| A7	| PA07	| 			| !EIC/EXTINT[7] ADC/AIN[5] AC/AIN[3] PTC/Y[5] !SERCOM0/PAD[1] !SERCOM0/PAD[3] !TC2/WO[1] TCC0/WO[3]
8	| --	| PA08	| Xin32 / Xin		| Xin32
9	| --	| PA09	| Xout32 / Xout		| Xout32
10	| A10	| PA10	| SPI MOSI / TX2	| EIC/EXTINT[2] ADC/AIN[8] PTC/X[2] PTC/Y[8] SERCOM0/PAD[2] !SERCOM2/PAD[2] !TC2/WO[0] !TCC0/WO[2]
11	| A11	| PA11	| SPI SCK / RX2		| !EIC/EXTINT[3] ADC/AIN[9] PTC/X[3] PTC/Y[9] SERCOM0/PAD[3] !SERCOM2/PAD[3] !TC2/WO[1] !TCC0/WO[3]
12	| ---	| ----	| NOT A PIN		| NOT A PIN
13	| ---	| ----	| NOT A PIN		| NOT A PIN
14	| A14	| PA14	| SPI MISO		| EIC/NMI ADC/AIN[6] PTC/X[0] PTC/Y[6] SERCOM0/PAD[0] !SERCOM2/PAD[0] !TC1/WO[0] !TCC0/WO[0]
15	| A15	| PA15	| Button / SPI SS	| EIC/EXTINT[1] ADC/AIN[7] PTC/X[1] PTC/Y[7] SERCOM0/PAD[1] !SERCOM2/PAD[1] !TC1/WO[1] !TCC0/WO[1] Button
16	| A16	| PA16	| LED			| EIC/EXTINT[0] PTC/X[4] PTC/Y[10] !SERCOM1/PAD[2] !SERCOM2/PAD[2] TC1/WO[0] !TCC0/WO[6] LED
17	| A17	| PA17	| HOST_ENABLE		| !EIC/EXTINT[1] PTC/X[5] PTC/Y[11] !SERCOM1/PAD[3] !SERCOM2/PAD[3] TC1/WO[1] !TCC0/WO[7] HOST_ENABLE
18	| ---	| ----	| NOT A PIN		| NOT A PIN
19	| ---	| ----	| NOT A PIN		| NOT A PIN
20	| ---	| ----	| NOT A PIN		| NOT A PIN
21	| ---	| ----	| NOT A PIN		| NOT A PIN
22	| A22	| PA22	| I2C/SDA w/pullup	| EIC/EXTINT[6] PTC/X[6] PTC/Y[12] !SERCOM1/PAD[0] SERCOM2/PAD[0] !TC1/WO[0] !TCC0/WO[4]
23	| A23	| PA23	| I2C/SCL w/pullup	| !EIC/EXTINT[7] PTC/X[7] PTC/Y[13] !SERCOM1/PAD[1] SERCOM2/PAD[0] !TC1/WO[1] !TCC0/WO[5]
24	| ---	| PA24	| USB_NEGATIVE		| USB/DM
25	| ---	| PA25	| USB_POSITIVE		| USB/DP
26	| ---	| ----	| NOT A PIN		| NOT A PIN
27	| A27	| PA27	| 			| EIC/EXTINT[7] PTC/X[10]
28	| A28	| PA28	| Reset			| Reset, BOOT (double tap bootloader entry)
29	| ---	| ----	| NOT A PIN		| NOT A PIN
30	| A30	| PA30	| TX1 / SWD CLK		| !EIC/EXTINT[2] !SERCOM1/PAD[0] SERCOM1/PAD[2] TC2/WO[0] !TCC0/WO[2] SWD CLK, leave floating during boot
31	| A31	| PA31	| RX1 / SWD IO		| EIC/EXTINT[3] !SERCOM1/PAD[1] SERCOM1/PAD[3] TC2/WO[1] !TCC0/WO[3] SWD IO
============================================================================================================================================

## PIN_MAP_COMPACT
============================================================================================================================================
Arduino	| Silk	| Port	| Alternate Function	| Comments (! means not used with this peripheral assignment)
--------|-------|-------|-----------------------|-------------------------------------------------------------------------------------------
0	| A2	| PA02	| DAC			| !EIC/EXTINT[2] ADC/AIN[0] PTC/Y[0] DAC/VOUT
1	| A3	| PA03	| REFA			| !EIC/EXTINT[3] REF/ADC/VREFA REF/DAC/VREFA ADC/AIN[1] PTC/Y[1]
2	| A4	| PA04	| REFB / VM		| EIC/EXTINT[4] REF/ADC/VREFB ADC/AIN[4] AC/AIN[0] PTC/Y[2] !SERCOM0/PAD[2] !SERCOM0/PAD[0] !TC1/WO[0] TCC0/WO[0]
3	| A5	| PA05	| 			| EIC/EXTINT[5] ADC/AIN[3] AC/AIN[1] PTC/Y[3] !SERCOM0/PAD[3] !SERCOM0/PAD[1] !TC1/WO[1] TCC0/WO[1]
4	| A6	| PA06	| 			| !EIC/EXTINT[6] ADC/AIN[4] AC/AIN[2] PTC/Y[4] !SERCOM0/PAD[0] !SERCOM0/PAD[2] !TC2/WO[0] TCC0/WO[2]
5	| A7	| PA07	| 			| !EIC/EXTINT[7] ADC/AIN[5] AC/AIN[3] PTC/Y[5] !SERCOM0/PAD[1] !SERCOM0/PAD[3] !TC2/WO[1] TCC0/WO[3]
6	| A10	| PA10	| SPI MOSI / TX2	| EIC/EXTINT[2] ADC/AIN[8] PTC/X[2] PTC/Y[8] SERCOM0/PAD[2] !SERCOM2/PAD[2] !TC2/WO[0] !TCC0/WO[2]
7	| A11	| PA11	| SPI SCK / RX2		| !EIC/EXTINT[3] ADC/AIN[9] PTC/X[3] PTC/Y[9] SERCOM0/PAD[3] !SERCOM2/PAD[3] !TC2/WO[1] !TCC0/WO[3]
8	| A14	| PA14	| SPI MISO		| EIC/NMI ADC/AIN[6] PTC/X[0] PTC/Y[6] SERCOM0/PAD[0] !SERCOM2/PAD[0] !TC1/WO[0] !TCC0/WO[0]
9	| A15	| PA15	| Button / SPI SS	| EIC/EXTINT[1] ADC/AIN[7] PTC/X[1] PTC/Y[7] SERCOM0/PAD[1] !SERCOM2/PAD[1] !TC1/WO[1] !TCC0/WO[1] Button
10	| A16	| PA16	| LED			| EIC/EXTINT[0] PTC/X[4] PTC/Y[10] !SERCOM1/PAD[2] !SERCOM2/PAD[2] TC1/WO[0] !TCC0/WO[6] LED
11	| A17	| PA17	| HOST_ENABLE		| !EIC/EXTINT[1] PTC/X[5] PTC/Y[11] !SERCOM1/PAD[3] !SERCOM2/PAD[3] TC1/WO[1] !TCC0/WO[7] HOST_ENABLE
12	| A22	| PA22	| I2C/SDA w/pullup	| EIC/EXTINT[6] PTC/X[6] PTC/Y[12] !SERCOM1/PAD[0] SERCOM2/PAD[0] !TC1/WO[0] !TCC0/WO[4]
13	| A23	| PA23	| I2C/SCL w/pullup	| !EIC/EXTINT[7] PTC/X[7] PTC/Y[13] !SERCOM1/PAD[1] SERCOM2/PAD[0] !TC1/WO[1] !TCC0/WO[5]
14	| A27	| PA27	| 			| EIC/EXTINT[7] PTC/X[10]
15	| A30	| PA30	| TX1 / SWD CLK		| !EIC/EXTINT[2] !SERCOM1/PAD[0] SERCOM1/PAD[2] TC2/WO[0] !TCC0/WO[2] SWD CLK, leave floating during boot
16	| A31	| PA31	| RX1 / SWD IO		| EIC/EXTINT[3] !SERCOM1/PAD[1] SERCOM1/PAD[3] TC2/WO[1] !TCC0/WO[3] SWD IO
============================================================================================================================================

* Most pins can be used for more than one function. When using PIN_MAP_STANDARD, the port
  pin number printed on the board is also used in Arduino (but without the 'A') for all
  of the supported functions (ie: digitalRead(), analogRead(), analogWrite(), etc.). When
  using PIN_MAP_COMPACT, the Arduino numbering is sequential starting from 0 at the top
  left pin (A2). PIN_MAP_COMPACT uses less RAM.
* The following Arduino pin numbers are not mapped to a physical pin: 0, 1, 8, 9, 12, 13, 18, 19, 20, 21, 24, 25, 26, 28, and 29.
* Pins 24 and 25 are in use by USB (USB_NEGATIVE and USB_POSITIVE).
* Leave pin A30 floating (or use external pullup) during reset.
* Pins 8 and 9 are by default connected to the 32.768KHz crystal.
* The tone library uses TC2.


# Board Configuration Notes

TODO: Update this

* Either the 32.768KHz crystal or the 16MHz crystal can be used. Be sure to set the correct solder jumpers.
* The bootloader does not use an external crystal by default. Double-tap the reset button to enter.
* The button is connected to the Reset pin by default, but can be connected to pin 15 via the solder jumper.
* BTN pin is shared with SPI SS, so the button must be configured as reset (default) when using SPI.
* A reference voltage can be connected to REFA or REFB. In these cases, the capacitors should be enabled via the solder jumpers.
* The I2C (TWI) pullup resistors are enabled by default.
* The LED is enabled by default.

* **Digital: All pins can be used for general purpose I/O** 
  * Supports INPUT, OUTPUT, INPUT_PULLUP, and INPUT_PULLDOWN.
  * Each pin can source or sink a maximum of 7 mA (when PER_ATTR_DRIVE_STRONG is set for the pin).
  * Internal pull-up and pull-down resistors of 20-60 Kohms (40Kohm typ., disconnected by default).
  * Use the pinMode(), digitalWrite(), and digitalRead() functions.
* **Analog Inputs: 10 pins can be configured as ADC analog inputs.**
  * These are available using the analogRead() function.
  * All pins can be used for GPIO and some pins can be used for other digital functions (ie. pwm or serial).
  * Each pin provides 10 bits of resolution (1024 values) by default.
  * 12-bit resolution supported by using the analogReadResolution() function.
  * Each pin measures from ground to 3.3 volts.
  * The upper end of the measurement range can be changed using the AREF pin and the analogReference() function.
* **DAC: One analog output is available on pin 2.**
  * Provides a 10-bit voltage output with the analogWrite() function.
* **PWM: 12 pins (MT-D21E) or 8 pins (MT-D11) can be configured as PWM outputs.**
  * Available using the analogWrite() function.
  * Each pin provides 8 bits of resolution (256 values) by default.
  * 12-bit resolution supported by using the analogWriteResolution() function.
* **External Interrupts: 15 pins (MT-D21E) or 9 pins (MT-D11) can be configured with external interrupts.**
  * Available using the attachInterrupt() function.
* **Serial: 2 pairs of pins (MT-D21E) or 1 pair (MT-D11) can be configured for TTL serial I/O.**
  * MT-D21E: Serial1: pin 11 (RX) and pin 10 (TX). Serial2: pin 15 (RX) and pin 14 (TX).
  * MT-D11: Serial1: pin 31 (RX) and pin 30 (TX).
* **SPI: 3 or 4 pins can be configured for SPI I/O (SPI).**
  * MT-D21E: Pin 18 (MOSI), pin 19 (SCK), pin 22 (MISO), and optionally pin 23 (SS, not currently used).
  * MT-D11: Pin 10 (MOSI), pin 11 (SCK), pin 14 (MISO), and optionally pin 15 (SS, not currently used).
  * SPI communication using the SPI library.
  * Note that the SPI library will set SS as an output.
  * On the MT-D11, the button must be configured as reset (default) when using SPI.
* **TWI (I2C): 2 pins can be configured for TWI I/O (Wire).**
  * MT-D21E: Pin 16 (SDA) and pin 17 (SCL).
  * MT-D11: Pin 22 (SDA) and pin 23 (SCL).
  * TWI communication using the Wire library.
* **LED: One pin can be configured to light the onboard LED (LED_BUILTIN).**
  * Pin 28 (MT-D21E) or pin 16 (MT-D11). Bring the pin HIGH to turn the LED on. The pullup is disabled on this pin.
* **Button: One pin can be configured to read the onboard Button A (BUTTON_BUILTIN).**
  * Pin 27 (MT-D21E) or pin 15 (MT-D11). Pressing the button will bring the pin LOW. The pullup must be enabled first.
  * If the debouncing capacitor is connected, delay reading the pin at least 6ms after turning on the pullup.
* **AREF: One pin can be configured as an AREF analog input.**
  * The upper end of the analog measurement range can be changed using the analogReference() function.
* **Reset: Bring this line LOW to reset the microcontroller.**



# PinDescription table format

## Note that a new column (GCLKCCL) was added for 1.6.8-beta-b0.
MATTAIRTECH_ARDUINO_SAMD_VARIANT_COMPLIANCE in variant.h is used to track versions.
If using board variant files with the old format, the new core will still read the
table the old way, losing any new features introduced by the new column. Additionally,
new definitions have been added for L21 and C21 support.

## Each pin can have multiple functions.
The PinDescription table describes how each of the pins can be used by the Arduino
core. Each pin can have multiple functions (ie: ADC input, digital output, PWM,
communications, etc.), and the PinDescription table configures which functions can
be used for each pin. This table is mainly accessed by the pinPeripheral function in
wiring_private.c, which is used to attach a pin to a particular peripheral function.
The communications drivers (ie: SPI, I2C, and UART), analogRead(), analogWrite(),
analogReference(), attachInterrupt(), and pinMode() all call pinPeripheral() to
verify that the pin can perform the function requested, and to configure the pin for
that function. Most of the contents of pinMode() are now in pinPeripheral().

## Pin Mapping
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
from the MT-D11 variant. The MT-D21J combines both methods, using the actual port pin 
designators from both PORTA and PORTB for arduino numbers 0-31 (ie: B1=1, A2=2), then
using arduino numbering only above 31. For 0-31 only one pin from PORTA or PORTB can be
used, leaving the other pin for some number above 31.

## See WVariant.h in cores/arduino for the definitions used in the table.

### Port:
This is the port (ie: PORTA).

### Pin:
This is the pin (bit) within the port. Valid values are 0-31.

### PinType:
This indicates what peripheral function the pin can be attached to. In most cases,
this is PIO_MULTI, which means that the pin can be anything listed in the PinAttribute
field. It can also be set to a specific peripheral. In this case, any attempt to
configure the pin (using pinPeripheral or pinMode) as anything else will fail (and
pinPeripheral will return -1). This can be used to prevent accidental re-configuration
of a pin that is configured for only one function (ie: USB D- and D+ pins). If a pin
is not used or does not exist, PIO_NOT_A_PIN must be entered in this field. See
WVariant.h for valid entries. These entries are also used as a parameter to
pinPeripheral() with the exception of PIO_NOT_A_PIN and PIO_MULTI. The pinMode function
now calls pinPeripheral() with the desired mode. Note that this field is not used to
select between the two peripherals possible with each of the SERCOM and TIMER functions.
PeripheralAttribute is now used for this.

### PeripheralAttribute:
This is an 8-bit bitfield used for various peripheral configuration. It is primarily
used to select between the two peripherals possible with each of the SERCOM and TIMER
functions. TIMER pins are individual, while SERCOM uses a group of two to four pins.
This group of pins can span both peripherals. For example, pin 19 (SPI1 SCK) on the
MT-D21E uses PER_ATTR_SERCOM_ALT while pin 22 (SPI1 MISO) uses PER_ATTR_SERCOM_STD.
Both TIMER and SERCOM can exist for each pin. This bitfield is also used to set the
pin drive strength. In the future, other attributes (like input buffer configuration)
may be added. Starting with 1.6.8, the ADC instance on the C21 (there are two) is also
selected here. See WVariant.h for valid entries.

### PinAttribute
This is a 32-bit bitfield used to list all of the valid peripheral functions that a
pin can attach to. This includes GPIO functions like PIN_ATTR_OUTPUT. Certain
attributes are shorthand for a combination of other attributes. PIN_ATTR_DIGITAL
includes all of the GPIO functions, while PIN_ATTR_TIMER includes both
PIN_ATTR_TIMER_PWM and PIN_ATTR_TIMER_CAPTURE (capture is not used yet).
PIN_ATTR_ANALOG is an alias to PIN_ATTR_ANALOG_ADC. This bitfield is useful for
limiting a pin to only input related functions or output functions. This allows a pin
to have a more flexible configuration, while restricting the direction (ie: to avoid
contention). See WVariant.h for valid entries.

### TCChannel
This is the TC/TCC channel (if any) assigned to the pin. Some TC channels are available
on multiple pins. In general, only one pin should be configured in the pinDescription
table per TC channel. Starting with 1.6.8, the timer type is now encoded in this column
to support the L21 and C21, which use TC numbers starting at 0 (rather than 3 as on the
D21). See WVariant.h for valid entries.

### ADCChannelNumber
This is the ADC channel (if any) assigned to the pin. The C21 has two ADC instances,
which is selected in the PeripheralAttribute column. See WVariant.h for valid entries.

### ExtInt
This is the interrupt (if any) assigned to the pin. Some interrupt numbers are
available on multiple pins. In general, only one pin should be configured in the
pinDescription table per interrupt number. Thus, for example, if an interrupt was
needed on pin 2, EXTERNAL_INT_2 can be moved from pin 18. See WVariant.h for valid
entries.

### GCLKCCL
This column was added in 1.6.8-beta-b0. It is not yet used. It will eventually support
the Analog Comparators (AC), the Configurable Custom Logic (CCL) units of the L21 and
C21, and the GCLK outputs (inputs).
