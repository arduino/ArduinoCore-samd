# MattairTech Xeno Mini (ATSAMD51G/ATSAMD21G/ATSAML21G/ATSAMC21G)

```
======== MattairTech Xeno Mini (Rev B) (ATSAMD51G/ATSAMD21G/ATSAML21G/ATSAMC21G) ========
Alt   COM    PWM  Analog  INT  Arduino*            Arduino*  INT  Analog  PWM   COM   Alt
=========================================================================================
                                   -------------------
HB1+ (MTR)  TX1   TC~   O   I   0 | B8            RST |                              BOOT
HB2+ (MTR)  RX1   TC~   O   I   1 | B9             B3 | 35         O                Vcoin
DAC                     O       2 | A2             B2 | 34         O                  LED
REFA*                   O       3 | A3            B23 | 33   I                 SLP+ (MTR)
REFB                    O       4 | A4            B22 | 32   I                INT+! (IMU)
VI (VIN) / DAC1*        O       5 | A5            A31 | 31           TCC~        IO (DGB)
VU! (USB)               O       6 | A6            A30 | 30           TCC~       CLK (DGB)
VB (VBAT)               O       7 | A7            A27 | 27
MOSI (MEM)              O       8 | A8            A23 | 23   TCC0~ SCL1/SCK1*  EN1+ (MTR)
MISO (MEM)              O       9 | A9            A22 | 22   TCC0~ SDA1/MOSI1* EN2+ (MTR)
D2 (QSPI)               O      10 | A10           A21 | 21   I    TCC0~  MISO1
D3 (QSPI) / SCK (MEM)   O   I  11 | A11           A20 | 20   I    TCC0~
            TX2   TCC~      I  12 | A12           A19 | 19   I    TC~    RX3
CEN (CHG)   RX2   TCC~      I  13 | A13           A18 | 18   I    TC~    TX3  STA+! (CHG)
SCK (QSPI)        TC~       I  14 | B10           A17 | 17   I                  SCL (I2C)
CS (MEM)          TC~          15 | B11           A16 | 16   I                  SDA (I2C)
                                  | Vaux         VccL |
USB D- / CAN TX                   | A24   _____  VccH |   ! VccL is 3.3V by default.
USB D+ / CAN RX                   | A25  |     |  Vin |     DO NOT exceed 3.6V on VccL or
                                  | Gnd  | USB |  Gnd |     any IO pin with the D51, D21,
     USB: D51/D21/L21 only         -------------------      or L21 installed. 5V allowed
       CAN: D51/C21 only                                    ONLY with the C21 installed.

* Most pins can be used for more than one function. The port pin number printed on the
  board is also used in Arduino for all 'A' pins. For 'B' pins, see Arduino column.
  DAC1 is present only on the D51 and L21. With the D51, REFA is tied to VccL (J20).
  For the D51, the COM pins on A22 and A23 are reversed (ie: A22 is instead SCL1/SCK1).

+ This header pin has limited use because the alternate function, if installed, cannot be
  disconnected via solder jumper. EN1, EN2, HB1, and HB2 have pulldowns, but can be used
  if SLP is low. Note that Rev A boards differ with regards to the MTR pins (see docs).

! These pins should not be driven if the associated hardware is installed. This variant
  will only allow configuring these pins as inputs. To change this, edit configuration in
  variant.h. INT from IMU will be driven low after reset. See IMU docs.

~ D51: 3 TCC (6,4,3 ch.), 4 TC (2 ch.). D21/L21/C21: 3 TCC (4,2,2 ch.), 3 TC (2 ch.).
  The D51 adds timers to pins A4, A5, A6, A7, A16, A17, and B2, however, the timers on
  B8, B9, B10, and B11 are not present. The C21 adds timers to pins B2, B3, B22, and B23.

Silkscreen Legend:
  Top: A circle around pin is analog function, '~' is timer, small 'I' is interrupt
  Bottom: A box around pin means 'Alt' function enabled by default if installed
```


## Board Configuration Notes

* **Crystals**
  * Either the 32.768KHz crystal or the 24MHz crystal can be used. These pins do not route to headers.
  * The bootloader does not use an external crystal by default. Double-tap the reset button to enter manually.

* **LED (LED_BUILTIN)**
  * Bring the pin HIGH to turn the LED on.
  * The LED is enabled (solder jumper) by default.

* **GPIO** 
  * All pins (including analog) support INPUT, OUTPUT, INPUT_PULLUP, and INPUT_PULLDOWN.
  * When PER_ATTR_DRIVE_STRONG is set for the pin (enabled by default), each pin can source or sink a maximum of:
    * **D51:** 8mA high, 8mA low
    * **D21:** 7mA high, 10mA low
    * **L21:** 5mA high, 6mA low (8 high drive pins: 10mA high, 12mA low)
    * **C21:** 6mA high, 10mA low (2 high drive pins (A10, A11): 12mA high, 20mA low)
  * Internal pull-up and pull-down resistors of 20-60 Kohms (40Kohm typ., disconnected by default).

* **Analog Inputs**
  * Up to 14 pins can be configured as ADC analog inputs.
  * Each pin measures from ground to 3.3 volts by default.
  * Each pin provides 10 bits of resolution (1024 values) by default.
  * 12-bit resolution supported by using the analogReadResolution() function.
  * The upper end of the measurement range can be changed using the analogReference() function.
  * A reference voltage can be connected to REFA. In this case, the capacitors should be enabled via solder jumper J33.
  * Due to errata with the D51 DAC, J20 is set to route VccL to the MCU REFA pin (the header pin is disconnected).

* **DAC**
  * D21/C21: One 10-bit 350Ksps analog output is available on pin 2.
  * D51/L21: Two 12-bit 1Msps analog outputs are available on pins 2 and 5.
  * Due to errata with the D51 DAC, J20 is set to route VccL to the MCU REFA pin (the header pin is disconnected).

* **PWM**
  * Up to 14 pins can be configured as PWM outputs (17 for D51, 18 for C21).
  * Each pin provides 8 bits of resolution (256 values) by default.
  * 12-bit resolution supported by using the analogWriteResolution() function.

* **External Interrupts**
  * Up to 14 pins can be configured with external interrupts.

* **SERCOM**
  * 6 SERCOM are available.
  * Up to 3 UART instances (two for D51). More in a future release.
  * Up to 2 SPI instances.
  * Up to 2 WIRE (I2C) instances.
  * The WIRE pullup resistors are enabled by default.



## PinDescription table format

### Note that in 1.6.18-beta-b1 a new compact table format was added.
The standard PinDescription table uses 12 bytes per pin. Define PIN_DESCRIPTION_TABLE_SIMPLE
to use a more compact format that uses only 4 bytes per pin (currently only available
for the D11 chips). In this case, the PinType, PinAttribute, and GCLKCCL columns are not used
(they are not required). Additionally, the SetPortPin() and SetExtIntADC() macros are used to
pack Port and Pin into the PortPin column, and ExtInt and ADCChannelNumber into the ExtIntADC
column. Note that external libraries that reference the PinDescription table directly (uncommon)
will no longer work. This define can be combined with the PIN_MAP_COMPACT define, which
is available in variant.h of the D11 variants. This can save from 10's to over 200 bytes.

### Note that a new column (GCLKCCL) was added for 1.6.8-beta-b0.
MATTAIRTECH_ARDUINO_SAMD_VARIANT_COMPLIANCE in variant.h is used to track versions.
If using board variant files with the old format, the new core will still read the
table the old way, losing any new features introduced by the new column. Additionally,
new definitions have been added for D51, L21, and C21 support.

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
from the MT-D11 variant. The Xeno combines both methods, using the actual port pin 
designators from both PORTA and PORTB for arduino numbers 0-31 (ie: B1=1, A2=2), then
using arduino numbering only above 31. For 0-31 only one pin from PORTA or PORTB can be
used, leaving the other pin for some number above 31.

**See [WVariant.h](https://github.com/mattairtech/ArduinoCore-samd/tree/master/cores/arduino/WVariant.h) for the definitions used in the table.**

### Port
This is the port (ie: PORTA). Not used with PIN_DESCRIPTION_TABLE_SIMPLE.

### Pin
This is the pin (bit) within the port. Valid values are 0-31. Not used with
PIN_DESCRIPTION_TABLE_SIMPLE.

### SetPortPin()
When PIN_DESCRIPTION_TABLE_SIMPLE is defined, Port and Pin are combined into one column
using the SetPortPin() packing macro: SetPortPin(PORTA, 2). If the pin is not usable,
use SetPortPin(NOT_A_PORT, 0).

### PinType
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
select between the different peripherals possible with each of the SERCOM and TIMER
functions. PeripheralAttribute is now used for this. When PIN_DESCRIPTION_TABLE_SIMPLE
is defined, PinType is not used (the pin is treated as PIO_MULTI).

### PeripheralAttribute
This is an 8-bit bitfield used for various peripheral configuration. It is primarily
used to select between the different peripherals possible with each of the SERCOM and
TIMER functions. TIMER pins are individual, while SERCOM uses a group of two to four
pins. This group of pins can span both peripherals. For example, pin 19 (SPI1 SCK) on
the MT-D21E uses PER_ATTR_SERCOM_ALT while pin 22 (SPI1 MISO) uses PER_ATTR_SERCOM_STD.
Both TIMER and SERCOM can exist for each pin. This bitfield is also used to set the
pin drive strength and the input buffer configuration (ie: totem-pole, open-drain,
buskeeper, etc.). Starting with 1.6.8, the ADC instance on the D51 and C21 (there are
two) is also selected here. Note that the D51 adds a third timer attribute and
requires consultation of the datasheet IOSET tables. See WVariant.h for valid entries.

### PinAttribute
This is a 32-bit bitfield used to list all of the valid peripheral functions that a
pin can attach to. This includes GPIO functions like PIN_ATTR_OUTPUT. Certain
attributes are shorthand for a combination of other attributes. PIN_ATTR_DIGITAL
includes all of the GPIO functions, while PIN_ATTR_TIMER includes both
PIN_ATTR_TIMER_PWM and PIN_ATTR_TIMER_CAPTURE (capture is not used yet).
PIN_ATTR_ANALOG is an alias to PIN_ATTR_ANALOG_ADC. This bitfield is useful for
limiting a pin to only input related functions or output functions. This allows a pin
to have a more flexible configuration, while restricting the direction (ie: to avoid
contention). See WVariant.h for valid entries. Not used with PIN_DESCRIPTION_TABLE_SIMPLE.

### TCChannel
This is the TC/TCC channel (if any) assigned to the pin. Some TC channels are available
on multiple pins. In general, only one pin should be configured in the pinDescription
table per TC channel. Starting with 1.6.8, the timer type is now encoded in this column
to support the D51, L21 and C21, which use TC numbers starting at 0 (rather than 3 as on
the D21). See WVariant.h for valid entries.

### ADCChannelNumber
This is the ADC channel (if any) assigned to the pin. The D51 and C21 each have two ADC
instances, which are selected in the PeripheralAttribute column. See WVariant.h for
valid entries. Not used with PIN_DESCRIPTION_TABLE_SIMPLE.

### ExtInt
This is the interrupt (if any) assigned to the pin. Some interrupt numbers are
available on multiple pins. In general, only one pin should be configured in the
pinDescription table per interrupt number. Thus, for example, if an interrupt was
needed on pin 2, EXTERNAL_INT_2 can be moved from pin 18. See WVariant.h for valid
entries. Not used with PIN_DESCRIPTION_TABLE_SIMPLE.

### SetExtIntADC()
When PIN_DESCRIPTION_TABLE_SIMPLE is defined, ExtInt and ADCChannelNumber are combined
into one column using the SetExtIntADC() packing macro: SetExtIntADC(EXTERNAL_INT_4,
ADC_Channel2). If the pin is not usable, use SetExtIntADC(EXTERNAL_INT_NONE, No_ADC_Channel).

### GCLKCCL
This column was added in 1.6.8-beta-b0. It is not yet used. It will eventually support
the Analog Comparators (AC), the Configurable Custom Logic (CCL) peripherals of the D51,
L21 and C21, and the GCLK outputs (inputs) of all of the MCUs. Not used with
PIN_DESCRIPTION_TABLE_SIMPLE.
