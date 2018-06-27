# Pins descriptions for the Arduino M0 / M0 Pro

```
/*
 * 
 * + Pin number +  ZERO Board pin  |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+------------------------------
 * |            | Digital Low      |        |                 |
 * +------------+------------------+--------+-----------------+------------------------------
 * | 0          | 0 -> RX          |  PA11  |                 | EIC/EXTINT[11] ADC/AIN[19] PTC/X[3] *SERCOM0/PAD[3] SERCOM2/PAD[3] TCC1/WO[1] TCC0/WO[3]
 * | 1          | 1 <- TX          |  PA10  |                 | EIC/EXTINT[10] ADC/AIN[18] PTC/X[2] *SERCOM0/PAD[2] TCC1/WO[0] TCC0/WO[2]  
 * | 2          | ~2               |  PA08  |                 | EIC/NMI ADC/AIN[16] PTC/X[0] SERCOM0/PAD[0] SERCOM2/PAD[0] *TCC0/WO[0] TCC1/WO[2]
 * | 3          | ~3               |  PA09  |                 | EIC/EXTINT[9] ADC/AIN[17] PTC/X[1] SERCOM0/PAD[1] SERCOM2/PAD[1] *TCC0/WO[1] TCC1/WO[3]
 * | 4          | ~4               |  PA14  |                 | EIC/EXTINT[14] SERCOM2/PAD[2] SERCOM4/PAD[2] TC3/WO[0] *TCC0/WO[4]
 * | 5          | ~5               |  PA15  |                 | EIC/EXTINT[15] SERCOM2/PAD[3] SERCOM4/PAD[3] TC3/WO[1] *TCC0/WO[5]
 * | 6          | ~6               |  PA20  |                 | EIC/EXTINT[4] PTC/X[8] SERCOM5/PAD[2] SERCOM3/PAD[2] TC7/WO[0] *TCC0/WO[6]
 * | 7          | ~7               |  PA21  |                 | EIC/EXTINT[5] PTC/X[9] SERCOM5/PAD[3] SERCOM3/PAD[3] TC7/WO[1] *TCC0/WO[7]
 * +------------+------------------+--------+-----------------+------------------------------
 * |            | Digital High     |        |                 |
 * +------------+------------------+--------+-----------------+------------------------------
 * | 8          | ~8               |  PA06  |                 | EIC/EXTINT[6] PTC/Y[4] ADC/AIN[6] AC/AIN[2] SERCOM0/PAD[2] *TCC1/WO[0]
 * | 9          | ~9               |  PA07  |                 | EIC/EXTINT[7] PTC/Y[5] DC/AIN[7] AC/AIN[3] SERCOM0/PAD[3] *TCC1/WO[1]
 * | 10         | ~10              |  PA18  |                 | EIC/EXTINT[2] PTC/X[6] SERCOM1/PAD[2] SERCOM3/PAD[2] *TC3/WO[0] TCC0/WO[2]
 * | 11         | ~11              |  PA16  |                 | EIC/EXTINT[0] PTC/X[4] SERCOM1/PAD[0] SERCOM3/PAD[0] *TCC2/WO[0] TCC0/WO[6]
 * | 12         | ~12              |  PA19  |                 | EIC/EXTINT[3] PTC/X[7] SERCOM1/PAD[3] SERCOM3/PAD[3] *TC3/WO[1] TCC0/WO[3]
 * | 13         | ~13              |  PA17  | LED             | EIC/EXTINT[1] PTC/X[5] SERCOM1/PAD[1] SERCOM3/PAD[1] *TCC2/WO[1] TCC0/WO[7]
 * | 14         | GND              |        |                 |
 * | 15         | AREF             |  PA03  |                 | *DAC/VREFP PTC/Y[1]
 * | 16         | SDA              |  PA22  |                 | EIC/EXTINT[6] PTC/X[10] *SERCOM3/PAD[0] SERCOM5/PAD[0] TC4/WO[0] TCC0/WO[4]
 * | 17         | SCL              |  PA23  |                 | EIC/EXTINT[7] PTC/X[11] *SERCOM3/PAD[1] SERCOM5/PAD[1] TC4/WO[1] TCC0/WO[5]
 * +------------+------------------+--------+-----------------+------------------------------
 * |            |SPI (Legacy ICSP) |        |                 |
 * +------------+------------------+--------+-----------------+------------------------------
 * | 18         | 1                |  PA12  | MISO            | EIC/EXTINT[12] SERCOM2/PAD[0] *SERCOM4/PAD[0] TCC2/WO[0] TCC0/WO[6]
 * | 19         | 2                |        | 5V0             |
 * | 20         | 3                |  PB11  | SCK             | EIC/EXTINT[11]                *SERCOM4/PAD[3] TC5/WO[1] TCC0/WO[5]
 * | 21         | 4                |  PB10  | MOSI            | EIC/EXTINT[10]                *SERCOM4/PAD[2] TC5/WO[0] TCC0/WO[4]
 * | 22         | 5                |        | RESET           |
 * | 23         | 6                |        | GND             |
 * +------------+------------------+--------+-----------------+------------------------------
 * |            | Analog Connector |        |                 |
 * +------------+------------------+--------+-----------------+------------------------------
 * | 24         | A0               |  PA02  |                 | EIC/EXTINT[2] *ADC/AIN[0] PTC/Y[0] DAC/VOUT
 * | 25         | A1               |  PB08  |                 | EIC/EXTINT[8] *ADC/AIN[2] PTC/Y[14] SERCOM4/PAD[0] TC4/WO[0]
 * | 26         | A2               |  PB09  |                 | EIC/EXTINT[9] *ADC/AIN[3] PTC/Y[15] SERCOM4/PAD[1] TC4/WO[1]
 * | 27         | A3               |  PA04  |                 | EIC/EXTINT[4] *ADC/AIN[4] AC/AIN[0] PTC/Y[2] SERCOM0/PAD[0] TCC0/WO[0]
 * | 28         | A4               |  PA05  |                 | EIC/EXTINT[5] *ADC/AIN[5] AC/AIN[1] PTC/Y[5] SERCOM0/PAD[1] TCC0/WO[1]
 * | 29         | A5               |  PB02  |                 | EIC/EXTINT[2] *ADC/AIN[10] PTC/Y[8] SERCOM5/PAD[0] TC6/WO[0]
 * +------------+------------------+--------+-----------------+------------------------------
 * |            | LEDs             |        |                 |
 * +------------+------------------+--------+-----------------+------------------------------
 * | 30         |                  |  PB03  | RX              |
 * | 31         |                  |  PA27  | TX              |
 * +------------+------------------+--------+-----------------+------------------------------
 * |            | USB              |        |                 |
 * +------------+------------------+--------+-----------------+------------------------------
 * | 32         |                  |  PA28  | USB HOST ENABLE |
 * | 33         |                  |  PA24  | USB_NEGATIVE    | USB/DM
 * | 34         |                  |  PA25  | USB_POSITIVE    | USB/DP
 * +------------+------------------+--------+-----------------+------------------------------
 * |            | EDBG             |        |                 |
 * +------------+------------------+--------+-----------------+------------------------------
 * | 35         |                  |  PB22  | EDBG_UART TX    | SERCOM5/PAD[2]
 * | 36         |                  |  PB23  | EDBG_UART RX    | SERCOM5/PAD[3]
 * +------------+------------------+--------+-----------------+------------------------------
 * | 37         |                  |  PA22  | EDBG_SDA        | SERCOM3/PAD[0]
 * | 38         |                  |  PA23  | EDBG_SCL        | SERCOM3/PAD[1]
 * +------------+------------------+--------+-----------------+------------------------------
 * | 39         |                  |  PA19  | EDBG_MISO       | SERCOM1/PAD[3]
 * | 40         |                  |  PA16  | EDBG_MOSI       | SERCOM1/PAD[0]
 * | 41         |                  |  PA18  | EDBG_SS         | SERCOM1/PAD[2]
 * | 42         |                  |  PA17  | EDBG_SCK        | SERCOM1/PAD[1]
 * +------------+------------------+--------+-----------------+------------------------------
 * | 43         |                  |  PA13  | EDBG_GPIO0      | EIC/EXTINT[13] *TCC2/WO[1] TCC0/WO[7]
 * | 44         |                  |  PA21  | EDBG_GPIO1      | Pin 7
 * | 45         |                  |  PA06  | EDBG_GPIO2      | Pin 8
 * | 46         |                  |  PA07  | EDBG_GPIO3      | Pin 9
 * +------------+------------------+--------+-----------------+------------------------------
 * |            |32.768KHz Crystal |        |                 |
 * +------------+------------------+--------+-----------------+------------------------------
 * |            |                  |  PA00  | XIN32           | EXTINT[0] SERCOM1/PAD[0] TCC2/WO[0]
 * |            |                  |  PA01  | XOUT32          | EXTINT[1] SERCOM1/PAD[1] TCC2/WO[1]
 * +------------+------------------+--------+-----------------+------------------------------
 */
```



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
