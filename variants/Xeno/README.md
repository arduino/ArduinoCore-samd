# MattairTech Xeno (ATSAMD51J/ATSAMD21J/ATSAML21J/ATSAMC21J)

```
============== MattairTech Xeno (ATSAMD51J/ATSAMD21J/ATSAML21J/ATSAMC21J) ===============
Alt   COM    PWM  Analog  INT  Arduino*           Arduino*  INT  Analog  PWM    COM   Alt
=========================================================================================
                                   -------------------
INT1:                             |     no       / B3 | 49   I   O      VBAT / SDCD+ (SD)
MOTOR_ERROR/IMU_INT/AUX_SW_ERROR  |  external ---  B5 | 48   I                      INT1+
                                  |     pin      \ B4 | 47                 3SEN+ (3.3Vsw)
XBDS (XBEE)            O        0 | B0            RST |                              BOOT
MECS+ (MEM)            O        1 | B1            A31 | 31  RX1  IO (PROG) / XBDO+ (XBEE)
DAC0                   O   I    2 | A2            A30 | 30                    CLK+ (PROG)
REFA*                  O        3 | A3    A28(D21/C21)| 28                     SHCS (SPI)
3SVO (3.3Vsw) / REFB   O        4 | A4            A27 | 27   I           INT2 (XBEE, I2C)
DAC1*                  O        5 | A5        X34 (B2)| 34     O  TC~  LED+ / XBRT (XBEE)
CMVO (CUR)             O        6 | B6        X33(B16)| 33   I                INT0+ / BTN
ASEN+ (HOST)           O        7 | B7        X32(B17)| 32   I     TC~      MOPS+ (MOTOR)
          TX3          O   I    8 | B8            A23 | 23         TC~
          RX3          O   I    9 | B9            A22 | 22         TC~
VHDV (VccH)   MOSI1    O   I   10 | A10           A21 | 21         TC~
VBDV+ (Vbus)  SCK1     O   I   11 | A11           A20 | 20   I     TC~
XBCT (XBEE) SDA1/MISO1 TCC~  I 12 | A12           A19 | 19         TC~         CMRI (CUR)
        SCL1/SS1   TCC~    I   13 | A13           A18 | 18         TC~  TX1  XBDI+ (XBEE)
HSEN (HOST)        TC~     I   14 | B14           A17 | 17              SCL    SCL+ (I2C)
BKFS+ (BUCK)       TC~         15 | B15           A16 | 16              SDA    SDA+ (I2C)
                                  | Vaux         3.3V |
USB D- (+) / CAN TX               | A24   _____  VccL |   ! VccL is 3.3V by default.
USB D+ (+) / CAN RX               | A25  |     | VccH |     DO NOT exceed 3.6V on VccL or
                                  | Gnd  | USB |  Gnd |     any IO pin with the D51, D21,
     USB: D51/D21/L21 only         -------------------      or L21 installed. 5V allowed
       CAN: D51/C21 only                                    ONLY with the C21 installed.
                                  1-------------------      By default, VccH is 5V.
MISO+ (SD, MEM, SPI)     43 (S43) | B30          Vcon |
SCK+ (SD, MEM, SPI)      44 (S44) | B23    SPI    B22 | 45 (S45)     MOSI+ (SD, MEM, SPI)
SHCS(SPI) / SDCS+(SD) 28/46 (S46) | A28(B31)      Gnd |
                                   -------------------

                                  1-------------------
L0+ (LVL) TX2    O   I   35 (L35) | A6             A7 | 36 (L36)  I  O    RX2   L1+ (LVL)
L2+ (LVL) TCC~   O  NMI  37 (L37) | A8    LEVEL    A9 | 38 (L38)     O    TCC~  L3+ (LVL)
                                  | VccH  SHIFT  VccH |
                                  | Gnd           Gnd |
B1+ (MOTOR)    TCC~      39 (M39) | B10           B11 | 40 (M40)     TCC~     B2+ (MOTOR)
A1+ (MOTOR)    TCC~      41 (M41) | B12   MOTOR   B13 | 42 (M42)     TCC~     A2+ (MOTOR)
                                  | Vmotor        Gnd |
                                   -------------------

* Most pins can be used for more than one function. The same port pin number printed on
  the board is also used in Arduino (without the 'A') for all of the Arduino functions.
  DAC1 is present only on the D51 and L21. With the D51, REFA should be tied to VccL.

+ This alternate function is enabled by default if installed. Thus, the associated header
  pin cannot be used unless a solder jumper is available to disable the function.

~ D21/L21/C21: 3 TCC (4,2,2 ch.), 5 TC (2 ch.). D51: 5 TCC (6,4,3,2,2 ch.), 6 TC (2 ch.).
  The D51 adds timers to pins A10, A11, A30, A31, B30, B31, A6, A7, A16, A17, B8, and B9,
  however, the timer on pin B17 is not present.

I For the D51, the interrupt on pin A27 is moved to pin B15.

* The D51 SERCOM configuration is different for UART and SPI. Third UART not available.
  TX1=8, RX1=9, TX2=4, RX2=5, MOSI=23, MISO=21, SCK=22, MOSI1=38, MISO1=11, SCK1=37.

Silkscreen Legend:
  Top: A circle around pin is analog function, '~' is timer, small 'I' is interrupt
  Bottom: A box around pin means 'Other' function enabled by default depending on variant
```


## Board Configuration Notes

* **Crystals**
  * Either the 32.768KHz crystal or the 16MHz crystal can be used. These pins do not route to headers.
  * The bootloader does not use an external crystal by default. Double-tap the reset button to enter manually.

* **LED (LED_BUILTIN)**
  * Bring the pin HIGH to turn the LED on.
  * The LED is enabled (solder jumper) by default.

* **Button (BUTTON_BUILTIN)**
  * Button is connected to the Reset pin by default, but can be connected to pin 33 (B16) via solder jumper J2.
  * Pressing the button will bring the pin LOW. The pullup must be enabled first.
  * A debouncing capacitor is connected, so delay reading the pin at least 10ms after turning on the pullup.

* **GPIO** 
  * All pins (including analog) support INPUT, OUTPUT, INPUT_PULLUP, and INPUT_PULLDOWN.
  * When PER_ATTR_DRIVE_STRONG is set for the pin (enabled by default), each pin can source or sink a maximum of:
    * **D21:** 7mA high, 10mA low
    * **L21:** 5mA high, 6mA low (8 high drive pins: 10mA high, 12mA low)
    * **C21:** 6mA high, 10mA low (2 high drive pins (A10, A11): 12mA high, 20mA low)
    * **D51:** 8mA high, 8mA low
  * Internal pull-up and pull-down resistors of 20-60 Kohms (40Kohm typ., disconnected by default).

* **Analog Inputs**
  * Up to 18 pins can be configured as ADC analog inputs.
  * Each pin measures from ground to 3.3 volts by default.
  * Each pin provides 10 bits of resolution (1024 values) by default.
  * 12-bit resolution supported by using the analogReadResolution() function.
  * The upper end of the measurement range can be changed using the analogReference() function.
  * A reference voltage can be connected to REFA. In this case, the capacitors should be enabled via solder jumper J33.

* **DAC**
  * D21/C21: One 10-bit 350Ksps analog output is available on pin 2.
  * L21/D51: Two 12-bit 1Msps analog outputs are available on pins 2 and 5.

* **PWM**
  * Up to 18 pins can be configured as PWM outputs (29 for D51).
  * Each pin provides 8 bits of resolution (256 values) by default.
  * 12-bit resolution supported by using the analogWriteResolution() function.

* **External Interrupts**
  * Up to 16 pins can be configured with external interrupts.

* **SERCOM**
  * 6 SERCOM are available.
  * Up to 3 UART instances (two for D51). More in a future release.
  * Up to 2 SPI instances.
  * Up to 2 WIRE (I2C) instances.
  * The WIRE pullup resistors are enabled by default.

* **Special Notes for D51**
  * Due to errata with the D51 DAC, VccL must be routed to the REFA pin with an external jumper wire.
  * In order to use the SPI bus (Micro SD, optional memory device, SPI mode of Xbee radio):
    * Needed because SERCOM5 on the D51 must use IOSET2.
    * Solder a jumper wire between A21 and B30 (S43, pin 1 of the SPI header, MISO).
    * Solder a jumper wire between A23 and B22 (S45, pin 4 of the SPI header, MOSI).
    * Solder a jumper wire between A22 and B23 (S44, pin 3 of the SPI header, SCK).
    * B22, B23, and B30 cannot be used for other purposes (leave floating)

  * In order to use the Xbee radio UART interface (use SPI mode to avoid this fix):
    * Needed because D51 UART transmit data pinout options (TXPO) do not include TX on pad 2.
    * Solder a jumper wire between B8 (SERCOM4, TX pad 0) and A18 (XBDI). Solder a jumper wire between B9 (SERCOM4, RX pad 1) and A31 (XBDO).
    * A18 and A31 cannot be used for other purposes (leave floating)



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
