# MattairTech MT-D21E (rev B) (ATSAMx21Exxx)

```
========================== MattairTech MT-D21E rev B (ATSAMx21Exxx) =====================
Other  COM    PWM   Analog  INT  Arduino*             Arduino*  INT   PWM     COM   Other
=========================================================================================
                                      -------------------
XI32(+)                              | A0            RST |                        BOOT(+)
XO32(+)                              | A1            Gnd |
DAC                   *            2 | A2           Vbat |
REFA                  *            3 | A3            A31 | 31    *                IO/B(+)
REFB                  *      *     4 | A4            A30 | 30    *                 CLK(+)
DAC1(L)               *      *     5 | A5            NC  |
LED(+)       TCC10    *            6 | A6       A28 (D/C)| 28    *
VM           TCC11    *            7 | A7            A27 | 27    *               A/CS(+M)
             TCC00    *     NMI    8 | A8            A23 | 23    * TC41/TC01~ SS
             TCC01    *      *     9 | A9            A22 | 22    * TC40/TC00~ MISO(+M)
       TX1   TCC02    *           10 | A10           A19 | 19    *            SCK(+M)
       RX1   TCC03    *           11 | A11           A18 | 18    *            MOSI(+M)
       TX2 TC30/TC40~        *    14 | A14           A17 | 17    *   TCC21    SCL(+)
       RX2 TC31/TC41~             15 | A15           A16 | 16    *   TCC20    SDA(+)
                                     | NC            NC  |
     M=Memory device installed       | NC            NC  | ! Vcc is 3.3V by default.
                                     | Vbus          3.3V|   DO NOT exceed 3.6V on Vcc or
USB D- (D/L)(+), CAN TX (C)  TC50 24 | A24   _____   Vcc |   any IO pin with the D21 or
USB D+ (D/L)(+), CAN RX (C)  TC51 25 | A25  |     |  Vin |   L21 installed. 5V is allowed
                                     | Gnd  | USB |  Gnd |   ONLY with the C21 installed.
           Chip Variant:              -------------------
        D=D21, L=L21, C=C21

* Most pins can be used for more than one function. The same port pin number printed on
  the board is also used in Arduino (without the 'A') for all of the supported functions
  (ie: digitalRead(), analogRead(), analogWrite(), attachInterrupt(), etc.).

+ This alternate function is enabled by default (+M functions enabled only when a memory
  device is installed). Thus, the associated header pin cannot be used. Solder jumpers
  can be used to enable or disable the alternate onboard function.

~ When two timers are shown, the second is for L21/C21. TC5 is TC1 on the L21/C21.

Silkscreen Legend:
  Top: A circled pin means analog function and '*' means alternate function (see + above)
  Bottom: A circled pin means analog function
```


# Pins descriptions for the MattairTech MT-D21E (rev B)
============================================================================================================================================
Arduino	| Silk	| Port	| Alternate Function	| Comments (! means not used with this peripheral assignment)
--------|-------|-------|-----------------------|-------------------------------------------------------------------------------------------
0	| A0	| PA00	| Xin32			| Xin32
1	| A1	| PA01	| Xout32		| Xout32
2	| A2	| PA02	| DAC0			| !EIC/EXTINT[2] ADC/AIN[0] PTC/Y[0] DAC/VOUT
3	| A3	| PA03	| REFA			| !EIC/EXTINT[3] REF/ADC/VREFA REF/DAC/VREFA ADC/AIN[1] PTC/Y[1]
4	| A4	| PA04	| REFB			| EIC/EXTINT[4] REF/ADC/VREFB ADC/AIN[4] AC/AIN[0] PTC/Y[2] !SERCOM0/PAD[0] !TCC0/WO[0]
5	| A5	| PA05	| DAC1(L21)		| EIC/EXTINT[5] ADC/AIN[5] AC/AIN[1] PTC/Y[3] !SERCOM0/PAD[1] !TCC0/WO[1] DAC1(L21)
6	| A6	| PA06	| LED			| !EIC/EXTINT[6] ADC/AIN[6] AC/AIN[2] PTC/Y[4] !SERCOM0/PAD[2] !TCC1/WO[0] LED
7	| A7	| PA07	| Voltage Measurement	| !EIC/EXTINT[7] ADC/AIN[7] AC/AIN[3] PTC/Y[5] !SERCOM0/PAD[3] !TCC1/WO[1]
8	| A8	| PA08	|			| EIC/NMI ADC/AIN[16] PTC/X[0] !SERCOM0/PAD[0] !SERCOM2/PAD[0] TCC0/WO[0] !TCC1/WO[2]
9	| A9	| PA09	|			| EIC/EXTINT[9] ADC/AIN[17] PTC/X[1] !SERCOM0/PAD[1] !SERCOM2/PAD[1] TCC0/WO[1] !TCC1/WO[3]
10	| A10	| PA10	| TX1			| !EIC/EXTINT[10] ADC/AIN[18] PTC/X[2] SERCOM0/PAD[2] !SERCOM2/PAD[2] !TCC1/WO[0] TCC0/WO[2]
11	| A11	| PA11	| RX1			| !EIC/EXTINT[11] ADC/AIN[19] PTC/X[3] SERCOM0/PAD[3] !SERCOM2/PAD[3] !TCC1/WO[1] TCC0/WO[3]
12	| ---	| ----	| NOT A PIN		| NOT A PIN
13	| ---	| ----	| NOT A PIN		| NOT A PIN
14	| A14	| PA14	| Xin, TX2		| EIC/EXTINT[14] SERCOM2/PAD[2] TC3/WO[0] !TCC0/WO[4] Xin, HOST_ENABLE
15	| A15	| PA15	| Xout, RX2		| !EIC/EXTINT[15] SERCOM2/PAD[3] TC3/WO[1] !TCC0/WO[5] Xout
16	| A16	| PA16	| I2C/SDA w/pullup	| EIC/EXTINT[0] PTC/X[4] SERCOM1/PAD[0] !SERCOM3/PAD[0] TCC2/WO[0] !TCC0/WO[6]
17	| A17	| PA17	| I2C/SCL w/pullup	| EIC/EXTINT[1] PTC/X[5] SERCOM1/PAD[1] !SERCOM3/PAD[1] TCC2/WO[1] !TCC0/WO[7]
18	| A18	| PA18	| SPI MOSI		| EIC/EXTINT[2] PTC/X[6] !SERCOM1/PAD[2] SERCOM3/PAD[2] !TC3/WO[0] !TCC0/WO[2]
19	| A19	| PA19	| SPI SCK		| EIC/EXTINT[3] PTC/X[7] !SERCOM1/PAD[3] SERCOM3/PAD[3] !TC3/WO[1] !TCC0/WO[3]
20	| ---	| ----	| NOT A PIN		| NOT A PIN
21	| ---	| ----	| NOT A PIN		| NOT A PIN
22	| A22	| PA22	| SPI MISO		| EIC/EXTINT[6] PTC/X[10] SERCOM3/PAD[0] TC4/WO[0] !TCC0/WO[4]
23	| A23	| PA23	| SPI SS		| EIC/EXTINT[7] PTC/X[11] SERCOM3/PAD[1] TC4/WO[1] !TCC0/WO[5]
24	| A24-	| PA24	| USB_NEGATIVE		| USB/DM TC5/WO[0]
25	| A25+	| PA25	| USB_POSITIVE		| USB/DP TC5/WO[1]
26	| ---	| ----	| NOT A PIN		| NOT A PIN
27	| A27	| PA27	| A/CS			| EIC/EXTINT[15] A/CS (Jumper A / memory device chip select)
28	| A28	| PA28	|			| EIC/EXTINT[8]
29	| ---	| ----	| NOT A PIN		| NOT A PIN
30	| A30	| PA30	| SWD CLK		| EIC/EXTINT[10] !SERCOM1/PAD[2] TCC1/WO[0] SWD CLK, leave floating during boot
31	| A31	| PA31	| Button B / SWD IO	| EIC/EXTINT[11] !SERCOM1/PAD[3] TCC1/WO[1] Button B SWD IO
--	| RST	| ----	|			| Reset, BOOT (double tap bootloader entry)
============================================================================================================================================

* Most pins can be used for more than one function. The same port pin number printed on the board is also used in Arduino (without the 'A')
  for all of the supported functions (ie: digitalRead(), analogRead(), analogWrite(), attachInterrupt(), etc.).

* The following Arduino pin numbers are not mapped to a physical pin: 12, 13, 20, 21, 26, and 29.

* Pins 24 and 25 are by default in use by USB (USB_NEGATIVE and USB_POSITIVE). TC5(D21) is available on these pins otherwise.
  The tone library uses TC5.

* A0 and A1 are by default connected to the 32.768KHz crystal.



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
