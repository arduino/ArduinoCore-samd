# MattairTech Xeno (ATSAMx21Jxxx)

```
============================= MattairTech Xeno (ATSAMx21Jxxx) ===========================
Other  COM   PWM  Analog  INT  Arduino*           Arduino*  INT  Analog  PWM   COM  Other
=========================================================================================
                                   -------------------
         Board Variant:           | (no external pin) |
 B=Basic, S=Standard, A=Advanced  |            |-- B3 | 49   I   O       VBAT(L)/SDCD(+B)
    M=Memory device installed     |            |-- B5 | 48   I                   INT1(+B)
                                  |            |-- B4 | 47                       3SEN(+B)
                                  |                   |
XBDS(B)                O        0 | B0            RST |                          BOOT(+B)
MECS(+M)               O        1 | B1            A31 | 31            RX1  SWDIO/XBDO(+B)
DAC0                   O   I    2 | A2            A30 | 30                     SWDCLK(+B)
REFA(B)                O        3 | A3        A28(D/C)| 28                      SHCS(D/C)
3SVO(S)/REFB           O        4 | A4            A27 | 27   I                       INT2
DAC1(L)                O        5 | A5        X34 (B2)| 34       O   TC60~   LED(+B)/XBRT
CMVO(S)                O        6 | B6        X33(B16)| 33   I               INT0(+B)/BTN
ASEN(+A)               O        7 | B7        X32(B17)| 32   I       TC61~       MOPS(+S)
          TX3          O   I    8 | B8            A23 | 23           TC41~
          RX3          O   I    9 | B9            A22 | 22           TC40~
VHDV(A)   MOSI1        O   I   10 | A10           A21 | 21           TC71~
VBDV(+A)  SCK1         O   I   11 | A11           A20 | 20   I       TC70~
XBCT(B) SDA1/MISO1 TCC20~  I   12 | A12           A19 | 19           TC31~        CMRI(S)
        SCL1/SS1  TCC21~   I   13 | A13           A18 | 18           TC30~  TX1  XBDI(+B)
HSEN(A)         TC50~      I   14 | B14           A17 | 17                  SCL   SCL(+B)
BKFS(+A)        TC51~          15 | B15           A16 | 16                  SDA   SDA(+B)
                                  | Vaux         3.3V |
USB D- (D/L)+B, CAN TX (C)        | A24   _____  VccL |   ! VccL is 3.3V by default.
USB D+ (D/L)+B, CAN RX (C)        | A25  |     | VccH |     DO NOT exceed 3.6V on VccL or
                                  | Gnd  | USB |  Gnd |     on any IO pin with the D21 or
       Chip Variant:               -------------------      L21 installed. 5V is allowed
    D=D21, L=L21, C=C21                                     ONLY with the C21 installed.
                                  1-------------------      By default, VccH is 5V.
MISO(+B)                 43 (S43) | B30          Vcon |
SCK(+B)                  44 (S44) | B23    SPI    B22 | 45 (S45)                 MOSI(+B)
SHCS(D/C), SDCS(+B)  28, 46 (S46) | A28(B31)      Gnd |
                                   -------------------

                                  1-------------------
LVL_0(+S) TX2    O   I   35 (L35) | A6             A7 | 36 (L36)  I  O    RX2   LVL_1(+S)
LVL_2(+S) TCC12~ O  NMI  37 (L37) | A8    LEVEL    A9 | 38 (L38)     O  TCC13~  LVL_3(+S)
                                  | VccH  SHIFT  VccH |
                                  | Gnd           Gnd |
MOTOR_B1(+S)    TCC04~   39 (M39) | B10           B11 | 40 (M40)   TCC05~    MOTOR_B2(+S)
MOTOR_A1(+S)    TCC06~   41 (M41) | B12   MOTOR   B13 | 42 (M42)   TCC07~    MOTOR_A2(+S)
                                  | Vmotor        Gnd |
                                   -------------------

* Most pins can be used for more than one function. The same port pin number printed on
  the board is also used in Arduino (without the 'A') for all of the Arduino functions.

* Different variants have different hardware installed onboard. The alternate functions
  column shows for which board variant(s) the associated hardware is installed: B=Basic,
  S=Standard, A=Advanced, and M=Memory device installed. The Advanced variant has all of
  the hardware from the Standard, and the Standard has all of hardware from the Basic.

+ This function is enabled by default depending on the variant indicated by the letter.
  Thus, the associated header pin cannot be used. In most cases (except most +A pins),
  solder jumpers can be used to enable or disable the alternate onboard function.

~ TC3, TC4, TC5, TC6, TC7 on the D21 are instead TC4, TC0, TC1, TC2, TC3 on the L21/C21.

Silkscreen Legend:
  Top: A circle around pin is analog function, '~' is timer, small 'I' is interrupt
  Bottom: A box around pin means 'Other' function enabled by default depending on variant
```


# Pins descriptions for the MattairTech Xeno (ATSAMD21J/ATSAML21J/ATSAMC21J)
```
=====================================================================================================================================================
Arduino	| Port	| Silk	| IC	| Peripheral functions			| Board functions (Silk)Variant	| Notes
--------|-------|-------|-------|---------------------------------------|-------------------------------|--------------------------------------------
0	| B0	| B0	| 61	| ADC					| XBee_DTR_SLEEP (XBDS)B	| 
1	| B1	| B1	| 62	| ADC					| MEM_CS (MECS)+M		| 
2	| A2	| A2	| 3	| ADC / DAC0 / EXTINT:2			| 				| 
3	| A3	| A3	| 4	| ADC / VREFA				| REFA (REFA)B			| 
4	| A4	| A4	| 13	| ADC / VREFB / AC			| 3.3Vsw_VOUT (3SVO)S		| 
5	| A5	| A5	| 14	| ADC / DAC1 (L) / AC			| 				| 
6	| B6	| B6	| 9	| ADC					| CURR_MON_VOUT (CMVO)A		| 
7	| B7	| B7	| 10	| ADC					| AUX_SW_EN (ASEN)+A		| 
8	| B8	| B8	| 11	| ADC / SERCOM4:0 / EXTINT:8		| TX3				| no I2C
9	| B9	| B9	| 12	| ADC / SERCOM4:1 / EXTINT:9		| RX3				| no I2C
10	| A10	| A10	| 19	| ADC / SERCOM2:2 / EXTINT:10		| VccH_DIVIDER (VHDV)A / MOSI1	| ADC on VDDIO
11	| A11	| A11	| 20	| ADC / SERCOM2:3 / EXTINT:11		| Vbus_DIVIDER (VBDV)+A / SCK1	| ADC on VDDIO
12	| A12	| A12	| 29	| SERCOM2:0 / TCC20 / EXTINT:12		| XBee_CTS (XBCT)B / SDA1	| also MISO1
13	| A13	| A13	| 30	| SERCOM2:1 / TCC21 / EXTINT:13		| SCL1				| also SS1
14	| B14	| B14	| 27	| SERCOM4:2 / TC5(TC1):0 /  EXTINT:13	| HOST_SW_EN / USB_ID (HSEN)A	| HOST_ENABLE (ID pin controls by default)
15	| B15	| B15	| 28	| SERCOM4:3 / TC5(TC1):1		| BUCK_FSW (BKFS)+A		| 
16	| A16	| A16	| 35	| SERCOM3:0				| I2C SDA (SDA)+B		| 4.7Kohm pullup to VccL
17	| A17	| A17	| 36	| SERCOM3:1				| I2C SCL (SCL)+B		| 4.7Kohm pullup to VccL
18	| A18	| A18	| 37	| SERCOM1:2 / TC3(TC4):0		| TX1 / XBee_Din (XBDI)+B	| 
19	| A19	| A19	| 38	| TC3(TC4):1				| CURR_MON_READINT (CMRI)S	| 
20	| A20	| A20	| 41	| SERCOM3:2 / TC7(TC3):0 / EXTINT:4	| 				| 
21	| A21	| A21	| 42	| SERCOM3:3 / TC7(TC3):0		| 				| 
22	| A22	| A22	| 43	| TC4(TC0):0				| 				| 
23	| A23	| A23	| 44	| TC4(TC0):1				| 				| 
24	| A24	| A24	| 45	| USB D- (D21/L21) / CAN TX (C21)	| USB Micro D-	(D-)+B		| 
25	| A25	| A25	| 46	| USB D+ (D21/L21) / CAN RX (C21)	| USB Micro D+	(D+)+B		| 
26	| ---	| ---	| --	| NOT_A_PIN				| NOT_A_PIN			| 
27	| A27	| A27	| 51	| EXTINT:15				| INT2 (INT2)			| I2C_INT / XBee_ATTN
28	| A28	| A28	| 53	| A28 exists only on D21 and C21	| SPI_HEADER_CS (SHCS)+B	| This is VDDCORE on the L21
29	| ---	| ---	| --	| NOT_A_PIN				| NOT_A_PIN			| 
30	| A30	| A30	| 57	| SWCLK					| SWCLK+B			| leave floating during reset
31	| A31	| A31	| 58	| SWDIO / SERCOM1:3			| RX1 / XBee_Dout (XBDO)+B	| SWDIO enabled only when using external tool
32	| B17	| X32	| 40	| EXTINT:1 / TC6(TC2):1			| MOTOR_PS (MOPS)+S		| 
33	| B16	| X33	| 39	| EXTINT:0				| INT0 (INT0)+B			| Button / XBee_BTN
34	| B2	| X34	| 63	| ADC / TC6(TC2):0			| LED (LED)+B / XBee_RTS (XBRT)	| RTS and ASSOC LED can be used at same time
35	| A6	| L35	| 15	| ADC / SERCOM0:2 / EXTINT:6		| LVL_0 (L35)+S / TX2   	| 
36	| A7	| L36	| 16	| ADC / SERCOM0:3 / EXTINT:7		| LVL_1 (L36)+S / RX2	        | 
37	| A8	| L37	| 17	| ADC / SERCOM0:0 / TCC1:2 / EXTINT:NMI	| LVL_2 (L37)+S		        | ADC on VDDIO
38	| A9	| L38	| 18	| ADC / SERCOM0:1 / TCC1:3		| LVL_3 (L38)+S		        | ADC on VDDIO
39	| B10	| M39	| 23	| TCC0:4				| MOTOR_B1 (M39)+S		| Also maps to TCC0:0
40	| B11	| M40	| 24	| TCC0:5				| MOTOR_B2 (M40)+S		| Also maps to TCC0:1
41	| B12	| M41	| 25	| TCC0:6				| MOTOR_A1 (M41)+S		| Also maps to TCC0:2
42	| B13	| M42	| 26	| TCC0:7				| MOTOR_A2 (M42)+S		| Also maps to TCC0:3
43	| B30	| S43	| 59	| SERCOM5:0				| SPI MISO (S43)+B		| 
44	| B23	| S44	| 50	| SERCOM5:3				| SPI SCK (S44)+B		| 
45	| B22	| S45	| 49	| SERCOM5:2				| SPI MOSI (S45)+B		| 
46	| B31	| S46	| 60	| SERCOM5:1				| SS / SD_CS (S46)+B		| or SPI_HEADER_CS (SHCS)
47	| B4	| --	| 5	| 					| 3.3Vsw_EN (3SEN)+B		| 
48	| B5	| --	| 6	| EXTINT:5				| INT1 (INT1)+B			| MOTOR_ERROR / IMU_INT / AUX_SW_ERROR
49	| B3	| --	| 64	| ADC / VBAT(L) / EXTINT:3		| VBAT / SD_CardDetect (SDCD)+B	| 
--	| RESETN| RST	| 52	| RESET					| RESET	(BOOT)+B		| Double-tap reset for bootloader entry
=====================================================================================================================================================

* Most pins can be used for more than one function. The same port pin number printed on the board is also used in Arduino (without the 'A')
  for all of the supported functions (ie: digitalRead(), analogRead(), analogWrite(), attachInterrupt(), etc.).
* Different variants have different hardware installed onboard. The alternate functions column shows for which board variant(s) the associated
  hardware is installed: B=Basic, S=Standard, A=Advanced, and M=Memory device installed. The Advanced variant has all of the hardware that the
  Standard and Basic have installed, and the Standard variant has all of the Basic hardware.
+ This function is enabled by default depending on the variant indicated by the letter. Thus, the associated header pin cannot be used.
  In most cases (except most +A pins), solder jumpers can be used to enable or disable the alternate onboard function.
* TC timers shown are for the D21 (the TC timers in parentheses are for the L21 and C21).
* There is no header pin available for: A0 and A1 (32.768KHz crystal), A14 aand A15 (16MHz crystal), and B3, B4, & B5 (used by onboard hardware).
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
  * L21: Two 12-bit 1Msps analog outputs are available on pins 2 and 5.

* **PWM**
  * Up to 18 pins can be configured as PWM outputs.
  * Each pin provides 8 bits of resolution (256 values) by default.
  * 12-bit resolution supported by using the analogWriteResolution() function.

* **External Interrupts**
  * Up to 16 pins can be configured with external interrupts.

* **SERCOM**
  * 6 SERCOM are available.
  * Up to 3 UART instances. More in a future release.
  * Up to 2 SPI instances.
  * Up to 2 WIRE (I2C) instances.
  * The WIRE pullup resistors are enabled by default.



## PinDescription table format

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
from the MT-D11 variant. The MT-D21J combines both methods, using the actual port pin 
designators from both PORTA and PORTB for arduino numbers 0-31 (ie: B1=1, A2=2), then
using arduino numbering only above 31. For 0-31 only one pin from PORTA or PORTB can be
used, leaving the other pin for some number above 31.

**See [WVariant.h](https://github.com/mattairtech/ArduinoCore-samd/tree/master/cores/arduino/WVariant.h) for the definitions used in the table.**

### Port
This is the port (ie: PORTA).

### Pin
This is the pin (bit) within the port. Valid values are 0-31.

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
select between the two peripherals possible with each of the SERCOM and TIMER functions.
PeripheralAttribute is now used for this.

### PeripheralAttribute
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
