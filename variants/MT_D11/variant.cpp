/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

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
  */

/*
 * Modified 8 August 2015 by Justin Mattair
 *   for MattairTech MT-D11 boards (www.mattairtech.com)
 */

/*
============================= MattairTech MT-D11 (ATsamd11D14AM) ========================
Other   INT    PWM   Digital  Analog                      Digital  PWM   INT    Other
=========================================================================================
                                       -------------------
DAC                    2    2 (ADC0)  | A2   | USB |  Gnd |
REF                    3    3 (ADC1)  | A3   |     |  Vcc |
VDIV   INT4   TCC0[0]  4    4 (ADC2)  | A4    -----   A31 |  31  TC2[1]  INT3  RX / SWDIO
       INT5   TCC0[1]  5    5 (ADC3)  | A5            A30 |  30  TC2[0]       TX / SWDCLK  
              TCC0[2]  6    6 (ADC4)  | A6            A27 |  27          INT7
              TCC0[3]  7    7 (ADC5)  | A7            A23 |  23                 I2C/SCL
SPI MOSI  INT2         10   10 (ADC8) | A10           A22 |  22          INT6   I2C/SDA
SPI SCK                11   11 (ADC9) | A11           A17 |  17  TC1[1]
SPI MISO  INTNMI       14   14 (ADC6) | A14           A16 |  16  TC1[0]  INT0   LED
Button    INT1         15   15 (ADC7) | A15           RST |                     Reset
                                       -------------------
Most pins have multiple configurations available (even analog pins). For example, pin
4 can be an analog input, a PWM output, Digital I/O, or interrupt input. These
always reference the pin number printed on the board but without the 'A' (with the
usable pins starting at 2). Tone available on TC2.
DO NOT connect voltages higher than 3.3V!
*/


/*   The PinDescription table describes how each of the pins can be used by the Arduino
 *   core. Each pin can have multiple functions (ie: ADC input, digital output, PWM,
 *   communications, etc.), and the PinDescription table configures which functions can
 *   be used for each pin. This table is mainly accessed by the pinPeripheral function in
 *   wiring_private.c, which is used to attach a pin to a particular peripheral function.
 *   The communications drivers (ie: SPI, I2C, and UART), analogRead(), analogWrite(),
 *   analogReference(), attachInterrupt(), and pinMode() all call pinPeripheral() to
 *   verify that the pin can perform the function requested, and to configure the pin for
 *   that function. Most of the contents of pinMode() are now in pinPeripheral().
 * 
 *   There are two ways that pins can be mapped. The first is to map pins contiguously
 *   (no PIO_NOT_A_PIN entries) in the table. This results in the least amount of space
 *   used by the table. A second method, used by default by the MT-D21E and MT-D11, maps
 *   Arduino pin numbers to the actual port pin number (ie: Arduino pin 28 = Port A28).
 *   This only works when there is one port. Because not all port pins are available,
 *   PIO_NOT_A_PIN entries must be added for these pins and more FLASH space is consumed.
 *   For an example of both types, see variant.cpp from the MT-D11 variant.
 * 
 *   Explanation of PinDescription table:
 * 
 *   Port                  This is the port (ie: PORTA).
 *   Pin                   This is the pin (bit) within the port. Valid values are 0-31.
 *   PinType               This indicates what peripheral function the pin can be
 *                         attached to. In most cases, this is PIO_MULTI, which means
 *                         that the pin can be anything listed in the PinAttribute field.
 *                         It can also be set to a specific peripheral. In this case, any
 *                         attempt to configure the pin (using pinPeripheral or pinMode)
 *                         as anything else will fail (and pinPeripheral will return -1).
 *                         This can be used to prevent accidental re-configuration of a
 *                         pin that is configured for only one function (ie: USB D- and
 *                         D+ pins). If a pin is not used or does not exist,
 *                         PIO_NOT_A_PIN must be entered in this field. See WVariant.h
 *                         for valid entries. These entries are also used as a parameter
 *                         to pinPeripheral() with the exception of PIO_NOT_A_PIN and
 *                         PIO_MULTI. The pinMode function now calls pinPeripheral() with
 *                         the desired mode. Note that this field is not used to select
 *                         between the two peripherals possible with each of the SERCOM
 *                         and TIMER functions. PeripheralAttribute is now used for this.
 *   PeripheralAttribute   This is an 8-bit bitfield used for various peripheral
 *                         configuration. It is primarily used to select between the two
 *                         peripherals possible with each of the SERCOM and TIMER
 *                         functions. TIMER pins are individual, while SERCOM uses a
 *                         group of two to four pins. This group of pins can span both
 *                         peripherals. For example, pin 19 (SPI1 SCK) on the MT-D21E
 *                         uses PER_ATTR_SERCOM_ALT while pin 22 (SPI1 MISO) uses
 *                         PER_ATTR_SERCOM_STD. Both TIMER and SERCOM can exist for each
 *                         pin. This bitfield is also used to set the pin drive strength.
 *                         In the future, other attributes (like input buffer
 *                         configuration) may be added. See WVariant.h for valid entries.
 *   PinAttribute          This is a 32-bit bitfield used to list all of the valid
 *                         peripheral functions that a pin can attach to. This includes
 *                         GPIO functions like PIN_ATTR_OUTPUT. Certain attributes are
 *                         shorthand for a combination of other attributes.
 *                         PIN_ATTR_DIGITAL includes all of the GPIO functions, while
 *                         PIN_ATTR_TIMER includes both PIN_ATTR_TIMER_PWM and
 *                         PIN_ATTR_TIMER_CAPTURE (capture is not used yet).
 *                         PIN_ATTR_ANALOG is an alias to PIN_ATTR_ANALOG_ADC. There is
 *                         only one DAC channel, so PIN_ATTR_DAC appears only once. This
 *                         bitfield is useful for limiting a pin to only input related
 *                         functions or output functions. This allows a pin to have a
 *                         more flexible configuration, while restricting the direction
 *                         (ie: to avoid contention). See WVariant.h for valid entries.
 *   TCChannel             This is the TC(C) channel (if any) assigned to the pin. Some
 *                         TC channels are available on multiple pins (ie: TCC0/WO[0] is
 *                         available on pin A4 or pin A8 on the MT-D21E). In general,
 *                         only one pin should be configured (in the pinDescription
 *                         table) per TC channel. See WVariant.h for valid entries.
 *                         The tone library uses TC5 (MT-D21E) or TC2 (MT-D11).
 *   ADCChannelNumber      This is the ADC channel (if any) assigned to the pin. See
 *                         WVariant.h for valid entries.
 *   ExtInt                This is the interrupt (if any) assigned to the pin. Some
 *                         interrupt numbers are available on multiple pins (ie:
 *                         EIC/EXTINT[2] is available on pin A2 or pin A18 on the
 *                         MT-D21E). In general, only one pin should be configured (in
 *                         the pinDescription table) per interrupt number. Thus, if an
 *                         interrupt was needed on pin 2, EXTERNAL_INT_2 can be moved
 *                         from pin 18. See WVariant.h for valid entries.
 */


/*  Pins descriptions for the MattairTech MT-D11
 * 
 * | PCB Pin | Arduino Pin Number |    Silkscreen    |  PIN   |   Alt. Function   | Comments (! means unavailable with this variant)
 * +---------+--------------------+------------------+--------+-------------------+-------------------------------------------------
 * | 0       | 2                  | A2               |  PA02  | DAC               | !EIC/EXTINT[2] ADC/AIN[0] PTC/Y[0] DAC/VOUT
 * | 1       | 3                  | A3               |  PA03  | REFA              | !EIC/EXTINT[3] REF/ADC/VREFA REF/DAC/VREFA ADC/AIN[1] PTC/Y[1]
 * | 2       | 4                  | A4               |  PA04  | REFB / VM         | EIC/EXTINT[4] REF/ADC/VREFB ADC/AIN[2] AC/AIN[0] PTC/Y[2] !SERCOM0/PAD[2] !SERCOM0/PAD[0] !TC1/WO[0] TCC0/WO[0]
 * | 3       | 5                  | A5               |  PA05  |                   | EIC/EXTINT[5] ADC/AIN[3] AC/AIN[1] PTC/Y[3] !SERCOM0/PAD[3] !SERCOM0/PAD[1] !TC1/WO[1] TCC0/WO[1]
 * | 4       | 6                  | A6               |  PA06  |                   | !EIC/EXTINT[6] ADC/AIN[4] AC/AIN[2] PTC/Y[4] !SERCOM0/PAD[0] !SERCOM0/PAD[2] !TC2/WO[0] TCC0/WO[2]
 * | 5       | 7                  | A7               |  PA07  |                   | !EIC/EXTINT[7] ADC/AIN[5] AC/AIN[3] PTC/Y[5] !SERCOM0/PAD[1] !SERCOM0/PAD[3] !TC2/WO[1] TCC0/WO[3]
 * | --      | --                 | --               |  PA08  | Xin32 / Xin       | Xin32
 * | --      | --                 | --               |  PA09  | Xout32 / Xout     | Xout32
 * | 6       | 10                 | A10              |  PA10  | SPI MOSI          | EIC/EXTINT[2] ADC/AIN[8] PTC/X[2] PTC/Y[8] SERCOM0/PAD[2] !SERCOM2/PAD[2] !TC2/WO[0] !TCC0/WO[2]
 * | 7       | 11                 | A11              |  PA11  | SPI SCK           | !EIC/EXTINT[3] ADC/AIN[9] PTC/X[3] PTC/Y[9] SERCOM0/PAD[3] !SERCOM2/PAD[3] !TC2/WO[1] !TCC0/WO[3]
 * | 8       | 14                 | A14              |  PA14  | SPI MISO          | EIC/NMI ADC/AIN[6] PTC/X[0] PTC/Y[6] SERCOM0/PAD[0] !SERCOM2/PAD[0] !TC1/WO[0] !TCC0/WO[0]
 * | 9       | 15                 | A15              |  PA15  | Button / SPI SS   | EIC/EXTINT[1] ADC/AIN[7] PTC/X[1] PTC/Y[7] SERCOM0/PAD[1] !SERCOM2/PAD[1] !TC1/WO[1] !TCC0/WO[1] Button
 * | 10      | --                 | RST              |  PA28  | Reset             | Reset
 * | 11      | 16                 | A16              |  PA16  | LED               | EIC/EXTINT[0] PTC/X[4] PTC/Y[10] !SERCOM1/PAD[2] !SERCOM2/PAD[2] TC1/WO[0] !TCC0/WO[6] LED
 * | 12      | 17                 | A17              |  PA17  | HOST_ENABLE       | !EIC/EXTINT[1] PTC/X[5] PTC/Y[11] !SERCOM1/PAD[3] !SERCOM2/PAD[3] TC1/WO[1] !TCC0/WO[7] HOST_ENABLE
 * | 13      | 22                 | A22              |  PA22  | I2C/SDA w/pullup  | EIC/EXTINT[6] PTC/X[6] PTC/Y[12] !SERCOM1/PAD[0] SERCOM2/PAD[0] !TC1/WO[0] !TCC0/WO[4]
 * | 14      | 23                 | A23              |  PA23  | I2C/SCL w/pullup  | !EIC/EXTINT[7] PTC/X[7] PTC/Y[13] !SERCOM1/PAD[1] SERCOM2/PAD[0] !TC1/WO[1] !TCC0/WO[5]
 * | --      | --                 | ---              |  PA24  | USB_NEGATIVE      | USB/DM
 * | --      | --                 | ---              |  PA25  | USB_POSITIVE      | USB/DP
 * | 15      | 27                 | A27              |  PA27  |                   | EIC/EXTINT[7] PTC/X[10]
 * | 16      | 30                 | A30              |  PA30  | TX / SWD CLK      | !EIC/EXTINT[2] !SERCOM1/PAD[0] SERCOM1/PAD[2] TC2/WO[0] !TCC0/WO[2] SWD CLK
 * | 17      | 31                 | A31              |  PA31  | RX / SWD IO       | EIC/EXTINT[3] !SERCOM1/PAD[1] SERCOM1/PAD[3] TC2/WO[1] !TCC0/WO[3] SWD IO
 * | 18      | --                 | Vcc              |  ----  |                   | Vcc
 * | 19      | --                 | Gnd              |  ----  |                   | Ground
 * +---------+--------------------+------------------+--------+-------------------+-------------------------------------------------
 * 
 * You may use Arduino pin numbers ranging from 0 to 31. The Arduino pin corresponds to the silkscreen (without the 'A').
 * For example, use pinMode(16, OUTPUT) to set the LED pin (marked as A16) as an output.
 * However, the following Arduino pin numbers are not mapped to a physical pin: 0, 1, 8, 9, 12, 13, 18, 19, 20, 21, 24, 25, 26, 28, and 29.
 * Pins 8 and 9 are used by the 32.768KHz crystal which in turn is used by the Arduino core (the 16MHz crystal is unused by Arduino).
 * Pins 24 and 25 are in use by USB (USB_NEGATIVE and USB_POSITIVE).
 * The tone library uses TC2.
 */


#include "variant.h"

/*
 * Pins descriptions
 */
#if defined PIN_MAP_STANDARD
const PinDescription g_APinDescription[]=
{
	// 0..1 pins don't exist
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	
	// 2..7 - Analog capable pins (DAC available on 2)
	{ PORTA,  2, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_ADC|PIN_ATTR_DAC|PIN_ATTR_DIGITAL), NOT_ON_TIMER, ADC_Channel0, EXTERNAL_INT_NONE },
	{ PORTA,  3, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_ADC|PIN_ATTR_REF|PIN_ATTR_DIGITAL), NOT_ON_TIMER, ADC_Channel1, EXTERNAL_INT_NONE },
	{ PORTA,  4, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_ALT), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), TCC0_CH0, ADC_Channel2, EXTERNAL_INT_4 },
	{ PORTA,  5, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_ALT), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), TCC0_CH1, ADC_Channel3, EXTERNAL_INT_5 },
	{ PORTA,  6, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_ALT), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), TCC0_CH2, ADC_Channel4, EXTERNAL_INT_NONE },
	{ PORTA,  7, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_ALT), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), TCC0_CH3, ADC_Channel5, EXTERNAL_INT_NONE },
	
	// 8..9 are unused (pins in use by 32.768KHz crystal, which in turn is used by the Arduino core)
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	
	// 10..11 - SERCOM/SPI (SPI) or Analog or Digital functions
	{ PORTA, 10, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), NOT_ON_TIMER, ADC_Channel8, EXTERNAL_INT_2 }, // SPI MOSI
	{ PORTA, 11, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM), NOT_ON_TIMER, ADC_Channel9, EXTERNAL_INT_NONE }, // SPI SCK
	
	// 12..13 pins don't exist
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	
	// 14..15 - SERCOM/SPI (SPI) or Analog or Digital functions (Button available on pin 15)
	{ PORTA, 14, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), NOT_ON_TIMER, ADC_Channel6, EXTERNAL_INT_NMI }, // SPI MISO
	{ PORTA, 15, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), NOT_ON_TIMER, ADC_Channel7, EXTERNAL_INT_1 }, // Button / SPI SS (unused)
	
	// 16..17 Digital functions (LED available on pin 16)
	{ PORTA, 16, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), TC1_CH0, No_ADC_Channel, EXTERNAL_INT_0 }, // LED
	{ PORTA, 17, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), TC1_CH1, No_ADC_Channel, EXTERNAL_INT_NONE },
	
	// 18..21 pins don't exist
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	
	// 22..23 SERCOM/I2C (Wire) or Digital functions
	{ PORTA, 22, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_ALT), (PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_6 }, // SDA
	{ PORTA, 23, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_ALT), (PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM), NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // SCL
	
	// 24..26 are unused (25 and 26 in use by USB_NEGATIVE and USB_POSITIVE, pin 26 does not exist)
	{ PORTA, 24, PIO_COM, PER_ATTR_NONE, PIN_ATTR_COM, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // USB/DM
	{ PORTA, 25, PIO_COM, PER_ATTR_NONE, PIN_ATTR_COM, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // USB/DP
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	
	// 27..29 Digital functions (pin 28 is Reset and pin 29 does not exist)
	{ PORTA, 27, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT), NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_7 },
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused (Reset)
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	
	// 30..31  SERCOM/UART (Serial1) or Digital functions or Debug interface (SWD CLK and SWD IO)
	{ PORTA, 30, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_ALT), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM), TC2_CH0, No_ADC_Channel, EXTERNAL_INT_NONE }, // TX / SWD CLK
	{ PORTA, 31, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_ALT), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TC2_CH1, No_ADC_Channel, EXTERNAL_INT_3 }, // RX / SWD IO
} ;
#elif defined PIN_MAP_COMPACT
const PinDescription g_APinDescription[]=
{
	// 0..5 - Analog capable pins (DAC available on 0)
	{ PORTA,  2, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_ADC|PIN_ATTR_DAC|PIN_ATTR_DIGITAL), NOT_ON_TIMER, ADC_Channel0, EXTERNAL_INT_NONE },
	{ PORTA,  3, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_ADC|PIN_ATTR_REF|PIN_ATTR_DIGITAL), NOT_ON_TIMER, ADC_Channel1, EXTERNAL_INT_NONE },
	{ PORTA,  4, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_ALT), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), TCC0_CH0, ADC_Channel2, EXTERNAL_INT_4 },
	{ PORTA,  5, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_ALT), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), TCC0_CH1, ADC_Channel3, EXTERNAL_INT_5 },
	{ PORTA,  6, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_ALT), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), TCC0_CH2, ADC_Channel4, EXTERNAL_INT_NONE },
	{ PORTA,  7, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_ALT), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), TCC0_CH3, ADC_Channel5, EXTERNAL_INT_NONE },
	
	// 6..9 - SERCOM/SPI (SPI) or Analog or Digital functions (Button available on pin 9)
	{ PORTA, 10, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), NOT_ON_TIMER, ADC_Channel8, EXTERNAL_INT_2 }, // SPI MOSI
	{ PORTA, 11, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM), NOT_ON_TIMER, ADC_Channel9, EXTERNAL_INT_NONE }, // SPI SCK
	{ PORTA, 14, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), NOT_ON_TIMER, ADC_Channel6, EXTERNAL_INT_NMI }, // SPI MISO
	{ PORTA, 15, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), NOT_ON_TIMER, ADC_Channel7, EXTERNAL_INT_1 }, // Button / SPI SS (unused)
	
	// 10..11 Digital functions (LED available on pin 10)
	{ PORTA, 16, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), TC1_CH0, No_ADC_Channel, EXTERNAL_INT_0 }, // LED
	{ PORTA, 17, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), TC1_CH1, No_ADC_Channel, EXTERNAL_INT_NONE },
	
	// 12..13 SERCOM/I2C (Wire) or Digital functions
	{ PORTA, 22, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_ALT), (PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_6 }, // SDA
	{ PORTA, 23, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_ALT), (PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM), NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // SCL
	
	// 14 Digital functions
	{ PORTA, 27, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT), NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_7 },
	
	// 15..16  SERCOM/UART (Serial1) or Digital functions or Debug interface (SWD CLK and SWD IO)
	{ PORTA, 30, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_ALT), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM), TC2_CH0, No_ADC_Channel, EXTERNAL_INT_NONE }, // TX / SWD CLK
	{ PORTA, 31, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_ALT), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TC2_CH1, No_ADC_Channel, EXTERNAL_INT_3 }, // RX / SWD IO
} ;
#endif

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TC1, TC2 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;

#if defined(ARDUINO_UART_ONLY) || defined(ARDUINO_CDC_HID_UART) || defined(ARDUINO_CDC_UART) || defined(ARDUINO_HID_UART)
Uart Serial1( SERCOM_INSTANCE_SERIAL1, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

void SERCOM1_Handler()
{
  Serial1.IrqHandler();
}
#endif
