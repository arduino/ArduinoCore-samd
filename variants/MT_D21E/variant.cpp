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
  Modified 20 July 2015 by Justin Mattair
     for MattairTech MT-D21E boards (www.mattairtech.com)
*/

/*
============================================ MattairTech MT-D21E (ATsamd21eXXa) ========================================
Comm/Other INT      PWM    Digital    Analog                               Digital    PWM       INT    Comm/other
========================================================================================================================
                                                  -----------------------
Xin32                                            |   A0            RST   |                             Reset
Xout32                                           |   A1            NC    |
                             2     2 (ADC0, DAC) |   A2            NC    |
                             3     3 (ADC1, REF) |   A3            A31   |   31    TCC1/WO[1]  INT11   Button B / SWD IO
          INT4               4     4 (ADC4)      |   A4            A30   |   30    TCC1/WO[0]  INT10   SWD CLK
          INT5               5     5 (ADC5)      |   A5            NC    |
                             6     6 (ADC6)      |   A6            A28   |   28                INT8    LED
VDIV                         7     7 (ADC7)      |   A7            A27   |   27                INT15   Button A
          INTNMI TCC0/WO[0]  8     8 (ADC16)     |   A8            A23   |   23    TC4/WO[1]   INT7    SPI SS
          INT9   TCC0/WO[1]  9     9 (ADC17)     |   A9            A22   |   22    TC4/WO[0]   INT6    SPI MISO
TX (1)           TCC0/WO[2]  10    10 (ADC18)    |   A10           A19   |   19                INT3    SPI SCK
RX (1)           TCC0/WO[3]  11    11 (ADC19)    |   A11           A18   |   18                INT2    SPI MOSI
TX (2)    INT14  TC3/WO[0]   14                  |   A14           A17   |   17    TCC2/WO[1]  INT1    I2C/SCL
RX (2)           TC3/WO[1]   15                  |   A15           A16   |   16    TCC2/WO[0]  INT0    I2C/SDA
                                                 |   NC            NC    |
                                                 |   NC            NC    |
                                                 |   Vbus          3.3V  |
USB D-                                           |   A24-  _____   Vcc   |
USB D+                                           |   A25+ |     |  Vin   |
                                                 |   Gnd  | USB |  Gnd   |
                                                  -----------------------
  Most pins have multiple configurations available (even analog pins). For example, pin A10 can be an analog input, a
  PWM output, Digital I/O, or the TX pin of 'Serial1'. These always reference the pin number printed on the board but
  without the 'A' (with the usable pins starting at 2). DO NOT connect voltages higher than 3.3V!
*/


/*  Pins descriptions for the MattairTech MT-D21E
 * 
 * | PCB Pin | Arduino Pin Number |    Silkscreen    |  PIN   |   Alt. Function   | Comments (* is for default peripheral in use, ! means unavailable with this variant)
 * +---------+--------------------+------------------+--------+-------------------+-------------------------------------------------
 * | 0       | --                 | A0               |  PA00  | Xin32             | *Xin32
 * | 1       | --                 | A1               |  PA01  | Xout32            | *Xout32
 * | 2       | 2                  | A2               |  PA02  |                   | !EIC/EXTINT[2] *ADC/AIN[0] PTC/Y[0] DAC/VOUT
 * | 3       | 3                  | A3               |  PA03  | REFA              | !EIC/EXTINT[3] REF/ADC/VREFA REF/DAC/VREFA *ADC/AIN[1] PTC/Y[1]
 * | 4       | 4                  | A4               |  PA04  | REFB              | EIC/EXTINT[4] REF/ADC/VREFB *ADC/AIN[4] AC/AIN[0] PTC/Y[2] !SERCOM0/PAD[0] !TCC0/WO[0]
 * | 5       | 5                  | A5               |  PA05  |                   | EIC/EXTINT[5] *ADC/AIN[5] AC/AIN[1] PTC/Y[3] !SERCOM0/PAD[1] !TCC0/WO[1]
 * | 6       | 6                  | A6               |  PA06  |                   | !EIC/EXTINT[6] *ADC/AIN[6] AC/AIN[2] PTC/Y[4] !SERCOM0/PAD[2] !TCC1/WO[0]
 * | 7       | 7                  | A7               |  PA07  | Voltage Divider   | !EIC/EXTINT[7] *ADC/AIN[7] AC/AIN[3] PTC/Y[5] !SERCOM0/PAD[3] !TCC1/WO[1]
 * | 8       | 8                  | A8               |  PA08  |                   | EIC/NMI ADC/AIN[16] PTC/X[0] !SERCOM0/PAD[0] !SERCOM2/PAD[0] *TCC0/WO[0] !TCC1/WO[2]
 * | 9       | 9                  | A9               |  PA09  |                   | EIC/EXTINT[9] ADC/AIN[17] PTC/X[1] !SERCOM0/PAD[1] !SERCOM2/PAD[1] *TCC0/WO[1] !TCC1/WO[3]
 * | 10      | 10                 | A10              |  PA10  | TX                | !EIC/EXTINT[10] ADC/AIN[18] PTC/X[2] *SERCOM0/PAD[2] !SERCOM2/PAD[2] !TCC1/WO[0] TCC0/WO[2]
 * | 11      | 11                 | A11              |  PA11  | RX                | !EIC/EXTINT[11] ADC/AIN[19] PTC/X[3] *SERCOM0/PAD[3] !SERCOM2/PAD[3] !TCC1/WO[1] TCC0/WO[3]
 * | 12      | 14                 | A14              |  PA14  | Xin, HOST_ENABLE  | EIC/EXTINT[14] SERCOM2/PAD[2] *TC3/WO[0] !TCC0/WO[4] Xin, HOST_ENABLE
 * | 13      | 15                 | A15              |  PA15  | Xout              | !EIC/EXTINT[15] SERCOM2/PAD[3] *TC3/WO[1] !TCC0/WO[5] Xout
 * | 14      | --                 | NC               |  ----  |                   | Not Connected
 * | 15      | --                 | NC               |  ----  |                   | Not Connected
 * | 16      | --                 | Vbus             |  ----  |                   | USB Vbus
 * | 17      | --                 | A24-             |  PA24  | USB_NEGATIVE      | *USB/DM
 * | 18      | --                 | A25+             |  PA25  | USB_POSITIVE      | *USB/DP
 * | 19      | --                 | Gnd              |  ----  |                   | Ground
 * | 20      | --                 | Gnd              |  ----  |                   | Ground
 * | 21      | --                 | Vin              |  ----  |                   | Vin
 * | 22      | --                 | Vcc              |  ----  |                   | Vcc
 * | 23      | --                 | 3.3V             |  ----  |                   | 3.3V
 * | 24      | --                 | NC               |  ----  |                   | Not Connected
 * | 25      | --                 | NC               |  ----  |                   | Not Connected
 * | 26      | 16                 | A16              |  PA16  | I2C/SDA w/pullup  | EIC/EXTINT[0] PTC/X[4] *SERCOM1/PAD[0] !SERCOM3/PAD[0] TCC2/WO[0] !TCC0/WO[6]
 * | 27      | 17                 | A17              |  PA17  | I2C/SCL w/pullup  | EIC/EXTINT[1] PTC/X[5] *SERCOM1/PAD[1] !SERCOM3/PAD[1] TCC2/WO[1] !TCC0/WO[7]
 * | 28      | 18                 | A18              |  PA18  | SPI MOSI          | EIC/EXTINT[2] PTC/X[6] !SERCOM1/PAD[2] *SERCOM3/PAD[2] !TC3/WO[0] !TCC0/WO[2]
 * | 29      | 19                 | A19              |  PA19  | SPI SCK           | EIC/EXTINT[3] PTC/X[7] !SERCOM1/PAD[3] *SERCOM3/PAD[3] !TC3/WO[1] !TCC0/WO[3]
 * | 30      | 22                 | A22              |  PA22  | SPI MISO          | EIC/EXTINT[6] PTC/X[10] *SERCOM3/PAD[0] TC4/WO[0] !TCC0/WO[4]
 * | 31      | 23                 | A23              |  PA23  | SPI SS            | EIC/EXTINT[7] PTC/X[11] *SERCOM3/PAD[1] TC4/WO[1] !TCC0/WO[5]
 * | 32      | 27                 | A27              |  PA27  | Button A          | EIC/EXTINT[15] *Button A
 * | 33      | 28                 | A28              |  PA28  | LED               | EIC/EXTINT[8] *LED
 * | 34      | --                 | NC               |  ----  |                   | Not Connected
 * | 35      | 30                 | A30              |  PA30  | SWD CLK           | EIC/EXTINT[10] !SERCOM1/PAD[2] *TCC1/WO[0] SWD CLK
 * | 36      | 31                 | A31              |  PA31  | Button B / SWD IO | EIC/EXTINT[11] !SERCOM1/PAD[3] *TCC1/WO[1] Button B SWD IO
 * | 37      | --                 | NC               |  ----  |                   | Not Connected
 * | 38      | --                 | NC               |  ----  |                   | Not Connected
 * | 39      | --                 | RST              |  ----  |                   | Reset
 * +---------+--------------------+------------------+--------+-------------------+-------------------------------------------------
 * 
 * You may use Arduino pin numbers ranging from 0 to 31. The Arduino pin corresponds to the silkscreen (without the 'A').
 * For example, use pinMode(28, OUTPUT) to set the LED pin (marked as A28) as an output.
 * However, the following Arduino pin numbers are not mapped to a physical pin: 0, 1, 12, 13, 20, 21, 24, 25, 26, and 29.
 * Pins 0 and 1 are used by the 32.768KHz crystal which in turn is used by the Arduino core (the 16MHz crystal is unused by Arduino).
 * Pins 24 and 25 are in use by USB (USB_NEGATIVE and USB_POSITIVE).
 * When using pins 22 and/or 23 as pwm outputs, it will use Timer TC4, which conflicts with the servo library.
 * The tone library uses TC5.
 */


/*   The PinDescription table describes how each of the pins can be used by the Arduino core. Each pin can have multiple
 *   functions (ie: ADC input, digital output, PWM, communications, etc.), and the PinDescription table configures
 *   which functions can be used for each pin. This table is mainly accessed by the pinPeripheral function in
 *   wiring_private.c, which is used to attach a pin to a particular peripheral function. The communications drivers
 *   (ie: SPI, I2C, and UART), analogRead(), analogWrite(), analogReference(), attachInterrupt(), and pinMode() all
 *   call pinPeripheral() to verify that the pin can perform the function requested, and to configure the pin for
 *   that function. Most of the contents of pinMode() are now in pinPeripheral().
 * 
 *   Explanation of PinDescription table:
 * 
 *   Port			This is the port (ie: PORTA).
 *   Pin			This is the pin (bit) within the port. Valid values are 0 through 31.
 *   PinType			This indicates what peripheral function the pin can be attached to. In most cases, this
 *                              is PIO_MULTI, which means that the pin can be anything listed in the PinAttribute field.
 *                              It can also be set to a specific peripheral. In this case, any attempt to configure the
 *                              pin (using pinPeripheral or pinMode) as anything else will fail (and pinPeripheral will
 *                              return -1). This can be used to prevent accidental re-configuration of a pin that is
 *                              configured for only one function (ie: USB D- and D+ pins). If a pin is not used or does
 *                              not exist, PIO_NOT_A_PIN must be entered in this field. See WVariant.h for valid
 *                              entries. These entries are also used as a parameter to pinPeripheral() with the
 *                              exception of PIO_NOT_A_PIN and PIO_MULTI. The pinMode function now calls pinPeripheral()
 *                              with the desired mode. Note that this field is not used to select between the two
 *                              peripherals possible with each of the SERCOM and TIMER functions. PeripheralAttribute
 *                              is now used for this.
 *   PeripheralAttribute	This is an 8-bit bitfield used for various peripheral configuration. It is primarily
 *                              used to select between the two peripherals possible with each of the SERCOM and TIMER
 *                              functions. TIMER pins are individual, while SERCOM uses a group of two to four pins.
 *                              This group of pins can span both peripherals. For example, pin 19 (SPI1 SCK) uses
 *                              PER_ATTR_SERCOM_ALT while pin 22 (SPI1 MISO) uses PER_ATTR_SERCOM_STD. Both TIMER and
 *                              SERCOM can exist for each pin. This bitfield is also used to set the pin drive strength.
 *                              In the future, other attributes (like input buffer configuration) may be added.
 *                              See WVariant.h for valid entries.
 *   PinAttribute		This is a 32-bit bitfield used to list all of the valid peripheral functions that a pin
 *                              can attach to. This includes GPIO functions like PIN_ATTR_OUTPUT. Certain attributes
 *                              are shorthand for a combination of other attributes. PIN_ATTR_DIGITAL includes all of
 *                              the GPIO functions, while PIN_ATTR_TIMER includes both PIN_ATTR_TIMER_PWM and
 *                              PIN_ATTR_TIMER_CAPTURE (capture is not used yet). PIN_ATTR_ANALOG is an alias to
 *                              PIN_ATTR_ANALOG_ADC. There is only one DAC channel, so PIN_ATTR_DAC appears only once.
 *                              This bitfield is useful for limiting a pin to only input related functions or output
 *                              functions. This allows a pin to have a more flexible configuration, while restricting
 *                              the direction (ie: to avoid contention). See WVariant.h for valid entries.
 *   TCChannel			This is the TC(C) channel (if any) assigned to the pin. Some TC channels are available
 *                              on multiple pins (ie: TCC0/WO[0] is available on pin A4 or pin A8). In general, only
 *                              one pin should be configured (in the pinDescription table) per TC channel.
 *                              See WVariant.h for valid entries. The tone library uses TC5.
 *   ADCChannelNumber		This is the ADC channel (if any) assigned to the pin. See WVariant.h for valid entries.
 *   ExtInt			This is the interrupt (if any) assigned to the pin. Some interrupt numbers are available
 *                              on multiple pins (ie: EIC/EXTINT[2] is available on pin A2 or pin A18). In general, only
 *                              one pin should be configured (in the pinDescription table) per interrupt number. Thus,
 *                              if an interrupt was needed on pin 2, EXTERNAL_INT_2 can be moved from pin 18.
 *                              See WVariant.h for valid entries.
 */

#include "variant.h"

/*
 * Pins descriptions
 */
const PinDescription g_APinDescription[]=
{
	// 0..1 are unused (pins in use by 32.768KHz crystal, which in turn is used by the Arduino core)
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	
	// 2..9 - Analog capable pins (DAC avalable on 2)
	{ PORTA,  2, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_ADC|PIN_ATTR_DAC), NOT_ON_TIMER, ADC_Channel0, EXTERNAL_INT_NONE }, // ADC/AIN[0] / DAC
	{ PORTA,  3, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_ADC|PIN_ATTR_REF), NOT_ON_TIMER, ADC_Channel1, EXTERNAL_INT_NONE }, // ADC/AIN[1]
	{ PORTA,  4, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT), NOT_ON_TIMER, ADC_Channel4, EXTERNAL_INT_4 }, // ADC/AIN[4]
	{ PORTA,  5, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT), NOT_ON_TIMER, ADC_Channel5, EXTERNAL_INT_5 }, // ADC/AIN[5]
	{ PORTA,  6, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_ADC|PIN_ATTR_DIGITAL), NOT_ON_TIMER, ADC_Channel6, EXTERNAL_INT_NONE }, // ADC/AIN[6]
	{ PORTA,  7, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_COM), NOT_ON_TIMER, ADC_Channel7, EXTERNAL_INT_NONE }, // ADC/AIN[7]
	{ PORTA,  8, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), TCC0_CH0, ADC_Channel16, EXTERNAL_INT_NMI }, // TCC0/WO[0]
	{ PORTA,  9, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_EXTINT|PIN_ATTR_COM), TCC0_CH1, ADC_Channel17, EXTERNAL_INT_9 }, // TCC0/WO[1]
	
	// 10..11 - SERCOM/UART (Serial1) or Analog or Digital functions
	{ PORTA, 10, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_ALT|PER_ATTR_SERCOM_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_COM), TCC0_CH2, ADC_Channel18, EXTERNAL_INT_NONE }, // TX: SERCOM0/PAD[2] (PIN_ATTR_TIMER_ALT)
	{ PORTA, 11, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_ALT|PER_ATTR_SERCOM_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_COM), TCC0_CH3, ADC_Channel19, EXTERNAL_INT_NONE }, // RX: SERCOM0/PAD[3] (PIN_ATTR_TIMER_ALT)
	
	// 12..13 pins don't exist
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	
	// 14..15 - Digital functions
	{ PORTA, 14, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TC3_CH0, No_ADC_Channel, EXTERNAL_INT_14 }, // TC3/WO[0], HOST_ENABLE
	{ PORTA, 15, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM), TC3_CH1, No_ADC_Channel, EXTERNAL_INT_NONE }, // TC3/WO[1], ATN
	
	// 16..17 I2C pins (SDA/SCL) or Digital functions
	{ PORTA, 16, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TCC2_CH0, No_ADC_Channel, EXTERNAL_INT_0 }, // SDA: SERCOM1/PAD[0]
	{ PORTA, 17, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TCC2_CH1, No_ADC_Channel, EXTERNAL_INT_1 }, // SCL: SERCOM1/PAD[1]
	
	// 18..23 - SPI Pins or Digital functions (pins 20..21 do not exist)
	{ PORTA, 18, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_ALT), (PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_2 }, // SPI MOSI: SERCOM3/PAD[2] (PIN_ATTR_SERCOM_ALT)
	{ PORTA, 19, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_ALT), (PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_3 }, // SPI SCK: SERCOM3/PAD[3] (PIN_ATTR_SERCOM_ALT)
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	{ PORTA, 22, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TC4_CH0, No_ADC_Channel, EXTERNAL_INT_6 }, // SPI MISO: SERCOM3/PAD[0]
	{ PORTA, 23, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TC4_CH1, No_ADC_Channel, EXTERNAL_INT_7 }, // SPI SS: SERCOM3/PAD[1]
	
	// 24..26 are unused (25 and 26 in use by USB_NEGATIVE and USB_POSITIVE, pin 26 does not exist)
	{ PORTA, 24, PIO_COM, PER_ATTR_NONE, PIN_ATTR_COM, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // USB/DM
	{ PORTA, 25, PIO_COM, PER_ATTR_NONE, PIN_ATTR_COM, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // USB/DP
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	
	// 27..29 Button A and LED (pin 29 does not exist)
	{ PORTA, 27, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_INPUT_PULLUP|PIN_ATTR_EXTINT), NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_15 }, // Button A
	{ PORTA, 28, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_OUTPUT|PIN_ATTR_INPUT|PIN_ATTR_INPUT_PULLDOWN|PIN_ATTR_EXTINT), NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_8 }, // LED
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PER_ATTR_NONE, PIN_ATTR_NONE, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // Unused
	
	// 30..31 Digital functions / Debug interface (SWD CLK and SWD IO)
	{ PORTA, 30, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), TCC1_CH0, No_ADC_Channel, EXTERNAL_INT_10 }, // TCC1/WO[0] / SWD CLK
	{ PORTA, 31, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), TCC1_CH1, No_ADC_Channel, EXTERNAL_INT_11 }, // TCC1/WO[1] / SWD IO
} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TC3, TC4, TC5 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;

Uart Serial1( SERCOM_INSTANCE_SERIAL1, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;
Uart Serial2( SERCOM_INSTANCE_SERIAL2, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX ) ;

void SERCOM0_Handler()
{
  Serial1.IrqHandler();
}

void SERCOM2_Handler()
{
  Serial2.IrqHandler();
}

