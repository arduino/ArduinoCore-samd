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

/*
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * + Pin number +  ZERO Board pin  |  PIN   | Label/Name      | Comments (* is for available peripherals with this configuration)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | Digital Low      |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 0          | 0 -> RX          |  PA11  |                 | *EIC/EXTINT[11] *ADC/AIN[19]           PTC/X[3] *SERCOM0/PAD[3]   SERCOM2/PAD[3]  TCC0/WO[3]  TCC1/WO[1]
 * | 1          | 1 <- TX          |  PA10  |                 | *EIC/EXTINT[10] *ADC/AIN[18]           PTC/X[2] *SERCOM0/PAD[2]                   TCC0/WO[2]  TCC1/WO[0]
 * | 2          | 2                |  PA14  |                 | *EIC/EXTINT[14]                                  SERCOM2/PAD[2]   SERCOM4/PAD[2]  TC3/WO[0]   TCC0/WO[4]
 * | 3          | ~3               |  PA09  |                 | *EIC/EXTINT[9]  *ADC/AIN[17]           PTC/X[1]  SERCOM0/PAD[1]  *SERCOM2/PAD[1] *TCC0/WO[1]  TCC1/WO[3]
 * | 4          | ~4               |  PA08  |                 | *EIC/NMI        *ADC/AIN[16]           PTC/X[0]  SERCOM0/PAD[0]  *SERCOM2/PAD[0] *TCC0/WO[0]  TCC1/WO[2]
 * | 5          | ~5               |  PA15  |                 | *EIC/EXTINT[15]                                  SERCOM2/PAD[3]   SERCOM4/PAD[3] *TC3/WO[1]   TCC0/WO[5]
 * | 6          | ~6               |  PA20  |                 | *EIC/EXTINT[4]                         PTC/X[8]  SERCOM5/PAD[2]   SERCOM3/PAD[2]             *TCC0/WO[6]
 * | 7          | 7                |  PA21  |                 | *EIC/EXTINT[5]                         PTC/X[9]  SERCOM5/PAD[3]   SERCOM3/PAD[3]              TCC0/WO[7]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | Digital High     |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 8          | ~8               |  PA06  |                 | EIC/EXTINT[6]  *ADC/AIN[6]  AC/AIN[2] PTC/Y[4]  SERCOM0/PAD[2]                 *TCC1/WO[0]
 * | 9          | ~9               |  PA07  |                 | EIC/EXTINT[7]  *ADC/AIN[7]  AC/AIN[3] PTC/Y[5]  SERCOM0/PAD[3]                 *TCC1/WO[1]
 * | 10         | ~10              |  PA18  |                 | *EIC/EXTINT[2]                        PTC/X[6] *SERCOM1/PAD[2]  SERCOM3/PAD[2] *TC3/WO[0]    TCC0/WO[2]
 * | 11         | ~11              |  PA16  |                 | *EIC/EXTINT[0]                        PTC/X[4] *SERCOM1/PAD[0]  SERCOM3/PAD[0] *TCC2/WO[0]   TCC0/WO[6]
 * | 12         | ~12              |  PA19  |                 | *EIC/EXTINT[3]                        PTC/X[7] *SERCOM1/PAD[3]  SERCOM3/PAD[3]  TC3/WO[1]   *TCC0/WO[3]
 * | 13         | ~13              |  PA17  | LED             | *EIC/EXTINT[1]                        PTC/X[5] *SERCOM1/PAD[1]  SERCOM3/PAD[1] *TCC2/WO[1]   TCC0/WO[7]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | Analog Connector |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 14         | A0               |  PA02  | A0              | EIC/EXTINT[2]  *ADC/AIN[0]  *DAC/VOUT  PTC/Y[0]
 * | 15         | A1               |  PB08  | A1              | *EIC/EXTINT[8] *ADC/AIN[2]            PTC/Y[14] SERCOM4/PAD[0]                  TC4/WO[0]
 * | 16         | A2               |  PB09  | A2              | EIC/EXTINT[9]  *ADC/AIN[3]            PTC/Y[15] SERCOM4/PAD[1]                  TC4/WO[1]
 * | 17         | A3               |  PA04  | A3              | EIC/EXTINT[4]  *ADC/AIN[4]  AC/AIN[0] PTC/Y[2]  SERCOM0/PAD[0]                  TCC0/WO[0]
 * | 18         | A4               |  PA05  | A4              | EIC/EXTINT[5]  *ADC/AIN[5]  AC/AIN[1] PTC/Y[5]  SERCOM0/PAD[1]                  TCC0/WO[1]
 * | 19         | A5               |  PB02  | A5              | EIC/EXTINT[2]  *ADC/AIN[10]           PTC/Y[8]  SERCOM5/PAD[0]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | Wire             |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 20         | SDA              |  PA22  | SDA             | *EIC/EXTINT[6]                        PTC/X[10] *SERCOM3/PAD[0] SERCOM5/PAD[0] *TC4/WO[0] TCC0/WO[4]
 * | 21         | SCL              |  PA23  | SCL             | *EIC/EXTINT[7]                        PTC/X[11] *SERCOM3/PAD[1] SERCOM5/PAD[1] *TC4/WO[1] TCC0/WO[5]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            |SPI (Legacy ICSP) |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 22         | 1                |  PA12  | MISO            | *EIC/EXTINT[12] SERCOM2/PAD[0] *SERCOM4/PAD[0] TCC2/WO[0] TCC0/WO[6]
 * |            | 2                |        | 5V0             |
 * | 23         | 4                |  PB10  | MOSI            | EIC/EXTINT[10]                *SERCOM4/PAD[2] *TC5/WO[0]  TCC0/WO[4]
 * | 24         | 3                |  PB11  | SCK             | EIC/EXTINT[11]                *SERCOM4/PAD[3] *TC5/WO[1]  TCC0/WO[5]
 * |            | 5                |        | RESET           |
 * |            | 6                |        | GND             |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | LEDs             |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 25         |                  |  PB03  | RX              |
 * | 26         |                  |  PA27  | TX              |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | USB              |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 27         |                  |  PA28  | USB_HOST_ENABLE | EIC/EXTINT[8]
 * | 28         |                  |  PA24  | USB_NEGATIVE    | *USB/DM
 * | 29         |                  |  PA25  | USB_POSITIVE    | *USB/DP
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | EDBG             |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 30         |                  |  PB22  | EDBG_UART TX    | *SERCOM5/PAD[2]
 * | 31         |                  |  PB23  | EDBG_UART RX    | *SERCOM5/PAD[3]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 32         |                  |  PA22  | EDBG_SDA        | Pin 20 (SDA)
 * | 33         |                  |  PA23  | EDBG_SCL        | Pin 21 (SCL)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 34         |                  |  PA19  | EDBG_MISO       | EIC/EXTINT[3] *SERCOM1/PAD[3] SERCOM3/PAD[3] TC3/WO[1]  TCC0/WO[3]
 * | 35         |                  |  PA16  | EDBG_MOSI       | EIC/EXTINT[0] *SERCOM1/PAD[0] SERCOM3/PAD[0] TCC2/WO[0] TCC0/WO[6]
 * | 36         |                  |  PA18  | EDBG_SS         | EIC/EXTINT[2] *SERCOM1/PAD[2] SERCOM3/PAD[2] TC3/WO[0]  TCC0/WO[2]
 * | 37         |                  |  PA17  | EDBG_SCK        | EIC/EXTINT[1] *SERCOM1/PAD[1] SERCOM3/PAD[1] TCC2/WO[1] TCC0/WO[7]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 38         | ATN              |  PA13  | EDBG_GPIO0      | *EIC/EXTINT[13] SERCOM2/PAD[1] SERCOM4/PAD[1] TCC2/WO[1] TCC0/WO[7]
 * | 39         |                  |  PA21  | EDBG_GPIO1      | Pin 7
 * | 40         |                  |  PA06  | EDBG_GPIO2      | Pin 8
 * | 41         |                  |  PA07  | EDBG_GPIO3      | Pin 9
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            |                  |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | GND              |        |                 |
 * | 42         | AREF             |  PA03  |                 | EIC/EXTINT[3] *[ADC|DAC]/VREFA *ADC/AIN[1] PTC/Y[1]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            |32.768KHz Crystal |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            |                  |  PA00  | XIN32           | EIC/EXTINT[0] SERCOM1/PAD[0] TCC2/WO[0]
 * |            |                  |  PA01  | XOUT32          | EIC/EXTINT[1] SERCOM1/PAD[1] TCC2/WO[1]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 */


#include "variant.h"

/*
 * Pins descriptions
 */
const PinDescription g_APinDescription[]=
{
  // 0..13 - Digital pins
  // ----------------------
  // 0/1 - SERCOM/UART (Serial1)
  { PORTA, 11, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT|PIN_ATTR_COM), NOT_ON_TIMER, ADC_Channel19, EXTERNAL_INT_11 }, // RX: SERCOM0/PAD[3]
  { PORTA, 10, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT|PIN_ATTR_COM), NOT_ON_TIMER, ADC_Channel18, EXTERNAL_INT_10 }, // TX: SERCOM0/PAD[2]

  // 2..12
  // Digital Low
  { PORTA, 14, PIO_MULTI, (PER_ATTR_DRIVE_STRONG), (PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT), NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_14 },
  { PORTA,  9, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_ALT), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT|PIN_ATTR_COM), TCC0_CH1, ADC_Channel17, EXTERNAL_INT_9 }, // TCC0/WO[1] SCL: SERCOM2/PAD[1]
  { PORTA,  8, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_ALT), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT|PIN_ATTR_COM), TCC0_CH0, ADC_Channel16, EXTERNAL_INT_NMI }, // TCC0/WO[0] SDA: SERCOM2/PAD[0]
  { PORTA, 15, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), TC3_CH1, No_ADC_Channel, EXTERNAL_INT_15 }, // TC3/WO[1]
  { PORTA, 20, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_ALT), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_EXTINT|PIN_ATTR_COM), TCC0_CH6, No_ADC_Channel, EXTERNAL_INT_4 }, // TCC0/WO[6]
  { PORTA, 21, PIO_MULTI, (PER_ATTR_DRIVE_STRONG), (PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT|PIN_ATTR_COM), NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_5 },

  // Digital High
  { PORTA,  6, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), TCC1_CH0, ADC_Channel6, EXTERNAL_INT_6 }, // TCC1/WO[0] ADC/AIN[6]
  { PORTA,  7, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_COM), TCC1_CH1, ADC_Channel7, EXTERNAL_INT_7 }, // TCC1/WO[1] ADC/AIN[7]
  { PORTA, 18, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TC3_CH0, No_ADC_Channel, EXTERNAL_INT_2 }, // TC3/WO[0] SS: SERCOM1/PAD[2]
  { PORTA, 16, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TCC2_CH0, No_ADC_Channel, EXTERNAL_INT_0 }, // TCC2/WO[0] MOSI: SERCOM1/PAD[0]
  { PORTA, 19, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_ALT|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TCC0_CH3, No_ADC_Channel, EXTERNAL_INT_3 }, // TCC0/WO[3] MISO: SERCOM1/PAD[3]

  // 13 (LED)
  { PORTA, 17, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TCC2_CH1, No_ADC_Channel, EXTERNAL_INT_1 }, // TCC2/WO[1] SCK: SERCOM1/PAD[1]

  // 14..19 - Analog pins
  // --------------------
  { PORTA,  2, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_ADC|PIN_ATTR_DAC|PIN_ATTR_DIGITAL), NOT_ON_TIMER, ADC_Channel0, EXTERNAL_INT_NONE }, // ADC/AIN[0] / DAC
  { PORTB,  8, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT), NOT_ON_TIMER, ADC_Channel2, EXTERNAL_INT_8 }, // ADC/AIN[2]
  { PORTB,  9, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_ADC|PIN_ATTR_DIGITAL), NOT_ON_TIMER, ADC_Channel3, EXTERNAL_INT_NONE }, // ADC/AIN[3]
  { PORTA,  4, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_ADC|PIN_ATTR_DIGITAL), NOT_ON_TIMER, ADC_Channel4, EXTERNAL_INT_NONE }, // ADC/AIN[4]
  { PORTA,  5, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_ADC|PIN_ATTR_DIGITAL), NOT_ON_TIMER, ADC_Channel5, EXTERNAL_INT_NONE }, // ADC/AIN[5]
  { PORTB,  2, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_ADC|PIN_ATTR_DIGITAL), NOT_ON_TIMER, ADC_Channel10, EXTERNAL_INT_NONE }, // ADC/AIN[10]

  // 20..21 I2C pins (SDA/SCL and also EDBG:SDA/SCL)
  // ----------------------
  { PORTA, 22, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TC4_CH0, No_ADC_Channel, EXTERNAL_INT_NONE }, // SDA: SERCOM3/PAD[0]
  { PORTA, 23, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TC4_CH1, No_ADC_Channel, EXTERNAL_INT_NONE }, // SCL: SERCOM3/PAD[1]

  // 22..24 - SPI pins (ICSP:MISO,SCK,MOSI)
  // ----------------------
  { PORTA, 12, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_ALT), (PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_12 }, // MISO: SERCOM4/PAD[0]
  { PORTB, 10, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_ALT), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM), TC5_CH0, No_ADC_Channel, EXTERNAL_INT_NONE }, // MOSI: SERCOM4/PAD[2]
  { PORTB, 11, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_ALT), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM), TC5_CH1, No_ADC_Channel, EXTERNAL_INT_NONE }, // SCK: SERCOM4/PAD[3]

  // 25..26 - RX/TX LEDS (PB03/PA27)
  // --------------------
  { PORTB,  3, PIO_OUTPUT, PER_ATTR_DRIVE_STRONG, PIN_ATTR_OUTPUT, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // used as output only
  { PORTA, 27, PIO_OUTPUT, PER_ATTR_DRIVE_STRONG, PIN_ATTR_OUTPUT, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // used as output only

  // 27..29 - USB
  // --------------------
  { PORTA, 28, PIO_MULTI, PER_ATTR_DRIVE_STRONG, PIN_ATTR_DIGITAL, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // USB Host enable
  { PORTA, 24, PIO_COM, PER_ATTR_NONE, PIN_ATTR_COM, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // USB/DM
  { PORTA, 25, PIO_COM, PER_ATTR_NONE, PIN_ATTR_COM, NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // USB/DP

  // 30..41 - EDBG
  // ----------------------
  // 30/31 - EDBG/UART
  { PORTB, 22, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_ALT), (PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM), NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // TX: SERCOM5/PAD[2]
  { PORTB, 23, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_ALT), (PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM), NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE }, // RX: SERCOM5/PAD[3]

	// 32/33 I2C (SDA/SCL and also EDBG:SDA/SCL). These are duplicate entries of pins 20 and 21
	{ PORTA, 22, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TC4_CH0, No_ADC_Channel, EXTERNAL_INT_6 }, // SDA: SERCOM3/PAD[0]
	{ PORTA, 23, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TC4_CH1, No_ADC_Channel, EXTERNAL_INT_7 }, // SCL: SERCOM3/PAD[1]

	// 34..37 - EDBG/SPI. These are duplicate entries of pins 12, 11, 10, and 13.
	{ PORTA, 19, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_ALT|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TCC0_CH3, No_ADC_Channel, EXTERNAL_INT_3 }, // MISO: SERCOM1/PAD[3]
	{ PORTA, 16, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TCC2_CH0, No_ADC_Channel, EXTERNAL_INT_0 }, // MOSI: SERCOM1/PAD[0]
	{ PORTA, 18, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TC3_CH0, No_ADC_Channel, EXTERNAL_INT_2 }, // SS: SERCOM1/PAD[2]
	{ PORTA, 17, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TCC2_CH1, No_ADC_Channel, EXTERNAL_INT_1 }, // SCK: SERCOM1/PAD[1]

  // 38..41 - EDBG/Digital
  { PORTA, 13, PIO_MULTI, (PER_ATTR_DRIVE_STRONG), (PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT), NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_13 }, // EIC/EXTINT[13] PIN_ATN

	// 39..41 These are duplicate entries of pins 7, 8, and 9 so that AREF and DAC are still available on pins 42 and 43
	{ PORTA, 21, PIO_MULTI, (PER_ATTR_DRIVE_STRONG), (PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT|PIN_ATTR_COM), NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_5 },
	{ PORTA,  6, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), TCC1_CH0, ADC_Channel6, EXTERNAL_INT_NONE }, // TCC1/WO[0] ADC/AIN[6]
	{ PORTA,  7, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_COM), TCC1_CH1, ADC_Channel7, EXTERNAL_INT_NONE }, // TCC1/WO[1] ADC/AIN[7]

  // 42 (AREF)
  { PORTA,  3, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_ADC|PIN_ATTR_REF|PIN_ATTR_DIGITAL), NOT_ON_TIMER, ADC_Channel1, EXTERNAL_INT_NONE }, // VREF ADC/AIN[1]

	// 43 - Alternate use of A0 (DAC output). This is a duplicate entry of pin 
	{ PORTA,  2, PIO_MULTI, PER_ATTR_DRIVE_STRONG, (PIN_ATTR_ADC|PIN_ATTR_DAC|PIN_ATTR_DIGITAL), NOT_ON_TIMER, ADC_Channel0, EXTERNAL_INT_NONE }, // ADC/AIN[0] / DAC
} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TC3, TC4, TC5 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;

#if defined(ARDUINO_UART_ONLY) || defined(ARDUINO_CDC_HID_UART) || defined(ARDUINO_CDC_UART) || defined(ARDUINO_HID_UART)
Uart Serial1( SERCOM_INSTANCE_SERIAL1, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;
Uart Serial( SERCOM_INSTANCE_SERIAL, PIN_SERIAL_RX, PIN_SERIAL_TX, PAD_SERIAL_RX, PAD_SERIAL_TX ) ;
void SERCOM0_Handler()
{
  Serial1.IrqHandler();
}

void SERCOM5_Handler()
{
  Serial.IrqHandler();
}
#endif

