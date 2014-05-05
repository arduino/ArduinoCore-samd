/*
  Copyright (c) 2014 Arduino.  All right reserved.

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
 *
 * + Pin number +  ZERO Board pin  |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+------------------------------
 * |            | Digital Low      |        |                 |
 * +------------+------------------+--------+-----------------+------------------------------
 * | 0          | 0 -> RX          |  PA10  |                 | EIC/EXTINT[10] ADC/AIN[18] *SERCOM0/PAD[2] TCC1/WO[0] TCC0/WO[2]
 * | 1          | 1 <- TX          |  PA11  |                 | EIC/EXTINT[11] ADC/AIN[19] *SERCOM0/PAD[3] SERCOM2/PAD[3] TCC1/WO[1] TCC0/WO[3]
 * | 2          | ~2               |  PA08  |                 | EIC/NMI ADC/AIN[16] SERCOM0/PAD[0] SERCOM2/PAD[0] *TCC0/WO[0] TCC1/WO[2]
 * | 3          | ~3               |  PA09  |                 | EIC/EXTINT[9] ADC/AIN[17] SERCOM0/PAD[1] SERCOM2/PAD[1] *TCC0/WO[1] TCC1/WO[3]
 * | 4          | ~4               |  PA14  |                 | EIC/EXTINT[14] SERCOM2/PAD[2] SERCOM4/PAD[2] *TC3/WO[0] TCC0/WO[4]
 * | 5          | ~5               |  PA15  |                 | EIC/EXTINT[15] SERCOM2/PAD[3] SERCOM4/PAD[3] *TC3/WO[1] TCC0/WO[5]
 * | 6          | ~6               |  PA20  |                 | EIC/EXTINT[4] SERCOM5/PAD[2] SERCOM3/PAD[2] TC7/WO[0] *TCC0/WO[6]
 * | 7          | ~7               |  PA21  |                 | EIC/EXTINT[5] SERCOM5/PAD[3] SERCOM3/PAD[3] TC7/WO[1] *TCC0/WO[7]
 * +------------+------------------+--------+-----------------+------------------------------
 * |            | Digital High     |        |                 |
 * +------------+------------------+--------+-----------------+------------------------------
 * | 8          | ~8               |  PA06  |                 | EIC/EXTINT[6] ADC/AIN[6] AC/AIN[2] SERCOM0/PAD[2] *TCC1/WO[0]
 * | 9          | ~9               |  PA07  |                 | EIC/EXTINT[7] ADC/AIN[7] AC/AIN[3] SERCOM0/PAD[3] *TCC1/WO[1]
 * | 10         | ~10              |  PA18  |                 | EIC/EXTINT[2] SERCOM1/PAD[2] SERCOM3/PAD[2] *TC3/WO[0] TCC0/WO[2]
 * | 11         | ~11              |  PA16  |                 | EIC/EXTINT[0] SERCOM1/PAD[0] SERCOM3/PAD[0] *TCC2/WO[0] TCC0/WO[6]
 * | 12         | ~12              |  PA19  |                 | EIC/EXTINT[3] SERCOM1/PAD[3] SERCOM3/PAD[3] *TC3/WO[1] TCC0/WO[3]
 * | 13         | ~13              |  PA17  | LED             | EIC/EXTINT[1] SERCOM1/PAD[1] SERCOM3/PAD[1] *TCC2/WO[1] TCC0/WO[7]
 * | 14         | GND              |        |                 |
 * | 15         | AREF             |  PA03  |                 | *DAC/VREFP
 * | 16         | SDA              |  PA22  |                 | EIC/EXTINT[6] *SERCOM3/PAD[0] SERCOM5/PAD[0] TC4/WO[0] TCC0/WO[4]
 * | 17         | SCL              |  PA23  |                 | EIC/EXTINT[7] *SERCOM3/PAD[1] SERCOM5/PAD[1] TC4/WO[1] TCC0/WO[5]
 * +------------+------------------+--------+-----------------+------------------------------
 * |            |SPI (Legacy ICSP) |        |                 |
 * +------------+------------------+--------+-----------------+------------------------------
 * | 18         | 1                |  PA12  | MISO            | EIC/EXTINT[12] SERCOM2/PAD[0] *SERCOM4/PAD[0] TCC2/WO[0] TCC0/WO[6]
 * | 19         | 2                |        | 5V0             |
 * | 20         | 3                |  PB11  | SCK             | EIC/EXTINT[11] *SERCOM4/PAD[3] TC5/WO[1] TCC0/WO[5]
 * | 21         | 4                |  PB10  | MOSI            | EIC/EXTINT[10] *SERCOM4/PAD[2] TC5/WO[0] TCC0/WO[4]
 * | 22         | 5                |        | RESET           |
 * | 23         | 6                |        | GND             |
 * +------------+------------------+--------+-----------------+------------------------------
 * |            | Analog Connector |        |                 |
 * +------------+------------------+--------+-----------------+------------------------------
 * | 24         | A0               |  PA02  |                 | EIC/EXTINT[2] *DAC/VOUT
 * | 25         | A1               |  PB08  |                 | EIC/EXTINT[8] *ADC/AIN[2] SERCOM4/PAD[0] TC4/WO[0]
 * | 26         | A2               |  PB09  |                 | EIC/EXTINT[9] *ADC/AIN[3] SERCOM4/PAD[1] TC4/WO[1]
 * | 27         | A3               |  PA04  |                 | EIC/EXTINT[4] *ADC/AIN[4] AC/AIN[0] SERCOM0/PAD[0] TCC0/WO[0]
 * | 28         | A4               |  PA05  |                 | EIC/EXTINT[5] *ADC/AIN[5] AC/AIN[1] SERCOM0/PAD[1] TCC0/WO[1]
 * | 29         | A5               |  PB02  |                 | EIC/EXTINT[2] *ADC/AIN[10] SERCOM5/PAD[0] TC6/WO[0]
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
 * |            |32.768KHz Crystal |        |                 |
 * +------------+------------------+--------+-----------------+------------------------------
 * |            |                  |  PA00  | XIN32           | EXTINT[0] SERCOM1/PAD[0] TCC2/WO[0]
 * |            |                  |  PA01  | XOUT32          | EXTINT[1] SERCOM1/PAD[1] TCC2/WO[1]
 * +------------+------------------+--------+-----------------+------------------------------
 */

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[]=
{
  // 0 .. 19 - Digital pins
  // ----------------------
  // 0/1 - SERCOM/UART (Serial)
  { PORTA, 10, PIO_SERCOM, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // RX: SERCOM0/PAD[2]
  { PORTA, 11, PIO_SERCOM, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // TX: SERCOM0/PAD[3]

	// 2..12
	{ PORTA, 8, PIO_TIMER, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // TCC0/WO[0]
	{ PORTA, 9, PIO_TIMER, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // TCC0/WO[1]
	{ PORTA, 14, PIO_TIMER, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // TC3/WO[0]
	{ PORTA, 15, PIO_TIMER, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // TC3/WO[1]
	{ PORTA, 20, PIO_TIMER_ALT, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // TCC0/WO[6]
	{ PORTA, 21, PIO_TIMER_ALT, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // TCC0/WO[7]
	{ PORTA, 6, PIO_TIMER, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // TCC1/WO[0]
	{ PORTA, 7, PIO_TIMER, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // TCC1/WO[1]
	{ PORTA, 18, PIO_TIMER, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // TC3/WO[0]
	{ PORTA, 16, PIO_TIMER, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // TCC2/WO[0]
	{ PORTA, 19, PIO_TIMER, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // TC3/WO[1]

	// 13 (LED)
	{ PORTA, 17, PIO_TIMER, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // TCC2/WO[1]

	// 14 (GND)
	{ NOT_A_PORT, 0, PIO_NOT_A_PIN, 0, 0, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },

	// 15 (AREF)
	{ PORTA, 3, PIO_ANALOG, PIO_DEFAULT, PIN_ATTR_ANALOG, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // DAC/VREFP

	// 16..17 I2C (SDA/SCL)
  { PORTA, 22, PIO_SERCOM, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // SDA: SERCOM3/PAD[0]
  { PORTA, 23, PIO_SERCOM, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // SCL: SERCOM3/PAD[1]

	// 18..23 SPI (ICSP:MISO,SCK,MOSI)
  { PORTA, 12, PIO_SERCOM, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // MISO: SERCOM4/PAD[0]
  { NOT_A_PORT, 0, PIO_NOT_A_PIN, 0, 0, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // 5V0
  { PORTB, 11, PIO_SERCOM, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // SCK: SERCOM4/PAD[3]
  { PORTB, 10, PIO_SERCOM, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // MOSI: SERCOM4/PAD[2]
  { NOT_A_PORT, 0, PIO_NOT_A_PIN, 0, 0, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // RESET
  { NOT_A_PORT, 0, PIO_NOT_A_PIN, 0, 0, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // GND

  // 24..29 - Analog pins
	// --------------------
	// 24 - A0 (DAC output)
	{ PORTA, PORT_PA02B_DAC_VOUT, PIO_ANALOG, PIO_DEFAULT, PIN_ATTR_ANALOG, DAC0, A0, NOT_ON_PWM, NOT_ON_TIMER }, // DAC/VOUT

	// 25..29 - A1-A5
  { PORTB, 8, PIO_ANALOG, PIO_DEFAULT, PIN_ATTR_ANALOG, ADC2, A1, NOT_ON_PWM, NOT_ON_TIMER }, // ADC/AIN[2]
  { PORTB, 9, PIO_ANALOG, PIO_DEFAULT, PIN_ATTR_ANALOG, ADC3, A2, NOT_ON_PWM, NOT_ON_TIMER }, // ADC/AIN[3]
  { PORTA, 4, PIO_ANALOG, PIO_DEFAULT, PIN_ATTR_ANALOG, ADC4, A3, NOT_ON_PWM, NOT_ON_TIMER }, // ADC/AIN[4]
  { PORTA, 5, PIO_ANALOG, PIO_DEFAULT, PIN_ATTR_ANALOG, ADC5, A4, NOT_ON_PWM, NOT_ON_TIMER }, // ADC/AIN[5]
  { PORTA, 2, PIO_ANALOG, PIO_DEFAULT, PIN_ATTR_ANALOG, ADC10, A5, NOT_ON_PWM, NOT_ON_TIMER }, // ADC/AIN[10]

	// 30..31 - RX/TX LEDS (PB03/PA27)
	{ PORTB, 3, PIO_DIGITAL, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // use as pure output
	{ PORTA, 27, PIO_DIGITAL, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // use as pure output

	// 32..33 - USB
	{ PORTA, 28, PIO_COM, PIO_DEFAULT, 0, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // USB/SOF
	{ PORTA, 24, PIO_COM, PIO_DEFAULT, 0, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // USB/DM
	{ PORTA, 25, PIO_COM, PIO_DEFAULT, 0, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // USB/DP
} ;
