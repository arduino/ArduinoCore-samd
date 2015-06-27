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
  Modified 21 June 2015 by Justin Mattair
     for the MattairTech MT-D21E board (www.mattairtech.com)
*/


/*  Pins descriptions for the MattairTech MT-D21E
 * 
 * | PCB Pin | Arduino Pin Number |    Silkscreen    |  PIN   |   Alt. Function   | Comments (* is for default peripheral in use, () means unavailable)
 * +---------+--------------------+------------------+--------+-------------------+-------------------------------------------------
 * | 0       | --                 | A0               |  PA00  | Xin32             | *Xin32
 * | 1       | --                 | A1               |  PA01  | Xout32            | *Xout32
 * | 2       | 2                  | A2               |  PA02  |                   | EIC/EXTINT[2] *ADC/AIN[0] PTC/Y[0] DAC/VOUT
 * | 3       | 3                  | A3               |  PA03  | REFA              | EIC/EXTINT[3] REF/ADC/VREFA REF/DAC/VREFP *ADC/AIN[1] PTC/Y[1]
 * | 4       | 4                  | A4               |  PA04  | REFB              | EIC/EXTINT[4] REF/ADC/VREFP *ADC/AIN[4] AC/AIN[0] PTC/Y[2] (SERCOM0/PAD[0]) (TCC0/WO[0])
 * | 5       | 5                  | A5               |  PA05  |                   | EIC/EXTINT[5] *ADC/AIN[5] AC/AIN[1] PTC/Y[3] (SERCOM0/PAD[1]) (TCC0/WO[1])
 * | 6       | 6                  | A6               |  PA06  |                   | EIC/EXTINT[6] *ADC/AIN[6] AC/AIN[2] PTC/Y[4] (SERCOM0/PAD[2]) (TCC1/WO[0])
 * | 7       | 7                  | A7               |  PA07  | Voltage Divider   | EIC/EXTINT[7] *ADC/AIN[7] AC/AIN[3] PTC/Y[5] (SERCOM0/PAD[3]) (TCC1/WO[1])
 * | 8       | 8                  | A8               |  PA08  |                   | EIC/NMI ADC/AIN[16] PTC/X[0] (SERCOM0/PAD[0]) (SERCOM2/PAD[0]) *TCC0/WO[0] (TCC1/WO[2])
 * | 9       | 9                  | A9               |  PA09  |                   | EIC/EXTINT[9] ADC/AIN[17] PTC/X[1] (SERCOM0/PAD[1]) (SERCOM2/PAD[1]) *TCC0/WO[1] (TCC1/WO[3])
 * | 10      | 10                 | A10              |  PA10  | TX                | EIC/EXTINT[10] ADC/AIN[18] PTC/X[2] *SERCOM0/PAD[2] (SERCOM2/PAD[2]) (TCC1/WO[0]) TCC0/WO[2]
 * | 11      | 11                 | A11              |  PA11  | RX                | EIC/EXTINT[11] ADC/AIN[19] PTC/X[3] *SERCOM0/PAD[3] (SERCOM2/PAD[3]) (TCC1/WO[1]) TCC0/WO[3]
 * | 12      | 14                 | A14              |  PA14  | Xin, HOST_ENABLE  | EIC/EXTINT[14] SERCOM2/PAD[2] (SERCOM4/PAD[2]) *TC3/WO[0] (TCC0/WO[4]) Xin, HOST_ENABLE
 * | 13      | 15                 | A15              |  PA15  | Xout, ATN         | EIC/EXTINT[15] SERCOM2/PAD[3] (SERCOM4/PAD[3]) *TC3/WO[1] (TCC0/WO[5]) Xout, ATN
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
 * | 26      | 16                 | A16              |  PA16  | I2C/SDA w/pullup  | EIC/EXTINT[0] PTC/X[4] *SERCOM1/PAD[0] (SERCOM3/PAD[0]) TCC2/WO[0] (TCC0/WO[6])
 * | 27      | 17                 | A17              |  PA17  | I2C/SCL w/pullup  | EIC/EXTINT[1] PTC/X[5] *SERCOM1/PAD[1] (SERCOM3/PAD[1]) TCC2/WO[1] (TCC0/WO[7])
 * | 28      | 18                 | A18              |  PA18  | SPI MOSI          | EIC/EXTINT[2] PTC/X[6] (SERCOM1/PAD[2]) *SERCOM3/PAD[2] (TC3/WO[0]) (TCC0/WO[2])
 * | 29      | 19                 | A19              |  PA19  | SPI SCK           | EIC/EXTINT[3] PTC/X[7] (SERCOM1/PAD[3]) *SERCOM3/PAD[3] (TC3/WO[1]) (TCC0/WO[3])
 * | 30      | 22                 | A22              |  PA22  | SPI MISO          | EIC/EXTINT[6] PTC/X[10] *SERCOM3/PAD[0] (SERCOM5/PAD[0]) TC4/WO[0] (TCC0/WO[4])
 * | 31      | 23                 | A23              |  PA23  | SPI SS            | EIC/EXTINT[7] PTC/X[11] *SERCOM3/PAD[1] (SERCOM5/PAD[1]) TC4/WO[1] (TCC0/WO[5])
 * | 32      | 27                 | A27              |  PA27  | Button A          | EIC/EXTINT[15] *Button A
 * | 33      | 28                 | A28              |  PA28  | LED               | EIC/EXTINT[8] *LED
 * | 34      | --                 | NC               |  ----  |                   | Not Connected
 * | 35      | 30                 | A30              |  PA30  | SWD CLK           | EIC/EXTINT[10] (SERCOM1/PAD[2]) *TCC1/WO[0] SWD CLK
 * | 36      | 31                 | A31              |  PA31  | Button B / SWD IO | EIC/EXTINT[11] (SERCOM1/PAD[3]) *TCC1/WO[1] Button B SWD IO
 * | 37      | --                 | NC               |  ----  |                   | Not Connected
 * | 38      | --                 | NC               |  ----  |                   | Not Connected
 * | 39      | --                 | RST              |  ----  |                   | Reset
 * +---------+--------------------+------------------+--------+-------------------+-------------------------------------------------
 * 
 * You may use Arduino pin numbers ranging from 0 to 31. The Arduino pin corresponds to the silkscreen (without the 'A').
 * For example, use pinMode(28, OUTPUT) to set the LED pin (marked as A28) as an output.
 * However, the following Arduino pin numbers are not mapped to a physical pin: 0, 1, 12, 13, 20, 21, 24, 25, 26, and 29.
 * Note that pins 0 and 1 are used by the 32.768KHz crystal which in turn is used by the Arduino core (the 16MHz crystal is unused by Arduino).
 * Note also that pins 24 and 25 are in use by USB (USB_NEGATIVE and USB_POSITIVE).
 */


#include "variant.h"

/*
 * Pins descriptions
 */
const PinDescription g_APinDescription[]=
{
	// 0..1 are unused (pins in use by 32.768KHz crystal, which in turn is used by the Arduino core)
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Unused
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Unused
	
	// 2..9 - Analog capable pins (DAC avalable on 2)
	{ PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // ADC/AIN[0] / DAC
	{ PORTA,  3, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel1, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // ADC/AIN[1]
	{ PORTA,  4, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT), ADC_Channel4, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 }, // ADC/AIN[4]
	{ PORTA,  5, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT), ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, // ADC/AIN[5]
	{ PORTA,  6, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_DIGITAL), ADC_Channel6, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // ADC/AIN[6]
	{ PORTA,  7, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_DIGITAL), ADC_Channel7, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // ADC/AIN[7]
	{ PORTA,  8, PIO_TIMER, (PIN_ATTR_ANALOG|PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), ADC_Channel16, PWM0_CH0, TCC0_CH0, EXTERNAL_INT_NMI }, // TCC0/WO[0]
	{ PORTA,  9, PIO_TIMER, (PIN_ATTR_ANALOG|PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), ADC_Channel17, PWM0_CH1, TCC0_CH1, EXTERNAL_INT_9 }, // TCC0/WO[1]
	
	// 10..11 - SERCOM/UART (Serial1) or Analog or Digital functions
	{ PORTA, 10, PIO_SERCOM, (PIN_ATTR_ANALOG|PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel18, PWM0_CH2, TCC0_CH2, EXTERNAL_INT_NONE }, // TX: SERCOM0/PAD[2], PIO_TIMER_ALT
	{ PORTA, 11, PIO_SERCOM, (PIN_ATTR_ANALOG|PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel19, PWM0_CH3, TCC0_CH3, EXTERNAL_INT_NONE }, // RX: SERCOM0/PAD[3], PIO_TIMER_ALT
	
	// 12..13 pins don't exist
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Unused
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Unused
	
	// 14..15 - Digital functions
	{ PORTA, 14, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), No_ADC_Channel, PWM3_CH0, TC3_CH0, EXTERNAL_INT_14 }, // TC3/WO[0], HOST_ENABLE
	{ PORTA, 15, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM3_CH1, TC3_CH1, EXTERNAL_INT_NONE }, // TC3/WO[1], ATN
	
	// 16..17 I2C pins (SDA/SCL) or Digital functions
	{ PORTA, 16, PIO_SERCOM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), No_ADC_Channel, PWM2_CH0, TCC2_CH0, EXTERNAL_INT_0 }, // SDA: SERCOM1/PAD[0]
	{ PORTA, 17, PIO_SERCOM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), No_ADC_Channel, PWM2_CH1, TCC2_CH1, EXTERNAL_INT_1 }, // SCL: SERCOM1/PAD[1]
	
	// 18..23 - SPI Pins or Digital functions (pins 20..21 do not exist)
	{ PORTA, 18, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // SPI MOSI: SERCOM3/PAD[2], PIO_SERCOM_ALT
	{ PORTA, 19, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 }, // SPI SCK: SERCOM3/PAD[3], PIO_SERCOM_ALT
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Unused
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Unused
//	{ PORTA, 22, PIO_SERCOM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), No_ADC_Channel, PWM4_CH0, TC4_CH0, EXTERNAL_INT_6 }, // SPI MISO: SERCOM3/PAD[0]
//	{ PORTA, 23, PIO_SERCOM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), No_ADC_Channel, PWM4_CH1, TC4_CH1, EXTERNAL_INT_7 }, // SPI SS: SERCOM3/PAD[1]
	{ PORTA, 22, PIO_SERCOM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, // SPI MISO: SERCOM3/PAD[0]
	{ PORTA, 23, PIO_SERCOM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, // SPI SS: SERCOM3/PAD[1]
	
	// 24..26 are unused (25 and 26 in use by USB_NEGATIVE and USB_POSITIVE, pin 26 does not exist)
	{ PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DM
	{ PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DP
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Unused
	
	// 27..29 Button A and LED (pin 29 does not exist)
	{ PORTA, 27, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 }, // Button A
	{ PORTA, 28, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 }, // LED
	{ NOT_A_PORT,  0, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Unused
	
	// 30..31 Digital functions / Debug interface (SWD CLK and SWD IO)
	{ PORTA, 30, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), No_ADC_Channel, PWM1_CH0, TCC1_CH0, EXTERNAL_INT_10 }, // TCC1/WO[0] / SWD CLK
	{ PORTA, 31, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER|PIN_ATTR_EXTINT), No_ADC_Channel, PWM1_CH1, TCC1_CH1, EXTERNAL_INT_11 }, // TCC1/WO[1] / SWD IO
	
	// 32 - Alternate use of pin 2 (DAC output)
	//{ PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VOUT
} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TC3, TC4, TC5 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;

Uart Serial1( &sercom0, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

void SERCOM0_Handler()
{
  Serial1.IrqHandler();
}

