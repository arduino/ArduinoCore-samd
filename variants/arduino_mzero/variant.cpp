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


#include "variant.h"

/*
 * Pins descriptions
 */
const PinDescription g_APinDescription[]=
{
  // 0 .. 19 - Digital pins
  // ----------------------
  // 0/1 - SERCOM/UART (Serial1)
  { PORTA, 11, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, // RX: SERCOM0/PAD[3]
  { PORTA, 10, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // TX: SERCOM0/PAD[2]

  // 2..12
  { PORTA,  8, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH0, TCC0_CH0, EXTERNAL_INT_NONE }, // EXTERNAL_INT_NMI }, // TCC0/WO[0]
  { PORTA,  9, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH1, TCC0_CH1, EXTERNAL_INT_9 }, // TCC0/WO[1]
  { PORTA, 14, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM3_CH0, TC3_CH0, EXTERNAL_INT_14 }, // TC3/WO[0]
  { PORTA, 15, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM3_CH1, TC3_CH1, EXTERNAL_INT_15 }, // TC3/WO[1]
  { PORTA, 20, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH6, TCC0_CH6, EXTERNAL_INT_4 }, // TCC0/WO[6]
  { PORTA, 21, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH7, TCC0_CH7, EXTERNAL_INT_5 }, // TCC0/WO[7]
  { PORTA,  6, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM1_CH0, TCC1_CH0, EXTERNAL_INT_6 }, // TCC1/WO[0]
  { PORTA,  7, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM1_CH1, TCC1_CH1, EXTERNAL_INT_7 }, // TCC1/WO[1]
  { PORTA, 18, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM3_CH0, TC3_CH0, EXTERNAL_INT_2 }, // TC3/WO[0]
  { PORTA, 16, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM2_CH0, TCC2_CH0, EXTERNAL_INT_0 }, // TCC2/WO[0]
  { PORTA, 19, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM3_CH1, TC3_CH1, EXTERNAL_INT_3 }, // TC3/WO[1]

  // 13 (LED)
  { PORTA, 17, PIO_PWM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM2_CH1, NOT_ON_TIMER, EXTERNAL_INT_1 }, // TCC2/WO[1]

  // 14 (GND)
  { NOT_A_PORT, 0, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },

  // 15 (AREF)
  { PORTA, 3, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VREFP

  // 16..17 I2C pins (SDA/SCL and also EDBG:SDA/SCL)
  // ----------------------
  { PORTA, 22, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, // SDA: SERCOM3/PAD[0]
  { PORTA, 23, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, // SCL: SERCOM3/PAD[1]

  // 18..23 - SPI pins (ICSP:MISO,SCK,MOSI)
  // ----------------------
  { PORTA, 12, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 }, // MISO: SERCOM4/PAD[0]
  { NOT_A_PORT, 0, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // 5V0
  { PORTB, 11, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, // SCK: SERCOM4/PAD[3]
  { PORTB, 10, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // MOSI: SERCOM4/PAD[2]
  { NOT_A_PORT, 0, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // RESET
  { NOT_A_PORT, 0, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // GND

  // 24..29 - Analog pins
  // --------------------
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // ADC/AIN[0]
  { PORTB,  8, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel2, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 }, // ADC/AIN[2]
  { PORTB,  9, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel3, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 }, // ADC/AIN[3]
  { PORTA,  4, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel4, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 }, // ADC/AIN[4]
  { PORTA,  5, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, // ADC/AIN[5]
  { PORTB,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel10, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // ADC/AIN[10]

  // 30..31 - RX/TX LEDS (PB03/PA27)
  // --------------------
  { PORTB,  3, PIO_OUTPUT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // used as output only
  { PORTA, 27, PIO_OUTPUT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // used as output only

  // 32..34 - USB
  // --------------------
  { PORTA, 28, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB Host enable
  { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DM
  { PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DP

  // 35 .. 46 - EDBG
  // ----------------------
  // 35/36 - EDBG/UART
  { PORTB, 22, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // TX: SERCOM5/PAD[2]
  { PORTB, 23, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // RX: SERCOM5/PAD[3]

  // 37/38 I2C (SDA/SCL and also EDBG:SDA/SCL)
  { PORTA, 22, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SDA: SERCOM3/PAD[0]
  { PORTA, 23, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SCL: SERCOM3/PAD[1]

  // 39 .. 42 - EDBG/SPI
  { PORTA, 19, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // MISO: SERCOM1/PAD[3]
  { PORTA, 16, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // MOSI: SERCOM1/PAD[0]
  { PORTA, 18, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SS: SERCOM1/PAD[2]
  { PORTA, 17, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SCK: SERCOM1/PAD[1]

  // 43 .. 46 - EDBG/Digital
  { PORTA, 13, PIO_PWM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM0_CH5, NOT_ON_TIMER, EXTERNAL_INT_13 }, // EIC/EXTINT[13] *TCC2/WO[1] TCC0/WO[7]
  { PORTA, 21, PIO_PWM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM0_CH7, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Pin 7
  { PORTA,  6, PIO_PWM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM1_CH0, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Pin 8
  { PORTA,  7, PIO_PWM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM1_CH1, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Pin 9

 // ----------------------
 // 47 - Alternate use of A0 (DAC output)
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // DAC/VOUT
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
Uart Serial( &sercom5, PIN_SERIAL_RX, PIN_SERIAL_TX, PAD_SERIAL_RX, PAD_SERIAL_TX ) ;
void SERCOM0_Handler()
{
  Serial1.IrqHandler();
}

void SERCOM5_Handler()
{
  Serial.IrqHandler();
}

