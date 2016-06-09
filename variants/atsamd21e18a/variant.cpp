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
/* NOTE: PA14/PA15 are used by external clock source.
 * PA15 may be pulled low to stay in SAM-BA bootloader mode (to upload another sketch via BOSSA utility)
 * PA30 (SWCLK)/PA31 (SWDIO) are reserved for SWD JTAG.
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * + Pin number +  SAMD21E18A pin  |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | Digital Low      |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 0          | 0 -> RX          |  PA11  | RX              | EIC/EXTINT[11] ADC/AIN[19]           PTC/X[3] *SERCOM0/PAD[3]  SERCOM2/PAD[3]  TCC0/WO[3]  TCC1/WO[1]
 * | 1          | 1 <- TX          |  PA10  | TX              | EIC/EXTINT[10] ADC/AIN[18]           PTC/X[2] *SERCOM0/PAD[2]                  TCC0/WO[2]  TCC1/WO[0]
 * | 2          | 2                |  PA00  |                 | EIC/EXTINT[0]                                    SERCOM4/PAD[2]  TC3/WO[0]   TCC0/WO[4] (32KHz config was EIC/EXTINT[0] SERCOM1/PAD[0] TCC2/WO[0])
 * | 3          | ~3               |  PA09  | Green           | EIC/EXTINT[9]  ADC/AIN[17]           PTC/X[1]  SERCOM0/PAD[1]  SERCOM2/PAD[1] *TCC0/WO[1]  TCC1/WO[3]
 * | 4          | ~4               |  PA08  | Red             | EIC/NMI        ADC/AIN[16]           PTC/X[0]  SERCOM0/PAD[0]  SERCOM2/PAD[0] *TCC0/WO[0]  TCC1/WO[2]
 * | 5          | 5                |  PA27  |                 | EIC/EXTINT[15]
 * | 6          | 6                |  PA01  |                 | EIC/EXTINT[1]                                                  SERCOM1/PAD[1] (32KHz config was EIC/EXTINT[1] SERCOM1/PAD[1] TCC2/WO[1])
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | Digital High     |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 7          | ~7               |  PA18  | SS              | EIC/EXTINT[2]                        PTC/X[6] +SERCOM1/PAD[2]  SERCOM3/PAD[2] *TC3/WO[0]    TCC0/WO[2]
 * | 8          | ~8               |  PA16  | MOSI            | EIC/EXTINT[0]                        PTC/X[4] +SERCOM1/PAD[0]  SERCOM3/PAD[0] *TCC2/WO[0]   TCC0/WO[6]
 * | 9          | ~9               |  PA19  | MISO            | EIC/EXTINT[3]                        PTC/X[7] +SERCOM1/PAD[3]  SERCOM3/PAD[3]  TC3/WO[1]   *TCC0/WO[3]
 * | 10         | ~10              |  PA17  | SCK/Blue        | EIC/EXTINT[1]                        PTC/X[5] +SERCOM1/PAD[1]  SERCOM3/PAD[1] *TCC2/WO[1]   TCC0/WO[7]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | Analog Connector |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 11         | A0/DAC           |  PA02  | A0              | EIC/EXTINT[2] *ADC/AIN[0]  DAC/VOUT  PTC/Y[0]
 * | 12         | A1               |  PA06  | A1              | EIC/EXTINT[6] *ADC/AIN[6]            PTC/Y[4]  SERCOM0/PAD[0]                  TC1/WO[0]
 * | 13         | A2               |  PA07  | A2              | EIC/EXTINT[9] *ADC/AIN[3]            PTC/Y[5]  SERCOM0/PAD[3]                  TC1/WO[1]
 * | 14         | A3               |  PA04  | A3              | EIC/EXTINT[4] *ADC/AIN[4]  AC/AIN[0] PTC/Y[2]  SERCOM0/PAD[0]                  TCC0/WO[0]
 * | 15         | A4               |  PA05  | A4              | EIC/EXTINT[5] *ADC/AIN[5]  AC/AIN[1] PTC/Y[3]  SERCOM0/PAD[1]                  TCC0/WO[1]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | Wire             |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 16         | SDA              |  PA22  | SDA             | EIC/EXTINT[6]                        PTC/X[10] *SERCOM3/PAD[0] SERCOM5/PAD[0] TC4/WO[0] TCC0/WO[4]
 * | 17         | SCL              |  PA23  | SCL             | EIC/EXTINT[7]                        PTC/X[11] *SERCOM3/PAD[1] SERCOM5/PAD[1] TC4/WO[1] TCC0/WO[5]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            |SPI (Legacy ICSP) |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 18         | 1                |  PA19  | MISO            | EIC/EXTINT[3] *SERCOM1/PAD[3] SERCOM3/PAD[3] TC3/WO[1]  TCC0/WO[3]
 * |            | 2                |        | 3V0             |
 * | 19         | 4                |  PA16  | MOSI            | EIC/EXTINT[0] *SERCOM1/PAD[0] SERCOM3/PAD[0] TCC2/WO[0] TCC0/WO[6]
 * | 20         |                  |  PA18  | SS              | EIC/EXTINT[2] *SERCOM1/PAD[2] SERCOM3/PAD[2] TC3/WO[0]  TCC0/WO[2]
 * | 21         | 3                |  PA17  | SCK             | EIC/EXTINT[1] *SERCOM1/PAD[1] SERCOM3/PAD[1] TCC2/WO[1] TCC0/WO[7]
 * |            | 5                |        | RESET           |
 * |            | 6                |        | GND             |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | USB              |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 22         |                  |  PA28  | USB_HOST_ENABLE | EIC/EXTINT[8]
 * | 23         |                  |  PA24  | USB_NEGATIVE    | *USB/DM
 * | 24         |                  |  PA25  | USB_POSITIVE    | *USB/DP
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            |                  |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | GND              |        |                 |
 * | 25         | AREF             |  PA03  | AREF            | EIC/EXTINT[3] *[ADC|DAC]/VREFA ADC/AIN[1] PTC/Y[1]
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
  // 0/1 - SERCOM/UART (Serial)
  { PORTA, 11, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, // RX: SERCOM0/PAD[3]
  { PORTA, 10, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // TX: SERCOM0/PAD[2]

  // 2..9
  // Digital Low
  { PORTA,  0, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },
  { PORTA,  9, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH1, TCC0_CH1, EXTERNAL_INT_9 }, // TCC0/WO[1] (LED Green)
  { PORTA,  8, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH0, TCC0_CH0, EXTERNAL_INT_NMI },  // TCC0/WO[0] (LED Red)
  { PORTA, 27, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },
  { PORTA,  1, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 },

  // Digital High
  { PORTA, 18, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM3_CH0, TC3_CH0, EXTERNAL_INT_2 }, // TC3/WO[0]
  { PORTA, 16, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM2_CH0, TCC2_CH0, EXTERNAL_INT_0 }, // TCC2/WO[0]
  { PORTA, 19, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), No_ADC_Channel, PWM0_CH3, TCC0_CH3, EXTERNAL_INT_3 }, // TCC0/WO[3]

  // 10 (LED)
  { PORTA, 17, PIO_PWM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM2_CH1, TCC2_CH1, EXTERNAL_INT_1 }, // TCC2/WO[1]

  // 11..15 - Analog pins
  // --------------------
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // ADC/AIN[0]
  { PORTA,  6, PIO_ANALOG, (PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel2, PWM4_CH0, TC4_CH0, EXTERNAL_INT_6 }, // ADC/AIN[6]
  { PORTA,  7, PIO_ANALOG, (PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel3, PWM4_CH1, TC4_CH1, EXTERNAL_INT_7 }, // ADC/AIN[7]

  { PORTA,  4, PIO_ANALOG, 0, ADC_Channel4, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 }, // ADC/AIN[4]
  { PORTA,  5, PIO_ANALOG, 0, ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, // ADC/AIN[5]

  // 16..17 I2C pins (SDA/SCL and also EDBG:SDA/SCL)
  // ----------------------
  { PORTA, 22, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, // SDA: SERCOM3/PAD[0]
  { PORTA, 23, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, // SCL: SERCOM3/PAD[1]

  // 18..21 - SPI pins (ICSP:MISO,MOSI,SS,SCK)
  // ----------------------
  { PORTA, 19, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // MISO: SERCOM1/PAD[3]
  { PORTA, 16, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // MOSI: SERCOM1/PAD[0]
  { PORTA, 18, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SS: SERCOM1/PAD[2]
  { PORTA, 17, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SCK: SERCOM1/PAD[1]

  // 22..24 - USB
  // --------------------
  { PORTA, 28, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB Host enable
  { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DM
  { PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DP

  // 25 (AREF)
  { PORTA, 3, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VREFP

  // ----------------------
  // 26 - Alternate use of A0 (DAC output)
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 } // DAC/VOUT
} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TC3, TC4, TC5 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;

// Uart Serial1( &sercom0, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;
// Uart Serial( &sercom5, PIN_SERIAL_RX, PIN_SERIAL_TX, PAD_SERIAL_RX, PAD_SERIAL_TX ) ;
Uart Serial( &sercom0, PIN_SERIAL_RX, PIN_SERIAL_TX, PAD_SERIAL_RX, PAD_SERIAL_TX ) ;
// void SERCOM0_Handler()
// {
//   Serial1.IrqHandler();
// }

// void SERCOM5_Handler()
// {
//   Serial.IrqHandler();
// }
void SERCOM0_Handler()
{
  Serial.IrqHandler();
}

