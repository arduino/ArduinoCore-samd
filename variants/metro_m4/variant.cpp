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
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * + Pin number +  ZERO Board pin  |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | Digital Low      |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 0          | 0 -> RX          |  PA23  |                 | EIC/EXTINT[7]                             PTCXY[17] *SERCOM3/PAD[1]  SERCOM5/PAD[0]  TC4/WO[1]  TCC1/WO[7]
 * | 1          | 1 <- TX          |  PA22  |                 | EIC/EXTINT[6]                             PTCXY[16] *SERCOM3/PAD[0]  SERCOM5/PAD[1]  TC4/WO[0]  TCC1/WO[6]
 * | 2          | ~2               |  PA08  |                 | EIC/NMI         ADC0/AIN[8]               PTCXY[6]   SERCOM0/PAD[0]  SERCOM2/PAD[1]  TC0/WO[0]  *TCC1/WO[4]
 * | 3          | ~3               |  PA10  |                 | EIC/EXTINT[10]  ADC0/AIN[10]              PTCXY[8]   SERCOM0/PAD[2]  SERCOM2/PAD[2]  TC1/WO[0]  *TCC0/WO[2]
 * | 4          | ~4               |  PB12  |                 | EIC/EXTINT[12]                            PTCXY[26]  SERCOM4/PAD[0]                  TC4/WO[0]  *TCC3/WO[0]
 * | 5          | ~5               |  PB14  |                 | EIC/EXTINT[14]                            PTCXY[28]  SERCOM4/PAD[2]                 *TC5/WO[0]   TCC4/WO[0]
 * | 6          | ~6               |  PB15  |                 | EIC/EXTINT[15]                            PTCXY[29]  SERCOM4/PAD[3]                 *TC5/WO[1]   TCC4/WO[1]
 * | 7          | ~7               |  PA14  |                 | EIC/EXTINT[14]                                       SERCOM2/PAD[2]  SERCOM4/PAD[2]  TC3/WO[0]   TCC2/WO[0]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | Digital High     |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 8          | 8                |  PA16  |                 | EIC/EXTINT[0]                             PTCXY[10]  SERCOM1/PAD[0]   SERCOM3/PAD[1]  TC2/WO[2]   TCC1/WO[1]                 
 * | 9          | 9                |  PA17  |                 | EIC/EXTINT[7]                             PTCXY[11]  SERCOM1/PAD[1]   SERCOM3/PAD[0]  TC2/WO[1]   TCC1/WO[1]            
 * | 10         | ~10              |  PA18  |                 | EIC/EXTINT[2]                             PTCXY[12]  SERCOM1/PAD[2]   SERCOM3/PAD[2]  TC3/WO[0]   TCC1/WO[2]
 * | 11         | ~11              |  PA19  |                 | EIC/EXTINT[3]                             PTCXY[13]  SERCOM1/PAD[3]   SERCOM3/PAD[3]  TC3/WO[1]   TCC1/WO[3]
 * | 12         | ~12              |  PA20  |                 | EIC/EXTINT[4]                             PTCXY[14]  SERCOM5/PAD[2]   SERCOM3/PAD[2]              TCC1/WO[4]
 * | 13         | ~13              |  PA21  | LED             | EIC/EXTINT[5]                             PTCXY[15]  SERCOM5/PAD[3]   SERCOM3/PAD[3]              TCC1/WO[5]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | Analog Connector |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 14         | A0               |  PA02  | A0              | EIC/EXTINT[2] *ADC/AIN[0]  DAC[0]/VOUT   
 * | 15         | A1               |  PA05  | A1              | EIC/EXTINT[5] *ADC/AIN[5]  DAC[1]/VOUT                                SERCOM0/PAD[1]   TC0/WO[1]          
 * | 16         | A2               |  PA06  | A2              | EIC/EXTINT[6] *ADC/AIN[3]                 PTCXY[4]                    SERCOM4/PAD[1]   TC1/WO[0]
 * | 17         | A3               |  PA07  | A3              | EIC/EXTINT[7] *ADC/AIN[7]                 PTCXY[5]                    SERCOM0/PAD[3]   TC1/WO[1]  
 * | 18         | A4               |  PA11  | A4              | EIC/EXTINT[11] *ADC/AIN[11]               PTCXY[9]  SERCOM0/PAD[3]    SERCOM2/PAD[3]   TC1/WO[1]  TCC0/WO[3]  
 * | 19         | A5               |  PA04  | A5              | EIC/EXTINT[4] *ADC/AIN[4]                 PTCXY[3]                    SERCOM0/PAD[0]   TC0/WO[0]   
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | Wire             |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 20         | SDA              |  PB02  | SDA             | EIC/EXTINT[6]  ADC/AIN[14]                PTCXY[20]                   *SERCOM5/PAD[0]              TCC2/WO[2]
 * | 21         | SCL              |  PB03  | SCL             | EIC/EXTINT[3]  ADC/AIN[15]                PTCXY[21]                   *SERCOM5/PAD[1]       
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            |SPI (Legacy ICSP) |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 22         | 1                |  PA15  | MISO            | EIC/EXTINT[15]                                       SERCOM2/PAD[3]   *SERCOM4/PAD[3]   TC3/WO[1] TCC2/WO[1]
 * |            | 2                |        | 5V0             |
 * | 23         | 4                |  PA13  | MOSI            | EIC/EXTINT[13]                                       SERCOM2/PAD[1]   *SERCOM4/PAD[0]   TC2/WO[1] TCC0/WO[7]
 * | 24         | 3                |  PA12  | SCK             | EIC/EXTINT[12]                                       SERCOM2/PAD[0]   *SERCOM4/PAD[1]   TC2/WO[0] TCC0/WO[6]
 * |            | 5                |        | RESET           |
 * |            | 6                |        | GND             |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | LEDs             |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 25         |                  |  PB06  | RX              |
 * | 26         |                  |  PA27  | TX              |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | USB              |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 27         |                  |  PB07  | USB_HOST_ENABLE | EIC/EXTINT[7]
 * | 28         |                  |  PA24  | USB_NEGATIVE    | *USB/DM
 * | 29         |                  |  PA25  | USB_POSITIVE    | *USB/DP
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            |                  |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            | GND              |        |                 |
 * | 42         | AREF             |  PA03  |                 | EIC/EXTINT[3] *[ADC|DAC]/VREFA ADC/AIN[1] PTCXY[0]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            |32.768KHz Crystal |        |                 |
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * |            |                  |  PA00  | XIN32           | EIC/EXTINT[0]
 * |            |                  |  PA01  | XOUT32          | EIC/EXTINT[1]
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
  { PORTA, 23, PIO_SERCOM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM1_CH7, NOT_ON_TIMER, EXTERNAL_INT_7 }, // RX: SERCOM3/PAD[1]
  { PORTA, 22, PIO_SERCOM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM1_CH6, NOT_ON_TIMER, EXTERNAL_INT_6 }, // TX: SERCOM3/PAD[0]

  // 2..12
  // Digital Low
  { PORTA,  8, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM1_CH4, NOT_ON_TIMER, NOT_AN_INTERRUPT },
  { PORTA,  10, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM0_CH2, NOT_ON_TIMER, EXTERNAL_INT_10 },
  { PORTB,  12, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM3_CH0, NOT_ON_TIMER, EXTERNAL_INT_12 },
  { PORTB,  14, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM4_CH0, NOT_ON_TIMER, EXTERNAL_INT_14 },
  { PORTB,  15, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM4_CH1, NOT_ON_TIMER, EXTERNAL_INT_15 },
  { PORTA,  14, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM2_CH0, NOT_ON_TIMER, EXTERNAL_INT_14 },

  // Digital High
  { PORTA,  16, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM1_CH1, NOT_ON_TIMER, EXTERNAL_INT_0 },
  { PORTA,  17, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM1_CH1, NOT_ON_TIMER, EXTERNAL_INT_7  },
  { PORTA,  18, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM1_CH2, NOT_ON_TIMER, EXTERNAL_INT_2  },
  { PORTA,  19, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM1_CH3, NOT_ON_TIMER, EXTERNAL_INT_3  },
  { PORTA,  20, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM1_CH4, NOT_ON_TIMER, EXTERNAL_INT_4  },

  // 13 (LED)
  { PORTA,  21, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM1_CH5, NOT_ON_TIMER, EXTERNAL_INT_5  },

  // 14..19 - Analog pins
  // --------------------
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },
  { PORTA,  5, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },
  { PORTA,  6, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel6, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },
  { PORTA,  4, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel4, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },
  { PORTA,  11, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel11, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },
  { PORTA,  7, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel7, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },

  // Extra Analog pins! 20..23
  { PORTA,  8, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel8, NOT_ON_PWM, NOT_ON_TIMER, NOT_AN_INTERRUPT },  // same as D2
  { PORTA,  10, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel10, NOT_ON_PWM, NOT_ON_TIMER, NOT_AN_INTERRUPT }, // same as D3
  { PORTB,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel14, NOT_ON_PWM, NOT_ON_TIMER, NOT_AN_INTERRUPT }, // same as sda
  { PORTB,  3, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel15, NOT_ON_PWM, NOT_ON_TIMER, NOT_AN_INTERRUPT }, //same as scl

  // 24..25 I2C pins (SDA/SCL)
  // ----------------------
  { PORTB,  2, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, PWM2_CH2, TCC2_CH2, EXTERNAL_INT_6 }, //sda
  { PORTB,  3, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 }, //scl

  // 26..28 - SPI pins (ICSP:MISO,SCK,MOSI)
  // ----------------------
  { PORTA,  15, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, PWM2_CH1, TCC2_CH1, EXTERNAL_INT_15 },
  { PORTA,  12, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, PWM0_CH7, TCC0_CH7, EXTERNAL_INT_12 },
  { PORTA,  13, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, PWM0_CH6, TCC0_CH6, EXTERNAL_INT_13 },

  // 29..30 - RX/TX LEDS (PB06/PA27)
  // --------------------
  { PORTB, 6, PIO_OUTPUT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // used as output only
  { PORTA, 27, PIO_OUTPUT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // used as output only

  // 31..33 - USB
  // --------------------
  { PORTB, 7, PIO_COM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB Host enable
  { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DM
  { PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DP

  // 34..36 - Secondary SPI
  // ----------------------
  { PORTB,  11, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, NOT_AN_INTERRUPT }, //flash miso
  { PORTB,  8, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, NOT_AN_INTERRUPT }, //flash mosi
  { PORTB,  9, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, NOT_AN_INTERRUPT }, //flash sck
  // 37 Secondary SPI SS
  { PORTB,  10, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, NOT_AN_INTERRUPT }, //flash cs

  // 38 - Internal NeoPixel
  { PORTB, 17, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // used as output only

  // 39 (AREF)
  { PORTA, 3, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VREFP

  // ----------------------
  // 40 - 41 Alternate use of A0 and A1 (DAC output)
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // DAC/VOUT[0]
  { PORTA,  5, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel1, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, // DAC/VOUT[1]

  // ----------------------
  // 42 - 47 QSPI (SCK, CS, IO0, IO1, IO2, IO3)
  { PORTB, 10, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTB, 11, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTA, 8, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTA, 9, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTA, 10, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTA, 11, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TCC3, TCC4, TC5 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;

Uart Serial1( &sercom3, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

void SERCOM3_0_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM3_1_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM3_2_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM3_3_Handler()
{
  Serial1.IrqHandler();
}
