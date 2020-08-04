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

#include "variant.h"

/*
 * Pins descriptions
 */
const PinDescription g_APinDescription[]=
{
  // GPIO 0-4 on external pads
  // D0, ADC, PWM, IRQ, and I2C SDA on sercom2 - no captouch
  { PORTA,  8, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel16, PWM0_CH0, TCC0_CH0, EXTERNAL_INT_NMI },  // TCC0/WO[0]

  // D1, ADC, DAC, IRQ, Captouch and general purpose pin - no timer
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // ADC/AIN[0]

  // D2, ADC, PWM, IRQ, and I2C SCL on sercom2 - no captouch
  { PORTA,  9, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel17, PWM0_CH1, TCC0_CH1, EXTERNAL_INT_9 }, // TCC0/WO[1]

  // D3, ADC, PWM, IRQ, UART RX, Captouch and general purpose pin
  { PORTA,  7, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel7, PWM1_CH1, TCC1_CH1, EXTERNAL_INT_7 }, // TCC1/WO[1]

  // D4, ADC, PWM, IRQ, UART TX, Captouch and general purpose pin
  { PORTA,  6, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel6, PWM1_CH0, TCC1_CH0, EXTERNAL_INT_6 }, // TCC1/WO[0]

  // D5, same as D0 but on sercom2 (for SPI sercommin')
  { PORTA,  8, PIO_SERCOM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel16, PWM0_CH0, TCC0_CH0, EXTERNAL_INT_NMI },  // TCC0/WO[0]
  // D6, same as D2 but on sercom2 (for SPI sercommin')
  { PORTA,  9, PIO_SERCOM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel17, PWM0_CH1, TCC0_CH1, EXTERNAL_INT_9 }, // TCC0/WO[1]

  // GPIO 7 & 8 - DotStar internal data/clock
  // ----------------------------------------
  { PORTA,  0, PIO_DIGITAL, 0, ADC_Channel4, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 }, // ADC/AIN[4]
  { PORTA,  1, PIO_DIGITAL, 0, ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, // ADC/AIN[5]

  // GPIO 9, 10, 11 USB Host enable
  // ----------------------------
  { PORTA, 28, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB Host enable - GPIO #5
  { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DM - GPIO #6
  { PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DP - GPIO #7

  // GPIO 12 is a placeholder, same as D13
  { PORTA, 10, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // TX: SERCOM0/PAD[2]

  // GPIO 13 (LED)
  { PORTA, 10, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // TX: SERCOM0/PAD[2]


  // Digital 14 - 18, Analog A0-A4
  // A0 same as D1
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // ADC/AIN[0]
  // A1, same as D2
  { PORTA,  9, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel17, PWM0_CH1, TCC0_CH1, EXTERNAL_INT_9 }, // TCC0/WO[1]
  // A2, same as D0
  { PORTA,  8, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel16, PWM0_CH0, TCC0_CH0, EXTERNAL_INT_NMI },  // TCC0/WO[0]
  // A3, same as D3
  { PORTA,  7, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel7, PWM1_CH1, TCC1_CH1, EXTERNAL_INT_7 }, // TCC1/WO[1]
  // A4, same as D4
  { PORTA,  6, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel6, PWM1_CH0, TCC1_CH0, EXTERNAL_INT_6 }, // TCC1/WO[0]


  // GPIO 19 & 20 (SWCLK & SWDIO)
  // --------------------------
  { PORTA, 30, PIO_TIMER, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTA, 31, PIO_TIMER, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },


  // Placeholder #21 & 22 for 'txled' and 'rxled'
  { PORTA, 14, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },
  { PORTA, 15, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM3_CH1, TC3_CH1, EXTERNAL_INT_15 },
} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TC3, TC4, TC5 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;

Uart Serial1( &sercom0, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

void SERCOM0_Handler()
{
  Serial1.IrqHandler();
}

