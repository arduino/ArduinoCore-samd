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
#include <Arduino.h>

/*
 * Pins descriptions
 */
const PinDescription g_APinDescription[]=
{
/*
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Pin number | Serial           |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 0          |                  |  PA15  | EXT3_5          | EIC/EXTINT[15] SERCOM2/PAD[3] SERCOM4/PAD[3] TC3/WO[1]  FECTRL[5] GCLK_IO[1]
 * | 1          |                  |  PA14  | EXT3_15         | EIC/EXTINT[14]  SERCOM2/PAD[2]  TC3/WO[0]  FECTRL[4]  GCLK_IO[0]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 */
  { PORTA, 15, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 }, // RX: SERCOM2/PAD[3]
  { PORTA, 14, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 }, // TX: SERCOM2/PAD[2]

/* +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Pin number | LEDs & button    |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 2          | N/A              |  PA19  | LED0/FSYNC      | EIC/EXTINT[3]  PTC/X[7]  SERCOM1/PAD[3]  SERCOM3_ALT/PAD[3] *TC3/WO[1]  TCC0_ALT/WO[3]  I2S/SD[0]  AC/CMP[1]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 */
  { PORTA, 19, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM3_CH1, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // TC3/WO[1]

/* +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Pin number | Digital/PWM      |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 3         | PWM+/INTA        |  PA18  | EXT1_7          | EIC/EXTINT[2]  PTC/X[6]  SERCOM1/PAD[2]  SERCOM3_ALT/PAD[2]  TC3/WO[0] *TCC0_ALT/WO[2]  AC/CMP[0]
 * | 4         | PWM-/FSYNC       |  PA19  | EXT1_8          | EIC/EXTINT[3]  PTC/X[7]  SERCOM1/PAD[3]  SERCOM3_ALT/PAD[3]  TC3/WO[1] *TCC0_ALT/WO[3]  I2S/SD[0]  AC/CMP[1]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 */
  { PORTA, 18, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH2, TCC0_CH2, EXTERNAL_INT_2 }, // TCC0/WO[2]
  { PORTA, 19, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH3, TCC0_CH3, EXTERNAL_INT_3 }, // TCC0/WO[3], also LED0

/*
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Pin number | Analog pins      |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 5          |                  |  PA06  | A0              | EIC/EXTINT[6] *ADC/AIN[6]  AC/AIN[2]  PTC/Y[4]  SERCOM0_ALT/PAD[2]  TCC1/WO[0]
 * | 6          |                  |  PA07  | A1              | EIC/EXTINT[7] *ADC/AIN[7]  AC/AIN[3]  PTC/Y[5]  SERCOM0_ALT/PAD[3]  TCC1/WO[1]  I2S/SD[0]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 */
  { PORTA,  6, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel6, PWM1_CH0, TCC1_CH0, EXTERNAL_INT_6 }, // ADC/AIN[6]
  { PORTA,  7, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel7, PWM1_CH1, TCC1_CH1, EXTERNAL_INT_7 }, // ADC/AIN[7]


/*
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Pin number | Wire             |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 7          | SCL              |  PA09  |                 | EIC/EXTINT[9] *ADC/AIN[17]  PTC/X[1]  SERCOM0/PAD[1] SERCOM2_ALT/PAD[1] TCC0/WO/[1]  FECTRL[1]
 * | 8          | SDA              |  PA08  | EXT3_10         | EIC/NMI  ADC/AIN[16]  PTC/X[0]  SERCOM0/PAD[0]  SERCOM2_ALT/PAD[0]  TCC0/WO[0]  FECTRL[0]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 */

 { PORTA,  9, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, PWM0_CH1, TCC0_CH1, EXTERNAL_INT_9 },  // SCL:SERCOM0/PAD[0]
 { PORTA,  8, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, PWM0_CH0, TCC0_CH0, EXTERNAL_INT_NMI },  // SDA:SERCOM0/PAD[0]

/* +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Pin number | ATRF233          |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 9          |                  |  PB30  | MOSI            | SERCOM4/PAD[2]
 * | 10         |                  |  PC19  | MISO            | SERCOM4/PAD[0]
 * | 11         |                  |  PC18  | SCK             | SERCOM4/PAD[3]
 * | 12         |                  |  PB31  | SS              | SERCOM4/PAD[1]
 * | 13         |                  |  PC16  | CLKM            | GCLK/IO[1]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 */
  { PORTB, 30, PIO_TIMER_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SERCOM4_ALT/PAD[2]
  { PORTC, 19, PIO_TIMER_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SERCOM4_ALT/PAD[0]
  { PORTC, 18, PIO_TIMER_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SERCOM4_ALT/PAD[3]
  { PORTB, 31, PIO_TIMER_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SERCOM4_ALT/PAD[1]
  { PORTC, 16, PIO_TIMER_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // GCLK/IO[1]

/* +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Pin number | USB              |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 14         |                  |  PA25  | USB_POSITIVE    | *USB/DP
 * | 15         |                  |  PA24  | USB_NEGATIVE    | *USB/DM
 * | 16         |                  |  PA28  | USB_HOST_ENABLE | USB/VBUS | EXT1_6          | EIC/EXTINT[8]  GCLK_IO[0]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 */
  { PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DP
  { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DM
  { PORTA, 28, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE } // USB/VBUS
} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TC3, TC4, TC5 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ; // Wire
// SERCOM sercom1( SERCOM1 ) ; // 
SERCOM sercom2( SERCOM2 ) ; // Serial
SERCOM sercom4( SERCOM4 ) ; // ATRF233/802.15.4
// SERCOM sercom5( SERCOM5 ) ; // SPI

Uart Serial( &sercom2, PIN_SERIAL_RX, PIN_SERIAL_TX, PAD_SERIAL_RX, PAD_SERIAL_TX ) ;

void SERCOM2_Handler(void)
{
  Serial.IrqHandler();
}
