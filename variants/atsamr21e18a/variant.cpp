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
 * Note: FSYNC is mislabeled as "18/INT" on FemtoBeacon coins r2.0.0 silkscreen
 * Note: INT is mislabeled as "19/FSYNC" on FemtoBeacon coins r2.0.0 silkscreen
 * Note: FSYNC and INT labels are correct on FemtoBeacon coins r2.0.7 and up. RGB LEDs available on r2.0.7 and up.
 */
const PinDescription g_APinDescription[]=
{
/*
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Pin number | Serial           |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 0          |                  |  PA17  | RX              | EXTINT[1]      PTX/X[5]  SERCOM1/PAD[1]  SERCOM3_ALT[PAD1]  TCC2/WO[1]  *TCC0_ALT/WO[1] GCLK_IO[3]
 * | 1          |                  |  PA16  | TX              |                PTC/X[4]  SERCOM1/PAD[0]  SERCOM3_ALT[PAD0]  TCC2/WO[0]  *TCC0_ALT/WO[0] GCLK_IO[2]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 */
  { PORTA, 17, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, // RX: SERCOM1/PAD[1]
  { PORTA, 16, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // TX: SERCOM1/PAD[0]

/* +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Pin number | LEDs & button    |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 2          | N/A              |  PA19  | INTA            | EIC/EXTINT[3]  PTC/X[7]  SERCOM1/PAD[3]  SERCOM3_ALT/PAD[3] *TC3/WO[1]  TCC0_ALT/WO[3]  I2S/SD[0]  AC/CMP[1]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 */
  { PORTA, 19, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM3_CH1, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // TC3/WO[1]

/* +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Pin number | Digital/PWM      |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 3         | PWM+/FSYNC        |  PA18  | EXT1_7          | EIC/EXTINT[2]  PTC/X[6]  SERCOM1/PAD[2]  SERCOM3_ALT/PAD[2]  TC3/WO[0] *TCC0_ALT/WO[2]  AC/CMP[0]
 * | 4         | PWM-/INTA         |  PA19  | EXT1_8          | EIC/EXTINT[3]  PTC/X[7]  SERCOM1/PAD[3]  SERCOM3_ALT/PAD[3]  TC3/WO[1] *TCC0_ALT/WO[3]  I2S/SD[0]  AC/CMP[1]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 */
  { PORTA, 18, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH2, TCC0_CH2, EXTERNAL_INT_2 }, // TCC0/WO[2]
  { PORTA, 19, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH3, TCC0_CH3, EXTERNAL_INT_3 }, // TCC0/WO[3], also LED0

/*
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Pin number | Analog pins      |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 5          |                  |  PA06  | A0/LED Red      | EIC/EXTINT[6] *ADC/AIN[6]  AC/AIN[2]  PTC/Y[4]  SERCOM0_ALT/PAD[2]  TCC1/WO[0]
 * | 6          |                  |  PA07  | A1/LED Green    | EIC/EXTINT[7] *ADC/AIN[7]  AC/AIN[3]  PTC/Y[5]  SERCOM0_ALT/PAD[3]  TCC1/WO[1]  I2S/SD[0]
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
 * | 9          |                  |  PB30  | MOSI            | SERCOM4/PAD[2]   peripheral F
 * | 10         |                  |  PC19  | MISO            | SERCOM4/PAD[0]   peripheral F
 * | 11         |                  |  PC18  | SCK             | SERCOM4/PAD[3]   peripheral F
 * | 12         |                  |  PB31  | SS (/SEL, CS)   | SERCOM4/PAD[1]   peripheral F
 * |            |                  |        |                 |  
 * | 13         |                  |  PB00  | IRQ             | EXTINT[0]
 * | 14         |                  |  PA20  | SLP_TR          | 
 * | 15         |                  |  PB15  | /RST            | 
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 */
  { PORTB, 30, PIO_TIMER_ALT, (PIN_ATTR_TIMER_ALT), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SERCOM4/PAD[2]
  { PORTC, 19, PIO_TIMER_ALT, (PIN_ATTR_TIMER_ALT), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SERCOM4/PAD[0]
  { PORTC, 18, PIO_TIMER_ALT, (PIN_ATTR_TIMER_ALT), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SERCOM4/PAD[3]
  { PORTB, 31, PIO_TIMER_ALT, (PIN_ATTR_TIMER_ALT), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SERCOM4/PAD[1]

  { PORTB,  0, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 }, // EXTINT[0]
  { PORTA, 20, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // 
  { PORTB, 15, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // 

/* +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Pin number | USB              |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 16         |                  |  PA25  | USB_POSITIVE    | *USB/DP
 * | 17         |                  |  PA24  | USB_NEGATIVE    | *USB/DM
 * | 18         |                  |  PA28  | USB_HOST_ENABLE | USB/VBUS | EXT1_6          | EIC/EXTINT[8]  GCLK_IO[0]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 */
  { PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DP
  { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DM
  { PORTA, 28, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/VBUS

/* +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Pin number | Antenna          |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 19         |                  |  PA09  | RF1             | FECTRL[0]/DIG1   peripheral F
 * | 20         |                  |  PA12  | RF2             | FECTRL[1]/DIG2   peripheral F
 * | 21         |                  |  PA27  | LED Blue        | EXTINT[15]  SERCOM3/PAD[0]   GCLK_IO[0]
 * | 22         |                  |  PA30  | SWCLK           | SERCOM1/PAD[2] (SERCOM_ALT, peripheral D), TCC1/WO[0]
 * | 23         |                  |  PA31  | SWDIO           | SERCOM1/PAD[3] (SERCOM_ALT, peripheral D), TCC1/WO[1]
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 */
  { PORTA,  9, PIO_FECTRL, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER_ALT), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DIG1
  { PORTA,  12, PIO_FECTRL, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER_ALT), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DIG2

  { PORTA, 28, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 }//, // USB/VBUS
  { PORTA, 30, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SWCLK
  { PORTA, 31, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE } // SWDIO

  /* +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | Pin number | Antenna          |  PIN   | Label/Name      | Comments (* is for default peripheral in use)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 * | 24         |                  |  PA15  |                 | SERCOM2/PAD[3] (TC3/WO[1], peripheral E) (FECTRL[5], peripheral F) (GCLK_IO[1], peripheral H)
 * +------------+------------------+--------+-----------------+--------------------------------------------------------------------------------------------------------
 */
} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TC3, TC4, TC5 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ; // Wire
SERCOM sercom1( SERCOM1 ) ; // Serial
SERCOM sercom2( SERCOM2 ) ; // ???
SERCOM sercom3( SERCOM3 ) ; // ???
SERCOM sercom4( SERCOM4 ) ; // ATRF233/802.15.4
// SERCOM sercom5( SERCOM5 ) ; // SPI

Uart Serial( &sercom1, PIN_SERIAL_RX, PIN_SERIAL_TX, PAD_SERIAL_RX, PAD_SERIAL_TX ) ;

void SERCOM1_Handler(void)
{
  Serial.IrqHandler();
}
