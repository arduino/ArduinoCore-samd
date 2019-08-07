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

// PyPortal uses ATSAMD51J20 (64-pin), Metro & Feather M4 use J19, Trellis uses 51G
// NOT SURE ABOUT PIN_ATTR_PWM_n VALUES, defaulting all to "E" for now, update as required
// Also setting PWM and TIMER pins to same value to start, update as required
// Is LIGHT pin connected/routed anywhere?

/* NEW PIN DEFS IN PROGRESS:

  { PORTA,  3, PIO_ANALOG    , PIN_ATTR_ANALOG                 , ADC_Channel1  , NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3    }, // AREF - NOT USED
  { PORTA, 30, PIO_DIGITAL   , PIN_ATTR_PWM_E                  , No_ADC_Channel, TC6_CH0   , TC6_CH0     , EXTERNAL_INT_14   }, // SWCLK - not used?
  { PORTA, 31, PIO_DIGITAL   , PIN_ATTR_PWM_E                  , No_ADC_Channel, TC6_CH1   , TC6_CH1     , EXTERNAL_INT_15   }, // SWDIO - not used?

*/


/*
 * Pins descriptions
 */
const PinDescription g_APinDescription[]=
{
  // 0..13 - Digital pins
  // ----------------------
  // 0/1 - SERCOM/UART (Serial1)
  { PORTB, 13, PIO_SERCOM    , PIN_ATTR_NONE                   , No_ADC_Channel, TC4_CH1   , TCC3_CH1    , EXTERNAL_INT_13   }, // RXD (pin 0, to ESP32), SERCOM4/PAD[1]
  { PORTB, 12, PIO_SERCOM    , PIN_ATTR_NONE                   , No_ADC_Channel, TC4_CH0   , TCC3_CH0    , EXTERNAL_INT_12   }, // TXD (pin 1, to ESP32), SERCOM4/PAD[0]

  // 2..4
  { PORTB, 22, PIO_DIGITAL   , PIN_ATTR_PWM_E                  , No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6    }, // NEOPIX
  { PORTA,  4, PIO_SERCOM_ALT, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel4  , TC0_CH0   , TC0_CH0     , EXTERNAL_INT_4    }, // D3 (SENSE JST)
  { PORTA,  5, PIO_SERCOM_ALT, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel5  , TC0_CH1   , TC0_CH1     , EXTERNAL_INT_5    }, // D4 (NEOPIX JST)

  // 5..12  ESP32 and TFT control lines
  { PORTB, 16, PIO_DIGITAL   , PIN_ATTR_PWM_E                  , No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0    }, // ESP_BUSY
  { PORTB, 15, PIO_DIGITAL   , PIN_ATTR_PWM_E                  , No_ADC_Channel, TC5_CH1   , TC5_CH1     , EXTERNAL_INT_15   }, // ESP_GPIO0
  { PORTB, 17, PIO_DIGITAL   , PIN_ATTR_PWM_E                  , No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1    }, // ESP RESET
  { PORTB, 14, PIO_DIGITAL   , PIN_ATTR_PWM_E                  , No_ADC_Channel, TC5_CH0   , TC5_CH0     , EXTERNAL_INT_14   }, // ESP_CS
  { PORTB,  4, PIO_DIGITAL   , PIN_ATTR_NONE                   , No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4    }, // TFT_RD
  { PORTB,  5, PIO_DIGITAL   , PIN_ATTR_NONE                   , No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5    }, // TFT_RS
  { PORTB,  6, PIO_DIGITAL   , PIN_ATTR_NONE                   , No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6    }, // TFT_CS
  { PORTB,  7, PIO_DIGITAL   , PIN_ATTR_NONE                   , No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7    }, // TFT_TE

  // 13 (LED)
  { PORTB, 23, PIO_DIGITAL   , PIN_ATTR_PWM_E                  , No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7    }, // D13 (LED)

  // 14..23  Analog pins
  { PORTA,  2, PIO_ANALOG    , PIN_ATTR_ANALOG                 , DAC_Channel0  , NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2    }, // AUDIO_OUT (A0)
  { PORTA,  4, PIO_ANALOG    , (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel4  , TC0_CH0   , TC0_CH0     , EXTERNAL_INT_4    }, // D3 (SENSE JST)
  { PORTA,  7, PIO_DIGITAL   , (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel7  , TC1_CH1   , TC1_CH1     , EXTERNAL_INT_7    }, // Light sensor (A2)
  { PORTA,  5, PIO_ANALOG    , (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel5  , TC0_CH1   , TC0_CH1     , EXTERNAL_INT_5    }, // D4 (NEOPIX JST)
  { PORTB,  0, PIO_ANALOG    , (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel12 , NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0    }, // TOUCH_YD
  { PORTB,  1, PIO_ANALOG    , (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel13 , NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1    }, // TOUCH_XL
  { PORTA,  6, PIO_ANALOG    , (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel6  , TC1_CH0   , TC1_CH0     , EXTERNAL_INT_6    }, // TOUCH_YU
  { PORTB,  8, PIO_ANALOG    , (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel2  , TC4_CH0   , TC4_CH0     , EXTERNAL_INT_8    }, // TOUCH_XR
  { PORTB,  2, PIO_SERCOM_ALT, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel14 , NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2    }, // SDA (JST), SERCOM5/PAD[0] - analog copy
  { PORTB,  3, PIO_SERCOM_ALT, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel15 , NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3    }, // SCL (JST), SERCOM5/PAD[1] - analog copy

  // 24..26 more TFT control lines
  { PORTA,  0, PIO_DIGITAL   , PIN_ATTR_PWM_E                  , No_ADC_Channel, TC2_CH0   , TC2_CH0     , EXTERNAL_INT_0    }, // TFT_RESET
  { PORTB, 31, PIO_DIGITAL   , PIN_ATTR_PWM_E                  , No_ADC_Channel, TC0_CH1   , TC0_CH1     , EXTERNAL_INT_15   }, // TFT_BACKLITE
  { PORTB,  9, PIO_DIGITAL   , (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel3  , TC4_CH1   , TC4_CH1     , EXTERNAL_INT_9    }, // TFT_WR, CCL/OUT[2]

  // 27..28  I2C pins
  { PORTB,  2, PIO_SERCOM_ALT, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel14 , NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2    }, // SDA (JST), SERCOM5/PAD[0]
  { PORTB,  3, PIO_SERCOM_ALT, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel15 , NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3    }, // SCL (JST), SERCOM5/PAD[1]

  // 29..31  SPI pins
  { PORTA, 12, PIO_SERCOM    , PIN_ATTR_PWM_E                  , No_ADC_Channel, TC2_CH0   , TC2_CH0     , EXTERNAL_INT_12   }, // MOSI, SERCOM2/PAD[0]
  { PORTA, 13, PIO_SERCOM    , PIN_ATTR_PWM_E                  , No_ADC_Channel, TC2_CH1   , TC2_CH1     , EXTERNAL_INT_13   }, // SCK,  SERCOM2/PAD[1]
  { PORTA, 14, PIO_SERCOM    , PIN_ATTR_PWM_E                  , No_ADC_Channel, TC3_CH0   , TC3_CH0     , EXTERNAL_INT_14   }, // MISO, SERCOM2/PAD[2]

  // 32..33  Some SD card control
  { PORTB, 30, PIO_DIGITAL   , PIN_ATTR_PWM_E                  , No_ADC_Channel, TC0_CH0   , TC0_CH0     , EXTERNAL_INT_14   }, // SD_CS
  { PORTA,  1, PIO_DIGITAL   , PIN_ATTR_PWM_E                  , No_ADC_Channel, TC2_CH1   , TC2_CH1     , EXTERNAL_INT_1    }, // CARDDET

  // 34..41  LCD data
  { PORTA, 16, PIO_DIGITAL   , PIN_ATTR_PWM_E                  , No_ADC_Channel, TC2_CH0   , TC2_CH0     , EXTERNAL_INT_0    }, // LCD_DATA0
  { PORTA, 17, PIO_DIGITAL   , PIN_ATTR_PWM_F                  , No_ADC_Channel, TCC1_CH1  , TC2_CH1     , EXTERNAL_INT_1    }, // LCD_DATA1
  { PORTA, 18, PIO_DIGITAL   , PIN_ATTR_PWM_F                  , No_ADC_Channel, TCC1_CH2  , TC3_CH0     , EXTERNAL_INT_2    }, // LCD_DATA2
  { PORTA, 19, PIO_DIGITAL   , PIN_ATTR_PWM_F                  , No_ADC_Channel, TCC1_CH3  , TC3_CH1     , EXTERNAL_INT_3    }, // LCD_DATA3
  { PORTA, 20, PIO_DIGITAL   , PIN_ATTR_PWM_G                  , No_ADC_Channel, TCC0_CH0  , TC0_CH0     , EXTERNAL_INT_4    }, // LCD_DATA4
  { PORTA, 21, PIO_DIGITAL   , PIN_ATTR_PWM_G                  , No_ADC_Channel, TCC0_CH0  , TC0_CH1     , EXTERNAL_INT_5    }, // LCD_DATA5
  { PORTA, 22, PIO_DIGITAL   , PIN_ATTR_PWM_G                  , No_ADC_Channel, TCC0_CH2  , TC4_CH0     , EXTERNAL_INT_6    }, // LCD_DATA6
  { PORTA, 23, PIO_DIGITAL   , PIN_ATTR_PWM_G                  , No_ADC_Channel, TCC0_CH3  , TC4_CH1     , EXTERNAL_INT_7    }, // LCD_DATA7

  // 42..47  QSPI
  { PORTB, 10, PIO_COM       , PIN_ATTR_NONE                   , No_ADC_Channel, TC5_CH0   , TC5_CH0     , EXTERNAL_INT_10   }, // QSPI_SCK
  { PORTB, 11, PIO_COM       , PIN_ATTR_NONE                   , No_ADC_Channel, TC5_CH1   , TC5_CH1     , EXTERNAL_INT_11   }, // QSPI_CS
  { PORTA,  8, PIO_COM       , PIN_ATTR_NONE                   , No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI  }, // QSPI_DATA0
  { PORTA,  9, PIO_COM       , PIN_ATTR_NONE                   , No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9    }, // QSPI_DATA1
  { PORTA, 10, PIO_COM       , PIN_ATTR_NONE                   , No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10   }, // QSPI_DATA2
  { PORTA, 11, PIO_COM       , PIN_ATTR_NONE                   , No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11   }, // QSPI_DATA3

  // 48..49  USB
  { PORTA, 24, PIO_COM       , PIN_ATTR_NONE                   , No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB D-
  { PORTA, 25, PIO_COM       , PIN_ATTR_NONE                   , No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB D+

  // 50..51
  { PORTA, 27, PIO_DIGITAL   , PIN_ATTR_NONE                   , No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11   }, // Speaker shutdown
  { PORTA, 15, PIO_DIGITAL   , PIN_ATTR_PWM_E                  , No_ADC_Channel, TC3_CH1   , TC3_CH1     , EXTERNAL_INT_15   }, // ESP_RTS

} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TCC3, TCC4, TC0, TC1, TC2, TC3, TC4, TC5 } ;
const uint32_t GCLK_CLKCTRL_IDs[TCC_INST_NUM+TC_INST_NUM] = { TCC0_GCLK_ID, TCC1_GCLK_ID, TCC2_GCLK_ID, TCC3_GCLK_ID, TCC4_GCLK_ID, TC0_GCLK_ID, TC1_GCLK_ID, TC2_GCLK_ID, TC3_GCLK_ID, TC4_GCLK_ID, TC5_GCLK_ID } ;


// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;

Uart Serial1( &sercom4, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

void SERCOM4_0_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM4_1_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM4_2_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM4_3_Handler()
{
  Serial1.IrqHandler();
}
