/*
  Copyright (c) 2011 Arduino.  All right reserved.

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

//#include "Arduino.h"
#include "variant.h"
//#include "wiring_constants.h"
#include "wiring_digital.h"
#include "wiring.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * System Core Clock is at 1MHz at Reset.
 * It is switched to 48MHz in the Reset Handler (startup.c)
 */
uint32_t SystemCoreClock=1000000ul ;

/*
void calibrateADC()
{
  volatile uint32_t valeur = 0;

  for(int i = 0; i < 5; ++i)
  {
    ADC->SWTRIG.bit.START = 1;
    while( ADC->INTFLAG.bit.RESRDY == 0 || ADC->STATUS.bit.SYNCBUSY == 1 )
    {
      // Waiting for a complete conversion and complete synchronization
    }

    valeur += ADC->RESULT.bit.RESULT;
  }

  valeur = valeur/5;
}*/

/*
 * Arduino Zero board initialization
 *
 * Good to know:
 *   - At reset, ResetHandler did the system clock configuration. Core is running at 48MHz.
 *   - Watchdog is disabled by default, unless someone plays with NVM User page
 *   - During reset, all PORT lines are configured as inputs with input buffers, output buffers and pull disabled.
 */
void init( void )
{
  uint32_t ul ;

  // Set Systick to 1ms interval, common to all Cortex-M variants
  if ( SysTick_Config( SystemCoreClock / 1000 ) )
  {
    // Capture error
    while ( 1 ) ;
  }

  // Clock PORT for Digital I/O
//	PM->APBBMASK.reg |= PM_APBBMASK_PORT ;
//
//  // Clock EIC for I/O interrupts
//	PM->APBAMASK.reg |= PM_APBAMASK_EIC ;

  // Clock SERCOM for Serial
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_SERCOM2 | PM_APBCMASK_SERCOM3 | PM_APBCMASK_SERCOM4 | PM_APBCMASK_SERCOM5 ;

  // Clock TC/TCC for Pulse and Analog
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1 | PM_APBCMASK_TCC2 | PM_APBCMASK_TC3 | PM_APBCMASK_TC4 | PM_APBCMASK_TC5 | PM_APBCMASK_TC6 | PM_APBCMASK_TC7 ;

  // Clock ADC/DAC for Analog
  PM->APBCMASK.reg |= PM_APBCMASK_ADC | PM_APBCMASK_DAC ;

  // Setup all pins (digital and analog) in INPUT mode (default is nothing)
	for ( ul = 0 ; ul < NUM_DIGITAL_PINS ; ul++ )
  {
	  pinMode( ul, INPUT ) ;
  }

  // Initialize Serial port U(S)ART pins
  // Todo

  // Initialize USB pins
  // Todo

  // Initialize Analog Controller
  // Setting clock
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GCM_ADC ) | // Generic Clock ADC
                      GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is source
                      GCLK_CLKCTRL_CLKEN ;

  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV128 |     // Divide Clock by 512.
                   ADC_CTRLB_RESSEL_10BIT;        // Result on 10 bits

  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;   // No Negative input (Internal Ground)

  // Averaging (see table 31-2 p.816 datasheet)
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_2 |    // 2 samples
                     ADC_AVGCTRL_ADJRES(0x01ul);  // Adjusting result by 1
  
  ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_AREFA; // RReference AREFA (pin AREF) [default]

  ADC->CTRLA.bit.ENABLE = 1; // Enable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 )
  {
    // Waiting for synchroinization
  }

  // Initialize DAC
  // Setting clock
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GCM_DAC ) | // Generic Clock ADC
                      GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is source
                      GCLK_CLKCTRL_CLKEN ;


  DAC->CTRLB.reg = DAC_CTRLB_REFSEL_AVCC | // Using the 3.3V reference
                   DAC_CTRLB_EOEN;  // External Output Enable (Vout)
  DAC->DATA.reg = 0x3FFul;

  // Enable DAC
  DAC->CTRLA.bit.ENABLE = 1;

  while(DAC->STATUS.bit.SYNCBUSY != 0)
  {
    // Waiting for synchronization
  }

}

#ifdef __cplusplus
}
#endif
