/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2017 MattairTech LLC. All right reserved.

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

#include "Arduino.h"
#include "sam.h"

#ifdef __cplusplus
extern "C" {
#endif

// not defined for SAML or SAMC in version of CMSIS used
#ifndef ADC_INPUTCTRL_MUXNEG_GND
#define ADC_INPUTCTRL_MUXNEG_GND (0x18ul << ADC_INPUTCTRL_MUXNEG_Pos)
#endif

// Wait for synchronization of registers between the clock domains
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
#if (SAMD)
  while ( ADC->STATUS.bit.SYNCBUSY == 1 );
#elif (SAML21)
  while ( ADC->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );
#elif (SAMC21)
  while ( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );
  while ( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );
#else
  #error "wiring_analog.c: Unsupported chip"
#endif
}

/*
 * System Core Clock is at 1MHz (8MHz/8) at Reset.
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
  // Set Systick to 1ms interval, common to all Cortex-M variants
  if ( SysTick_Config( SystemCoreClock / 1000 ) )
  {
    // Capture error
    while ( 1 ) ;
  }
  NVIC_SetPriority (SysTick_IRQn,  (1 << __NVIC_PRIO_BITS) - 2);  /* set Priority for Systick Interrupt (2nd lowest) */

  // Clock PORT for Digital I/O
//  PM->APBBMASK.reg |= PM_APBBMASK_PORT ;
//
//  // Clock EIC for I/O interrupts
//  PM->APBAMASK.reg |= PM_APBAMASK_EIC ;

  // Clock SERCOM for Serial, TC/TCC for Pulse and Analog, and ADC/DAC for Analog
#if (SAMD21 || SAMD11)
  uint32_t regAPBCMASK = PM->APBCMASK.reg;
  
  #if (SAMD11C)
  regAPBCMASK |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 ;
  regAPBCMASK |= PM_APBCMASK_TCC0 | PM_APBCMASK_TC1 | PM_APBCMASK_TC2 ;	// Note that on the D11C, TC2 is not routed to pins (but can be used internally)
  #elif (SAMD11D)
  regAPBCMASK |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_SERCOM2 ;
  regAPBCMASK |= PM_APBCMASK_TCC0 | PM_APBCMASK_TC1 | PM_APBCMASK_TC2 ;
  #elif (SAMD21E)
  regAPBCMASK |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_SERCOM2 | PM_APBCMASK_SERCOM3 ;
  regAPBCMASK |= PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1 | PM_APBCMASK_TCC2 | PM_APBCMASK_TC3 | PM_APBCMASK_TC4 | PM_APBCMASK_TC5
  #elif (SAMD21G)
  regAPBCMASK |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_SERCOM2 | PM_APBCMASK_SERCOM3 | PM_APBCMASK_SERCOM4 | PM_APBCMASK_SERCOM5 ;
  regAPBCMASK |= PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1 | PM_APBCMASK_TCC2 | PM_APBCMASK_TC3 | PM_APBCMASK_TC4 | PM_APBCMASK_TC5 ;
  #elif (SAMD21J)
  regAPBCMASK |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_SERCOM2 | PM_APBCMASK_SERCOM3 | PM_APBCMASK_SERCOM4 | PM_APBCMASK_SERCOM5 ;
  regAPBCMASK |= PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1 | PM_APBCMASK_TCC2 | PM_APBCMASK_TC3 | PM_APBCMASK_TC4 | PM_APBCMASK_TC5 | PM_APBCMASK_TC6 | PM_APBCMASK_TC7 ;
  #endif
  
  regAPBCMASK |= PM_APBCMASK_ADC | PM_APBCMASK_DAC ;
  
  PM->APBCMASK.reg |= regAPBCMASK ;
#elif (SAML21 || SAMC21)
  uint32_t regAPBCMASK = MCLK->APBCMASK.reg;
  
  #if (SAML)
  regAPBCMASK |= MCLK_APBCMASK_SERCOM0 | MCLK_APBCMASK_SERCOM1 | MCLK_APBCMASK_SERCOM2 | MCLK_APBCMASK_SERCOM3 | MCLK_APBCMASK_SERCOM4 ;
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM5;	// On the SAML, SERCOM5 is on the low power bridge
  regAPBCMASK |= MCLK_APBCMASK_TCC0 | MCLK_APBCMASK_TCC1 | MCLK_APBCMASK_TCC2 | MCLK_APBCMASK_TC0 | MCLK_APBCMASK_TC1 | MCLK_APBCMASK_TC2 ;
    #if (SAML21J)
    regAPBCMASK |= MCLK_APBCMASK_TC3 ;
    MCLK->APBDMASK.reg |= MCLK_APBDMASK_TC4;	// On the SAML, TC4 is on the low power bridge
    #endif
  #elif (SAMC21E)
  regAPBCMASK |= MCLK_APBCMASK_SERCOM0 | MCLK_APBCMASK_SERCOM1 | MCLK_APBCMASK_SERCOM2 | MCLK_APBCMASK_SERCOM3 ;
  regAPBCMASK |= MCLK_APBCMASK_TCC0 | MCLK_APBCMASK_TCC1 | MCLK_APBCMASK_TCC2 | MCLK_APBCMASK_TC0 | MCLK_APBCMASK_TC1 | MCLK_APBCMASK_TC2 | MCLK_APBCMASK_TC3 | MCLK_APBCMASK_TC4 ;
  #elif (SAMC21G) || (SAMC21J)
  regAPBCMASK |= MCLK_APBCMASK_SERCOM0 | MCLK_APBCMASK_SERCOM1 | MCLK_APBCMASK_SERCOM2 | MCLK_APBCMASK_SERCOM3 | MCLK_APBCMASK_SERCOM4 | MCLK_APBCMASK_SERCOM5 ;
  regAPBCMASK |= MCLK_APBCMASK_TCC0 | MCLK_APBCMASK_TCC1 | MCLK_APBCMASK_TCC2 | MCLK_APBCMASK_TC0 | MCLK_APBCMASK_TC1 | MCLK_APBCMASK_TC2 | MCLK_APBCMASK_TC3 | MCLK_APBCMASK_TC4 ;
  #endif
  
  #if (SAML)
  regAPBCMASK |= MCLK_APBCMASK_DAC ;
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_ADC;	// On the SAML, TC4 is on the low power bridge
  #elif (SAMC)
  regAPBCMASK |= MCLK_APBCMASK_ADC0 | MCLK_APBCMASK_ADC1 | MCLK_APBCMASK_DAC ;
  #endif
  
  MCLK->APBCMASK.reg |= regAPBCMASK ;
#else
  #error "wiring.c: Unsupported chip"
#endif

  //Setup all pins (digital and analog) in STARTUP mode (enable INEN and set default pull direction to pullup (pullup will not be enabled))
  for (uint32_t ul = 0 ; ul < NUM_DIGITAL_PINS ; ul++ )
  {
    pinMode( ul, PIO_STARTUP ) ;
  }

  // Initialize Analog Controller
  // Setting clock
#if (SAMD)
  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GCM_ADC ) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN ;

  syncADC();          // Wait for synchronization of registers between the clock domains

  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV512 |    // Divide Clock by 512.
  ADC_CTRLB_RESSEL_10BIT;         // 10 bits resolution as default
#elif (SAML21 || SAMC21)
  SUPC->VREF.reg |= SUPC_VREF_VREFOE;		// Enable Supply Controller Reference output for use with ADC and DAC (AR_INTREF)

  while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK );

  GCLK->PCHCTRL[GCM_ADC].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0 );

  syncADC();          // Wait for synchronization of registers between the clock domains

  #if (SAML21)
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV256;   // Divide Clock by 256.
  ADC->CTRLC.reg = ADC_CTRLB_RESSEL_10BIT;         // 10 bits resolution as default
  #elif (SAMC21)
  ADC0->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV256;   // Divide Clock by 256.
  ADC0->CTRLC.reg = ADC_CTRLB_RESSEL_10BIT;         // 10 bits resolution as default
  ADC1->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV256;   // Divide Clock by 256.
  ADC1->CTRLC.reg = ADC_CTRLB_RESSEL_10BIT;         // 10 bits resolution as default
  #endif
  syncADC();          // Wait for synchronization of registers between the clock domains
#endif
  
#if (SAMD || SAML21)
  ADC->SAMPCTRL.reg = 0x3f;     // Set max Sampling Time Length (in CLK_ADC cycles for SAMD, half CLK_ADC cycles for SAML and SAMC)
  syncADC();          // Wait for synchronization of registers between the clock domains

  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;   // No Negative input (Internal Ground)
  syncADC();          // Wait for synchronization of registers between the clock domains

  // Averaging (see datasheet table in AVGCTRL register description)
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |    // 1 sample only (no oversampling nor averaging)
                     ADC_AVGCTRL_ADJRES(0x0ul);   // Adjusting result by 0
#elif (SAMC21)
  ADC0->SAMPCTRL.reg = 0x3f;     // Set max Sampling Time Length (in CLK_ADC cycles for SAMD, half CLK_ADC cycles for SAML and SAMC)
  ADC1->SAMPCTRL.reg = 0x3f;
  syncADC();          // Wait for synchronization of registers between the clock domains

  ADC0->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;   // No Negative input (Internal Ground)
  ADC1->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;
  syncADC();          // Wait for synchronization of registers between the clock domains

  // Averaging (see datasheet table in AVGCTRL register description)
  ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |    // 1 sample only (no oversampling nor averaging)
                     ADC_AVGCTRL_ADJRES(0x0ul);   // Adjusting result by 0
  ADC1->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 | ADC_AVGCTRL_ADJRES(0x0ul);
#endif

  analogReference( VARIANT_AR_DEFAULT ) ;         // Use default reference from variant.h

  // Initialize DAC
  // Setting clock
#if (SAMD)
  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GCM_DAC ) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN ;

  while ( DAC->STATUS.bit.SYNCBUSY == 1 ); // Wait for synchronization of registers between the clock domains
  DAC->CTRLB.reg = DAC_CTRLB_REFSEL_AVCC | // Using the 3.3V reference
                   DAC_CTRLB_EOEN ;        // External Output Enable (Vout)
#elif (SAML21 || SAMC21)
  while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK );

  GCLK->PCHCTRL[GCM_DAC].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0 );
  while ( (GCLK->PCHCTRL[GCM_DAC].reg & GCLK_PCHCTRL_CHEN) == 0 );	// wait for sync

  while ( DAC->SYNCBUSY.reg & DAC_SYNCBUSY_MASK );

  #if (SAMC21)
    DAC->CTRLB.reg = DAC_CTRLB_REFSEL_AVCC;
  #elif (SAML21)
    DAC->CTRLB.reg = DAC_CTRLB_REFSEL_VDDANA;
    DAC->DACCTRL[0].reg = DAC_DACCTRL_REFRESH(2); // setup refresh
    DAC->DACCTRL[1].reg = DAC_DACCTRL_REFRESH(2); // setup refresh
  #endif
#endif
}

#ifdef __cplusplus
}
#endif
