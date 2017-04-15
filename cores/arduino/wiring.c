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
//	PM->APBBMASK.reg |= PM_APBBMASK_PORT ;
//
//  // Clock EIC for I/O interrupts
//	PM->APBAMASK.reg |= PM_APBAMASK_EIC ;

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
  regAPBCMASK |= PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1 | PM_APBCMASK_TCC2 | PM_APBCMASK_TC3 | PM_APBCMASK_TC4 | PM_APBCMASK_TC5 ;
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
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM5 | MCLK_APBDMASK_TC4;	// On the SAML, SERCOM5 and TC4 are on the low power bridge
  regAPBCMASK |= MCLK_APBCMASK_TCC0 | MCLK_APBCMASK_TCC1 | MCLK_APBCMASK_TCC2 | MCLK_APBCMASK_TC0 | MCLK_APBCMASK_TC1 | MCLK_APBCMASK_TC2 ;
    #if (SAML21J)
    regAPBCMASK |= MCLK_APBCMASK_TC2 | MCLK_APBCMASK_TC3 ;
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
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_ADC;	// On the SAML, ADC is on the low power bridge
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

// I/O mux table footnote for D21 and D11: enable pullups on PA24 and PA24 when using as GPIO to avoid excessive current
//  Errata: disable pull resistors on PA24 or PA25 manually before switching to peripheral
//  Errata: do not use continuous sampling (not enabled by default) on PA24 or PA25
#if (SAMD && defined(USB_DISABLED))
  PORT->Group[0].OUTSET.reg = (uint32_t)(1<<PIN_PA24G_USB_DM);
  PORT->Group[0].PINCFG[PIN_PA24G_USB_DM].bit.PULLEN = 1;
  PORT->Group[0].OUTSET.reg = (uint32_t)(1<<PIN_PA25G_USB_DP);
  PORT->Group[0].PINCFG[PIN_PA25G_USB_DP].bit.PULLEN = 1;
#endif


  /* ----------------------------------------------------------------------------------------------
   * Initialize Analog Controller
   */

  // Load ADC factory calibration values
#if (SAMD)
  // ADC Bias Calibration
  uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;

  // ADC Linearity bits 4:0
  uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;

  // ADC Linearity bits 7:5
  linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;

  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);

#elif (SAML21)
  uint32_t biasrefbuf = (*((uint32_t *) ADC_FUSES_BIASREFBUF_ADDR) & ADC_FUSES_BIASREFBUF_Msk) >> ADC_FUSES_BIASREFBUF_Pos;
  uint32_t biascomp = (*((uint32_t *) ADC_FUSES_BIASCOMP_ADDR) & ADC_FUSES_BIASCOMP_Msk) >> ADC_FUSES_BIASCOMP_Pos;

  ADC->CALIB.reg = ADC_CALIB_BIASREFBUF(biasrefbuf) | ADC_CALIB_BIASCOMP(biascomp);

#elif (SAMC21)
  uint32_t biasrefbuf = (*((uint32_t *) ADC0_FUSES_BIASREFBUF_ADDR) & ADC0_FUSES_BIASREFBUF_Msk) >> ADC0_FUSES_BIASREFBUF_Pos;
  uint32_t biascomp = (*((uint32_t *) ADC0_FUSES_BIASCOMP_ADDR) & ADC0_FUSES_BIASCOMP_Msk) >> ADC0_FUSES_BIASCOMP_Pos;
  ADC0->CALIB.reg = ADC_CALIB_BIASREFBUF(biasrefbuf) | ADC_CALIB_BIASCOMP(biascomp);

  biasrefbuf = (*((uint32_t *) ADC1_FUSES_BIASREFBUF_ADDR) & ADC1_FUSES_BIASREFBUF_Msk) >> ADC1_FUSES_BIASREFBUF_Pos;
  biascomp = (*((uint32_t *) ADC1_FUSES_BIASCOMP_ADDR) & ADC1_FUSES_BIASCOMP_Msk) >> ADC1_FUSES_BIASCOMP_Pos;
  ADC1->CALIB.reg = ADC_CALIB_BIASREFBUF(biasrefbuf) | ADC_CALIB_BIASCOMP(biascomp);
#endif
  syncADC();          // Wait for synchronization of registers between the clock domains

  // Setting clock, prescaler and resolution
#if (SAMD)
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GCM_ADC ) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN ;
  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );

  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV512 |    // Divide Clock by 512.
  ADC_CTRLB_RESSEL_10BIT;         // 10 bits resolution as default
#elif (SAML21 || SAMC21)
  SUPC->VREF.reg |= SUPC_VREF_VREFOE;		// Enable Supply Controller Reference output for use with ADC and DAC (AR_INTREF)

  #if (SAML21)
    GCLK->PCHCTRL[GCM_ADC].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0 );
  #elif (SAMC21)
    GCLK->PCHCTRL[GCM_ADC0].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0 );
    GCLK->PCHCTRL[GCM_ADC1].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0 );
  #endif
  while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK );

  #if (SAML21)
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV256;   // Divide Clock by 256.
  ADC->CTRLC.reg = ADC_CTRLC_RESSEL_10BIT;         // 10 bits resolution as default
  #elif (SAMC21)
  ADC0->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV256;   // Divide Clock by 256.
  ADC0->CTRLC.reg = ADC_CTRLC_RESSEL_10BIT | ADC_CTRLC_R2R;         // 10 bits resolution as default, R2R requires ADC_SAMPCTRL_OFFCOMP=1
  ADC1->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV256;   // Divide Clock by 256.
  ADC1->CTRLC.reg = ADC_CTRLC_RESSEL_10BIT | ADC_CTRLC_R2R;         // 10 bits resolution as default, R2R requires ADC_SAMPCTRL_OFFCOMP=1
  #endif
#endif
  syncADC();          // Wait for synchronization of registers between the clock domains

  // Setting configuration
#if (SAMD || SAML21)
  ADC->SAMPCTRL.reg = 0x3f;     // Set max Sampling Time Length
  syncADC();          // Wait for synchronization of registers between the clock domains

  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;   // No Negative input (Internal Ground)
  syncADC();          // Wait for synchronization of registers between the clock domains

  // Averaging (see datasheet table in AVGCTRL register description)
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |    // 1 sample only (no oversampling nor averaging)
                     ADC_AVGCTRL_ADJRES(0x0ul);   // Adjusting result by 0
#elif (SAMC21)
  ADC0->SAMPCTRL.reg = (ADC_SAMPCTRL_SAMPLEN(0x0) | ADC_SAMPCTRL_OFFCOMP);     // ADC_SAMPCTRL_SAMPLEN must be 0 when ADC_SAMPCTRL_OFFCOMP=1
  ADC1->SAMPCTRL.reg = (ADC_SAMPCTRL_SAMPLEN(0x0) | ADC_SAMPCTRL_OFFCOMP);     // ADC_SAMPCTRL_SAMPLEN must be 0 when ADC_SAMPCTRL_OFFCOMP=1
  syncADC();          // Wait for synchronization of registers between the clock domains

  ADC0->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;   // No Negative input (Internal Ground)
  ADC1->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;
  syncADC();          // Wait for synchronization of registers between the clock domains

  // Averaging (see datasheet table in AVGCTRL register description)
  ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |    // 1 sample only (no oversampling nor averaging)
                     ADC_AVGCTRL_ADJRES(0x0ul);   // Adjusting result by 0
  ADC1->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 | ADC_AVGCTRL_ADJRES(0x0ul);
#endif
  syncADC();          // Wait for synchronization of registers between the clock domains

  analogReference( VARIANT_AR_DEFAULT ) ;         // Use default reference from variant.h


  /* ----------------------------------------------------------------------------------------------
   * Initialize DAC
   */
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
    DAC->CTRLB.reg = (DAC_CTRLB_REFSEL_AVCC | DAC_CTRLB_EOEN);
  #elif (SAML21)
    DAC->CTRLB.reg = DAC_CTRLB_REFSEL_VDDANA;
    DAC->DACCTRL[0].reg = (DAC_DACCTRL_REFRESH(3) | DAC_DACCTRL_CCTRL(2));
    DAC->DACCTRL[1].reg = (DAC_DACCTRL_REFRESH(3) | DAC_DACCTRL_CCTRL(2));
  #endif
#endif
}

#ifdef __cplusplus
}
#endif
