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
#include "variant.h"
#include "wiring_analog.h"
#include "../../config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * System Core Clock is at 1MHz (8MHz/8) at Reset.
 * It is switched to 48MHz in the Reset Handler (startup.c)
 */
uint32_t SystemCoreClock=1000000ul ;

/*
 * Arduino Zero board initialization
 *
 * Good to know:
 *   - At reset, ResetHandler did the system clock configuration. Core is running at 48MHz (optionally 120MHz for D51).
 *   - Watchdog is disabled by default, unless someone plays with NVM User page
 *   - During reset, all PORT lines are configured as inputs with input buffers, output buffers and pull disabled.
 */
void init( void )
{
  // Set Systick to 1ms interval, common to all Cortex-M variants
  // Since this only runs at startup, using VARIANT_MCK instead of SystemCoreClock to save code space
  if ( SysTick_Config( VARIANT_MCK / 1000 ) )
  {
    // Capture error
    while ( 1 ) ;
  }
  NVIC_SetPriority (SysTick_IRQn,  (1 << __NVIC_PRIO_BITS) - 2);  /* set Priority for Systick Interrupt (2nd lowest) */

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

  #if (SAML21)
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

  #if (SAML21)
  regAPBCMASK |= MCLK_APBCMASK_DAC ;
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_ADC;	// On the SAML, ADC is on the low power bridge
  #elif (SAMC21)
  regAPBCMASK |= MCLK_APBCMASK_ADC0 | MCLK_APBCMASK_ADC1 | MCLK_APBCMASK_DAC ;
  #endif

  MCLK->APBCMASK.reg |= regAPBCMASK ;
#elif (SAMD51)
  MCLK->APBAMASK.reg |= MCLK_APBAMASK_SERCOM0 | MCLK_APBAMASK_SERCOM1 | MCLK_APBAMASK_TC0 | MCLK_APBAMASK_TC1;
  MCLK->APBBMASK.reg |= MCLK_APBBMASK_SERCOM2 | MCLK_APBBMASK_SERCOM3 | MCLK_APBBMASK_TC2 | MCLK_APBBMASK_TC3 | MCLK_APBBMASK_TCC0 | MCLK_APBBMASK_TCC1;
  #if (SAMD51G)
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC2;
  #else
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC4 | MCLK_APBCMASK_TC5 | MCLK_APBCMASK_TCC2 | MCLK_APBCMASK_TCC3;
  #endif
  #if (SAMD51G)
    MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM4 | MCLK_APBDMASK_SERCOM5 | MCLK_APBDMASK_ADC0 | MCLK_APBDMASK_ADC1 | MCLK_APBDMASK_DAC;
  #elif (SAMD51J)
    MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM4 | MCLK_APBDMASK_SERCOM5 | MCLK_APBDMASK_TCC4 | MCLK_APBDMASK_ADC0 | MCLK_APBDMASK_ADC1 | MCLK_APBDMASK_DAC;
  #else
    MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM4 | MCLK_APBDMASK_SERCOM5 | MCLK_APBDMASK_SERCOM6 | MCLK_APBDMASK_SERCOM7 | MCLK_APBDMASK_TC6 | MCLK_APBDMASK_TC7 | MCLK_APBDMASK_TCC4 | MCLK_APBDMASK_ADC0 | MCLK_APBDMASK_ADC1 | MCLK_APBDMASK_DAC;
  #endif
#else
  #error "wiring.c: Unsupported chip"
#endif

  // Setup all pins (digital and analog) in STARTUP mode (enable INEN and set default pull direction to pullup (pullup will not be enabled))
  for (uint32_t ul = 0 ; ul < NUM_DIGITAL_PINS ; ul++ )
  {
    pinMode( ul, PIO_STARTUP ) ;
  }

  // At least on the L21, pin A31 must be set as an input. It is possible that debugger probe detection is being falsely
  // detected (even with a pullup on A31 (SWCLK)), which would change the peripheral mux of A31 to COM.
  // This might not normally be a problem, but one strange effect is that Serial2 (even when not using A31) may lose characters
  // if pin A31 is not set as INPUT. This is done above.

// I/O mux table footnote for D21 and D11: enable pullups on PA24 and PA24 when using as GPIO to avoid excessive current
//  Errata: disable pull resistors on PA24 or PA25 manually before switching to peripheral
//  Errata: do not use continuous sampling (not enabled by default) on PA24 or PA25
#if ((SAMD21 || SAMD11) && defined(USB_DISABLED))
  PORT->Group[0].OUTSET.reg = (uint32_t)(1<<PIN_PA24G_USB_DM);
  PORT->Group[0].PINCFG[PIN_PA24G_USB_DM].reg = (PORT_PINCFG_PULLEN | PORT_PINCFG_INEN);
  PORT->Group[0].OUTSET.reg = (uint32_t)(1<<PIN_PA25G_USB_DP);
  PORT->Group[0].PINCFG[PIN_PA25G_USB_DP].reg = (PORT_PINCFG_PULLEN | PORT_PINCFG_INEN);
#endif

#if !defined(ADC_NO_INIT_IF_UNUSED)
  initADC();         // Initialize Analog Controller
  analogReference( VARIANT_AR_DEFAULT ) ;         // Use default reference from variant.h
#endif

#if !defined(DAC_NO_INIT_IF_UNUSED)
  initDAC();         // Initialize DAC
#endif
}

#ifdef __cplusplus
}
#endif
