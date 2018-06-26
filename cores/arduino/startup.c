/*
  Copyright (c) 2017-2018 MattairTech LLC. All right reserved.
  Copyright (c) 2015 Arduino LLC.  All right reserved.

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

#include "sam.h"
#include "variant.h"
#include "../../config.h"

#include <stdio.h>

/**
 * \brief SystemInit() configures the needed clocks and according Flash Read Wait States.
 */

// Constants for Clock generators (the D51 has 12 generators and all others have 9 generators). Unused generators are automatically stopped to reduce power consumption.
#define GENERIC_CLOCK_GENERATOR_MAIN      (0u)  /* Used for the CPU/APB clocks. With the D51, it runs at either 96MHz (divided by 2 in MCLK) or 120MHz undivided. Otherwise, it runs at 48MHz. */
#define GENERIC_CLOCK_GENERATOR_XOSC      (1u)  /* The high speed crystal is connected to GCLK1 in order to use the 16-bit prescaler. */
#define GENERIC_CLOCK_GENERATOR_OSCULP32K (2u)  /* Initialized at reset for WDT (D21 and D11 only). Not used by core. */
#define GENERIC_CLOCK_GENERATOR_OSC_HS    (3u)  /* 8MHz from internal RC oscillator (D21, D11, and L21 only). Setup by core but not used. */
#define GENERIC_CLOCK_GENERATOR_48MHz     (4u)  /* Used for USB or any peripheral that has a 48MHz (60MHz for D51) maximum peripheral clock. GCLK0 is now only 96MHz or 120MHz with the D51. */
#define GENERIC_CLOCK_GENERATOR_TIMERS    (5u)  /* Used by the timers for controlling PWM frequency. Can be up to 48MHz (up to 96MHz with the D51). */
#define GENERIC_CLOCK_GENERATOR_192MHz    (6u)  /* Used only by D51 for any peripheral that has a 200MHz maximum peripheral clock (note that GCLK8 - GCLK11 must be <= 100MHz). */
#define GENERIC_CLOCK_GENERATOR_I2S       (7u)  /* Used by D51 and D21 for I2S peripheral. This define is not currently used. The generator is defined in each variant.h. */
#define GENERIC_CLOCK_GENERATOR_I2S1      (8u)  /* Used by D51 and D21 for I2S peripheral. This define is not currently used. The generator is defined in each variant.h. */
#define GENERIC_CLOCK_GENERATOR_DFLL      (9u)  /* Used only by D51 (only when the cpu is 120MHz) with CLOCKCONFIG_INTERNAL or CLOCKCONFIG_INTERNAL_USB to generate 2MHz output for the PLL input. */
#define GENERIC_CLOCK_GENERATOR_96MHz     (10u) /* Used only by D51 for any peripheral that has a 100MHz maximum peripheral clock. */
#define GENERIC_CLOCK_GENERATOR_UNUSED11  (11u) /* Unused for now. D51 only. */

// Constants when using a GCLK as a source to a PLL. Make sure thay are consistent with the constants above.
#define GCLK_PCHCTRL_GEN_XOSC  GCLK_PCHCTRL_GEN_GCLK1
#define GCLK_PCHCTRL_GEN_DFLL  GCLK_PCHCTRL_GEN_GCLK9

// Constants for Clock multiplexers
#if (SAMD21 || SAMD11 || SAML21)
  #define GENERIC_CLOCK_MULTIPLEXER_DFLL    (0u)
  #define GENERIC_CLOCK_MULTIPLEXER_FDPLL   (1u)
  #define GENERIC_CLOCK_MULTIPLEXER_FDPLL_32K (2u)
#elif (SAMD51)
  #define GENERIC_CLOCK_MULTIPLEXER_DFLL    (0u)
  #define GENERIC_CLOCK_MULTIPLEXER_FDPLL_0   (1u)
  #define GENERIC_CLOCK_MULTIPLEXER_FDPLL_1   (2u)
  #define GENERIC_CLOCK_MULTIPLEXER_FDPLL_0_32K (3u)
  #define GENERIC_CLOCK_MULTIPLEXER_FDPLL_1_32K (3u)
#elif (SAMC21)
  #define GENERIC_CLOCK_MULTIPLEXER_FDPLL   (0u)
  #define GENERIC_CLOCK_MULTIPLEXER_FDPLL_32K (1u)
#else
  #error "startup.c: Missing dependency or unsupported chip. Please install CMSIS-Atmel from MattairTech (see Prerequisites for Building in README.md)."
#endif

void waitForSync( void )
{
#if (SAMD21 || SAMD11)
  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );
#elif (SAML21 || SAMC21 || SAMD51)
  while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK );
#endif
}

#if (SAMD21 || SAMD11 || SAML21 || SAMD51)
void waitForDFLL( void )
{
#if (SAMD21 || SAMD11)
  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 );
#elif (SAML21 || SAMD51)
  while ( (OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY) == 0 );
#endif
}
#endif

#if (SAML21 || SAMC21 || SAMD51)
void waitForPLL( void )
{
#if (SAMD51)
  while ( OSCCTRL->Dpll[0].DPLLSYNCBUSY.reg & OSCCTRL_DPLLSYNCBUSY_MASK );
  while ( OSCCTRL->Dpll[1].DPLLSYNCBUSY.reg & OSCCTRL_DPLLSYNCBUSY_MASK );
#else
  while ( OSCCTRL->DPLLSYNCBUSY.reg & OSCCTRL_DPLLSYNCBUSY_MASK );
#endif
}
#endif

void SystemInit( void )
{
  /* Set 1 Flash Wait State for 48MHz (2 for the L21 and C21), cf tables 20.9 and 35.27 in SAMD21 Datasheet
   * The D51 runs with 5 wait states at 120MHz (automatic).
   * Disable automatic NVM write operations (errata reference 13134, applies to D21/D11/L21, but not C21 or D51)
   */
#if (SAMD21 || SAMD11)
  NVMCTRL->CTRLB.reg = (NVMCTRL_CTRLB_RWS_HALF | NVMCTRL_CTRLB_MANW) ; // one wait state
#elif (SAML21 || SAMC21)
  NVMCTRL->CTRLB.reg = (NVMCTRL_CTRLB_RWS_DUAL | NVMCTRL_CTRLB_MANW) ; // two wait states
#elif (SAMD51)
  NVMCTRL->CTRLA.reg = (NVMCTRL_CTRLA_WMODE_MAN | NVMCTRL_CTRLA_AUTOWS);        // auto wait states
  //NVMCTRL->CTRLA.bit.RWS = 5 ; // 5 wait states
#endif

#if (SAMD51) && defined(CORTEX_M_CACHE_ENABLED)
  CMCC->CTRL.reg = CMCC_CTRL_CEN;
#endif

  /* Turn on the digital interface clock */
#if !defined(TRUST_RESET_DEFAULTS)
#if (SAMD21 || SAMD11)
  PM->APBAMASK.reg |= PM_APBAMASK_GCLK ;
#elif (SAML21 || SAMC21 || SAMD51)
  MCLK->APBAMASK.reg |= MCLK_APBAMASK_GCLK ;
#endif
#endif

#if (SAML21)
  PM->INTFLAG.reg = PM_INTFLAG_PLRDY; //clear flag
  PM->PLCFG.reg |= PM_PLCFG_PLSEL_PL2 ;	// must set to highest performance level
  while ( (PM->INTFLAG.reg & PM_INTFLAG_PLRDY) != PM_INTFLAG_PLRDY );
  PM->INTFLAG.reg = PM_INTFLAG_PLRDY; //clear flag
#endif

  /* ----------------------------------------------------------------------------------------------
   * Software reset the GCLK module to ensure it is re-initialized correctly
   */
#if !defined(TRUST_RESET_DEFAULTS)
#if (SAMD21 || SAMD11)
  GCLK->CTRL.reg = GCLK_CTRL_SWRST ;

  while ( (GCLK->CTRL.reg & GCLK_CTRL_SWRST) && (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) );	/* Wait for reset to complete */
#elif (SAML21 || SAMC21 || SAMD51)
  GCLK->CTRLA.reg = GCLK_CTRLA_SWRST ;

  while ( (GCLK->CTRLA.reg & GCLK_CTRLA_SWRST) && (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK) );	/* Wait for reset to complete */
#endif
#endif

#if defined(CLOCKCONFIG_32768HZ_CRYSTAL)
  /* ----------------------------------------------------------------------------------------------
   * Enable XOSC32K clock (External on-board 32.768Hz crystal oscillator)
   */

#if defined(PLL_FRACTIONAL_ENABLE)
  #if (SAMD51)
    #if (VARIANT_MCK == 120000000ul)
      #define DPLLRATIO_LDR       3661u
      #define DPLLRATIO_LDRFRAC   3u
      #define DPLL1RATIO_LDR      5858u
      #define DPLL1RATIO_LDRFRAC  12u
    #else
      #define DPLLRATIO_LDR       5858u
      #define DPLLRATIO_LDRFRAC   12u
    #endif
  #else
    #define DPLLRATIO_LDR       2928u
    #define DPLLRATIO_LDRFRAC   11u
  #endif
#else
  #if (SAMD51)
    #if (VARIANT_MCK == 120000000ul)
      #define DPLLRATIO_LDR       3661u
      #define DPLLRATIO_LDRFRAC   0u
      #define DPLL1RATIO_LDR      5858u
      #define DPLL1RATIO_LDRFRAC  0u
    #else
      #define DPLLRATIO_LDR       5858u
      #define DPLLRATIO_LDRFRAC   0u
    #endif
  #else
    #define DPLLRATIO_LDR       2929u
    #define DPLLRATIO_LDRFRAC   0u
  #endif
#endif

#if (SAMD21 || SAMD11)
  SYSCTRL->XOSC32K.reg = (SYSCTRL_XOSC32K_STARTUP( 0x6u ) | SYSCTRL_XOSC32K_XTALEN | SYSCTRL_XOSC32K_EN32K);
  SYSCTRL->XOSC32K.bit.ENABLE = 1; /* separate call, as described in chapter 15.6.3 */

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_XOSC32KRDY) == 0 );     /* Wait for oscillator stabilization */

  SYSCTRL->DPLLRATIO.reg = ( SYSCTRL_DPLLRATIO_LDR(DPLLRATIO_LDR) | SYSCTRL_DPLLRATIO_LDRFRAC(DPLLRATIO_LDRFRAC) );  /* set PLL multiplier */

  SYSCTRL->DPLLCTRLB.reg = SYSCTRL_DPLLCTRLB_REFCLK(0);  /* select 32KHz crystal input */

  SYSCTRL->DPLLCTRLA.reg = SYSCTRL_DPLLCTRLA_ENABLE;

  while ( (SYSCTRL->DPLLSTATUS.reg & SYSCTRL_DPLLSTATUS_CLKRDY) != SYSCTRL_DPLLSTATUS_CLKRDY );

#elif (SAML21 || SAMC21)
  OSC32KCTRL->XOSC32K.reg = (OSC32KCTRL_XOSC32K_STARTUP( 0x4u ) | OSC32KCTRL_XOSC32K_XTALEN | OSC32KCTRL_XOSC32K_EN32K | OSC32KCTRL_XOSC32K_EN1K);
  OSC32KCTRL->XOSC32K.bit.ENABLE = 1;

  while ( (OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY) == 0 );       /* Wait for oscillator stabilization */

  OSCCTRL->DPLLRATIO.reg = ( OSCCTRL_DPLLRATIO_LDR(DPLLRATIO_LDR) | OSCCTRL_DPLLRATIO_LDRFRAC(DPLLRATIO_LDRFRAC) );  /* set PLL multiplier */
  waitForPLL();

  OSCCTRL->DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_REFCLK(0);  /* select 32KHz crystal input */

  OSCCTRL->DPLLPRESC.reg = 0;
  waitForPLL();

  OSCCTRL->DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE;
  waitForPLL();

  while ( (OSCCTRL->DPLLSTATUS.reg & OSCCTRL_DPLLSTATUS_CLKRDY) != OSCCTRL_DPLLSTATUS_CLKRDY );

#elif (SAMD51)
  // Use high gain for 32.768KHz crystal
  OSC32KCTRL->XOSC32K.reg = (OSC32KCTRL_XOSC32K_STARTUP( 0x4u ) | OSC32KCTRL_XOSC32K_XTALEN | OSC32KCTRL_XOSC32K_EN32K | OSC32KCTRL_XOSC32K_EN1K | OSC32KCTRL_XOSC32K_CGM_HS);
  OSC32KCTRL->XOSC32K.bit.ENABLE = 1;

  while ( (OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY) == 0 );       /* Wait for oscillator stabilization */

  OSCCTRL->Dpll[0].DPLLRATIO.reg = ( OSCCTRL_DPLLRATIO_LDR(DPLLRATIO_LDR) | OSCCTRL_DPLLRATIO_LDRFRAC(DPLLRATIO_LDRFRAC) );  /* set PLL multiplier */
  waitForPLL();

  OSCCTRL->Dpll[0].DPLLCTRLB.reg = (OSCCTRL_DPLLCTRLB_REFCLK(1) | OSCCTRL_DPLLCTRLB_LBYPASS);  /* select 32KHz crystal input, must use LBYPASS (see errata) */

  OSCCTRL->Dpll[0].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE;
  waitForPLL();

  while ( (OSCCTRL->Dpll[0].DPLLSTATUS.reg & OSCCTRL_DPLLSTATUS_LOCK) != OSCCTRL_DPLLSTATUS_LOCK );

  /* If the CPU will run at 120MHz using the first PLL, setup the second PLL to generate 192MHz, which is divided down to 96MHz and 48MHz for use by USB, etc. */
  #if (VARIANT_MCK == 120000000ul)
    OSCCTRL->Dpll[1].DPLLRATIO.reg = ( OSCCTRL_DPLLRATIO_LDR(DPLL1RATIO_LDR) | OSCCTRL_DPLLRATIO_LDRFRAC(DPLL1RATIO_LDRFRAC) );  /* set PLL multiplier */
    waitForPLL();

    OSCCTRL->Dpll[1].DPLLCTRLB.reg = (OSCCTRL_DPLLCTRLB_REFCLK(1) | OSCCTRL_DPLLCTRLB_LBYPASS);  /* select 32KHz crystal input, must use LBYPASS (see errata) */

    OSCCTRL->Dpll[1].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE;
    waitForPLL();

    while ( (OSCCTRL->Dpll[1].DPLLSTATUS.reg & OSCCTRL_DPLLSTATUS_LOCK) != OSCCTRL_DPLLSTATUS_LOCK );
  #endif
#endif

#elif defined(CLOCKCONFIG_HS_CRYSTAL)
  /* ----------------------------------------------------------------------------------------------
   * Enable XOSC clock (External on-board high speed crystal oscillator)
   */

#if (SAMD51)
  #if ((HS_CRYSTAL_FREQUENCY_HERTZ < 8000000UL) || (HS_CRYSTAL_FREQUENCY_HERTZ > 48000000UL))
    #error "board.init.c: HS_CRYSTAL_FREQUENCY_HERTZ must be between 8000000UL and 48000000UL"
  #endif
#else
  #if ((HS_CRYSTAL_FREQUENCY_HERTZ < 400000UL) || (HS_CRYSTAL_FREQUENCY_HERTZ > 32000000UL))
    #error "board.init.c: HS_CRYSTAL_FREQUENCY_HERTZ must be between 400000UL and 32000000UL"
  #endif
#endif

#if defined(PLL_FAST_STARTUP)
  #if (HS_CRYSTAL_FREQUENCY_HERTZ < 1000000UL)
    #error "board.init.c: HS_CRYSTAL_FREQUENCY_HERTZ must be at least 1000000UL when PLL_FAST_STARTUP is defined"
  #else
    #define HS_CRYSTAL_DIVISOR  1000000UL
  #endif
#else
  #define HS_CRYSTAL_DIVISOR    32000UL
#endif

// All floating point math done by C preprocessor
#define HS_CRYSTAL_DIVIDER      (HS_CRYSTAL_FREQUENCY_HERTZ / HS_CRYSTAL_DIVISOR)
#if (SAMD51)
  #if (VARIANT_MCK == 120000000ul)
    #define DPLLRATIO_FLOAT       (120000000.0 / ((float)HS_CRYSTAL_FREQUENCY_HERTZ / HS_CRYSTAL_DIVIDER))
    #define DPLL1RATIO_FLOAT      (192000000.0 / ((float)HS_CRYSTAL_FREQUENCY_HERTZ / HS_CRYSTAL_DIVIDER))
  #else
    #define DPLLRATIO_FLOAT       (192000000.0 / ((float)HS_CRYSTAL_FREQUENCY_HERTZ / HS_CRYSTAL_DIVIDER))
  #endif
#else
  #define DPLLRATIO_FLOAT       (96000000.0 / ((float)HS_CRYSTAL_FREQUENCY_HERTZ / HS_CRYSTAL_DIVIDER))
#endif

#if defined(PLL_FRACTIONAL_ENABLED)
  #define DPLLRATIO_LDR         ((uint16_t)DPLLRATIO_FLOAT - 1)
  #if (SAMD51)
    #define DPLLRATIO_LDRFRAC   (uint8_t)((DPLLRATIO_FLOAT - (uint16_t)DPLLRATIO_FLOAT) * 32.0)
    #if (VARIANT_MCK == 120000000ul)
      #define DPLL1RATIO_LDRFRAC   (uint8_t)((DPLL1RATIO_FLOAT - (uint16_t)DPLL1RATIO_FLOAT) * 32.0)
    #endif
  #else
    #define DPLLRATIO_LDRFRAC   (uint8_t)((DPLLRATIO_FLOAT - (uint16_t)DPLLRATIO_FLOAT) * 16.0)
  #endif
#else
  #define DPLLRATIO_LDR         ((uint16_t)DPLLRATIO_FLOAT - 1)
  #define DPLLRATIO_LDRFRAC     0
  #if (SAMD51 && (VARIANT_MCK == 120000000ul))
    #define DPLL1RATIO_LDR        ((uint16_t)DPLL1RATIO_FLOAT - 1)
    #define DPLL1RATIO_LDRFRAC    0
  #endif
#endif

#if (SAMD21 || SAMD11)
  SYSCTRL->XOSC.reg = (SYSCTRL_XOSC_STARTUP( 0x8u ) | SYSCTRL_XOSC_GAIN( 0x4u ) | SYSCTRL_XOSC_XTALEN | SYSCTRL_XOSC_ENABLE) ;  // startup time is 8ms
  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_XOSCRDY) == 0 );        /* Wait for oscillator stabilization */

  SYSCTRL->XOSC.reg |= SYSCTRL_XOSC_AMPGC ;     // set only after startup time

  /* Connect GCLK1 to XOSC and set prescaler */
  GCLK->GENDIV.reg = ( GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_XOSC ) | GCLK_GENDIV_DIV(HS_CRYSTAL_DIVIDER) ) ; // Set divider for generic clock generator 1
  waitForSync();

  GCLK->GENCTRL.reg = ( GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_XOSC ) | GCLK_GENCTRL_SRC_XOSC | GCLK_GENCTRL_GENEN );
  waitForSync();

  /* Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 1 (FDPLL reference) */
  GCLK->CLKCTRL.reg = ( GCLK_CLKCTRL_ID( GENERIC_CLOCK_MULTIPLEXER_FDPLL ) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN );
  waitForSync();

  /* Configure PLL */
  SYSCTRL->DPLLRATIO.reg = ( SYSCTRL_DPLLRATIO_LDR(DPLLRATIO_LDR) | SYSCTRL_DPLLRATIO_LDRFRAC(DPLLRATIO_LDRFRAC) ) ;  /* set PLL multiplier */

  SYSCTRL->DPLLCTRLB.reg = SYSCTRL_DPLLCTRLB_REFCLK(2) ;  /* select GCLK input */

  SYSCTRL->DPLLCTRLA.reg = SYSCTRL_DPLLCTRLA_ENABLE ;

  while ( (SYSCTRL->DPLLSTATUS.reg & SYSCTRL_DPLLSTATUS_CLKRDY) != SYSCTRL_DPLLSTATUS_CLKRDY );

#elif (SAML21 || SAMC21)
  OSCCTRL->XOSCCTRL.reg = (OSCCTRL_XOSCCTRL_STARTUP( 0x8u ) | OSCCTRL_XOSCCTRL_GAIN( 0x4u ) | OSCCTRL_XOSCCTRL_XTALEN | OSCCTRL_XOSCCTRL_ENABLE) ;      // startup time is 8ms
  while ( (OSCCTRL->STATUS.reg & OSCCTRL_STATUS_XOSCRDY) == 0 );        /* Wait for oscillator stabilization */

  OSCCTRL->XOSCCTRL.reg |= OSCCTRL_XOSCCTRL_AMPGC ;     // set only after startup time

  /* Connect GCLK1 to XOSC and set prescaler */
  GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_XOSC].reg = ( GCLK_GENCTRL_DIV(HS_CRYSTAL_DIVIDER) | GCLK_GENCTRL_SRC_XOSC | GCLK_GENCTRL_GENEN );
  waitForSync();

  /* Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 1 (FDPLL reference) */
  GCLK->PCHCTRL[GENERIC_CLOCK_MULTIPLEXER_FDPLL].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_XOSC );
  while ( (GCLK->PCHCTRL[GENERIC_CLOCK_MULTIPLEXER_FDPLL].reg & GCLK_PCHCTRL_CHEN) != GCLK_PCHCTRL_CHEN );      // wait for sync

  /* Configure PLL */
  OSCCTRL->DPLLRATIO.reg = ( OSCCTRL_DPLLRATIO_LDR(DPLLRATIO_LDR) | OSCCTRL_DPLLRATIO_LDRFRAC(DPLLRATIO_LDRFRAC) ) ;  /* set PLL multiplier */
  waitForPLL();

  OSCCTRL->DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_REFCLK(2) ;  /* select GCLK input */

  OSCCTRL->DPLLPRESC.reg = 0;
  waitForPLL();

  OSCCTRL->DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE ;
  waitForPLL();

  while ( (OSCCTRL->DPLLSTATUS.reg & OSCCTRL_DPLLSTATUS_CLKRDY) != OSCCTRL_DPLLSTATUS_CLKRDY );

#elif (SAMD51)
  OSCCTRL->XOSCCTRL[0].reg = (OSCCTRL_XOSCCTRL_STARTUP( 0x8u ) | OSCCTRL_XOSCCTRL_IPTAT(3) | OSCCTRL_XOSCCTRL_IMULT(4) | OSCCTRL_XOSCCTRL_XTALEN | OSCCTRL_XOSCCTRL_ENABLE) ;      // startup time is 8ms
  while ( (OSCCTRL->STATUS.reg & OSCCTRL_STATUS_XOSCRDY0) == 0 );        /* Wait for oscillator stabilization */

  OSCCTRL->XOSCCTRL[0].reg |= OSCCTRL_XOSCCTRL_ENALC | OSCCTRL_XOSCCTRL_LOWBUFGAIN ;     // Enable automatic loop compensation

  /* Connect GCLK1 to XOSC and set prescaler */
  GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_XOSC].reg = ( GCLK_GENCTRL_DIV(HS_CRYSTAL_DIVIDER) | GCLK_GENCTRL_SRC_XOSC0 | GCLK_GENCTRL_GENEN );
  waitForSync();

  /* Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 1 (FDPLL reference) */
  GCLK->PCHCTRL[GENERIC_CLOCK_MULTIPLEXER_FDPLL_0].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_XOSC );
  while ( (GCLK->PCHCTRL[GENERIC_CLOCK_MULTIPLEXER_FDPLL_0].reg & GCLK_PCHCTRL_CHEN) != GCLK_PCHCTRL_CHEN );      // wait for sync

  /* Configure PLL */
  OSCCTRL->Dpll[0].DPLLRATIO.reg = ( OSCCTRL_DPLLRATIO_LDR(DPLLRATIO_LDR) | OSCCTRL_DPLLRATIO_LDRFRAC(DPLLRATIO_LDRFRAC) ) ;  /* set PLL multiplier */
  waitForPLL();

  OSCCTRL->Dpll[0].DPLLCTRLB.reg = (OSCCTRL_DPLLCTRLB_REFCLK_GCLK | OSCCTRL_DPLLCTRLB_LBYPASS) ;  /* select GCLK input, must use LBYPASS (see errata) */

  OSCCTRL->Dpll[0].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE ;
  waitForPLL();

  while ( (OSCCTRL->Dpll[0].DPLLSTATUS.reg & OSCCTRL_DPLLSTATUS_LOCK) != OSCCTRL_DPLLSTATUS_LOCK );

  /* If the CPU will run at 120MHz using the first PLL, setup the second PLL to generate 192MHz, which is divided down to 96MHz and 48MHz for use by USB, etc. */
  #if (VARIANT_MCK == 120000000ul)
    GCLK->PCHCTRL[GENERIC_CLOCK_MULTIPLEXER_FDPLL_1].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_XOSC );
    while ( (GCLK->PCHCTRL[GENERIC_CLOCK_MULTIPLEXER_FDPLL_1].reg & GCLK_PCHCTRL_CHEN) != GCLK_PCHCTRL_CHEN );      // wait for sync

    /* Configure PLL */
    OSCCTRL->Dpll[1].DPLLRATIO.reg = ( OSCCTRL_DPLLRATIO_LDR(DPLL1RATIO_LDR) | OSCCTRL_DPLLRATIO_LDRFRAC(DPLL1RATIO_LDRFRAC) );  /* set PLL multiplier */
    waitForPLL();

    OSCCTRL->Dpll[1].DPLLCTRLB.reg = (OSCCTRL_DPLLCTRLB_REFCLK_GCLK | OSCCTRL_DPLLCTRLB_LBYPASS) ;  /* select GCLK input, must use LBYPASS (see errata) */

    OSCCTRL->Dpll[1].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE;
    waitForPLL();

    while ( (OSCCTRL->Dpll[1].DPLLSTATUS.reg & OSCCTRL_DPLLSTATUS_LOCK) != OSCCTRL_DPLLSTATUS_LOCK );
  #endif
#endif

#elif (defined(CLOCKCONFIG_INTERNAL) || defined(CLOCKCONFIG_INTERNAL_USB))
  /* ----------------------------------------------------------------------------------------------
   * Enable DFLL48M clock (D21/L21) or RC oscillator (C21)
   */
#if (SAMD21 || SAMD11)
  /* Remove the OnDemand mode, Bug http://avr32.icgroup.norway.atmel.com/bugzilla/show_bug.cgi?id=9905 */
  SYSCTRL->DFLLCTRL.bit.ONDEMAND = 0 ;
  waitForDFLL();

  /* Load NVM Coarse calibration value */
  uint32_t calib = (*((uint32_t *) FUSES_DFLL48M_COARSE_CAL_ADDR) & FUSES_DFLL48M_COARSE_CAL_Msk) >> FUSES_DFLL48M_COARSE_CAL_Pos;
  SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE(calib) | SYSCTRL_DFLLVAL_FINE(512);

  /* Write full configuration to DFLL control register */
  #if defined(CLOCKCONFIG_INTERNAL_USB)
    SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP( 31 ) | // Coarse step is 31, half of the max value
                           SYSCTRL_DFLLMUL_FSTEP( 0xA ) | // value from datasheet USB Characteristics
                           SYSCTRL_DFLLMUL_MUL( 0xBB80 ) ; // 1KHz USB SOF signal (48MHz Fcpu / 1KHz SOF)

    SYSCTRL->DFLLCTRL.reg =  SYSCTRL_DFLLCTRL_USBCRM | /* USB correction */
                             SYSCTRL_DFLLCTRL_MODE | /* Closed loop mode */
                             SYSCTRL_DFLLCTRL_CCDIS ;
    waitForDFLL();
  #endif

  /* Enable the DFLL */
  SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_ENABLE ;
  waitForDFLL();

#elif (SAML21)
  /* Defines missing from CMSIS */
  #ifndef FUSES_DFLL48M_COARSE_CAL_ADDR
    #define FUSES_DFLL48M_COARSE_CAL_ADDR (NVMCTRL_OTP5)
  #endif
  #ifndef FUSES_DFLL48M_COARSE_CAL_Pos
    #define FUSES_DFLL48M_COARSE_CAL_Pos 26
  #endif
  #ifndef FUSES_DFLL48M_COARSE_CAL_Msk
    #define FUSES_DFLL48M_COARSE_CAL_Msk (0x3Ful << FUSES_DFLL48M_COARSE_CAL_Pos)
  #endif

  OSCCTRL->DFLLCTRL.bit.ONDEMAND = 0 ;
  waitForDFLL();

  /* Load NVM Coarse calibration value */
  uint32_t calib = (*((uint32_t *) FUSES_DFLL48M_COARSE_CAL_ADDR) & FUSES_DFLL48M_COARSE_CAL_Msk) >> FUSES_DFLL48M_COARSE_CAL_Pos;
  OSCCTRL->DFLLVAL.reg = OSCCTRL_DFLLVAL_COARSE(calib) | OSCCTRL_DFLLVAL_FINE(512);

  /* Write full configuration to DFLL control register */
  #if defined(CLOCKCONFIG_INTERNAL_USB)
    OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_CSTEP( 31 ) | // Coarse step is 31, half of the max value
                           OSCCTRL_DFLLMUL_FSTEP( 0xA ) | // value from datasheet USB Characteristics
                           OSCCTRL_DFLLMUL_MUL( 0xBB80 ) ; // 1KHz USB SOF signal (48MHz Fcpu / 1KHz SOF)

    OSCCTRL->DFLLCTRL.reg =  OSCCTRL_DFLLCTRL_USBCRM | /* USB correction */
                             OSCCTRL_DFLLCTRL_MODE | /* Closed loop mode */
                             OSCCTRL_DFLLCTRL_CCDIS ;
    waitForDFLL();
  #endif

  /* Enable the DFLL */
  OSCCTRL->DFLLCTRL.reg |= OSCCTRL_DFLLCTRL_ENABLE ;
  waitForDFLL();

#elif (SAMC21)
  #if defined(CLOCKCONFIG_INTERNAL_USB)
    #error "startup.c: CLOCKCONFIG_INTERNAL_USB setting invalid for C21 chips as they lack USB."
  #endif

  /* Change OSC48M divider to /1. CPU will run at 48MHz */
  OSCCTRL->OSC48MDIV.reg = OSCCTRL_OSC48MDIV_DIV(0);
  while ( OSCCTRL->OSC48MSYNCBUSY.reg & OSCCTRL_OSC48MSYNCBUSY_OSC48MDIV );

#elif (SAMD51)
  OSCCTRL->DFLLCTRLA.bit.ONDEMAND = 0 ;
  waitForDFLL();

  /* Write full configuration to DFLL control register */
  #if defined(CLOCKCONFIG_CLOCK_SOURCE) && (CLOCKCONFIG_CLOCK_SOURCE == CLOCKCONFIG_INTERNAL_USB)
    OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_CSTEP( 31 ) | // Coarse step is 31, half of the max value
                           OSCCTRL_DFLLMUL_FSTEP( 0xA ) | // value from datasheet USB Characteristics
                           OSCCTRL_DFLLMUL_MUL( 0xBB80 ) ; // 1KHz USB SOF signal (48MHz Fcpu / 1KHz SOF)

    OSCCTRL->DFLLCTRLB.reg =  OSCCTRL_DFLLCTRLB_USBCRM | /* USB correction */
                             OSCCTRL_DFLLCTRLB_MODE | /* Closed loop mode */
                             OSCCTRL_DFLLCTRLB_CCDIS ;
    waitForDFLL();
  #endif

  /* Enable the DFLL */
  OSCCTRL->DFLLCTRLA.reg |= OSCCTRL_DFLLCTRLA_ENABLE ;
  waitForDFLL();

  /* Switch Generic Clock Generator 4 to DFLL48M and divide down to 2MHz (to be used as PLL input) */
  GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_DFLL].reg = ( GCLK_GENCTRL_DIV(24) | GCLK_GENCTRL_SRC_DFLL | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
  waitForSync();

  /* Configure PLL0 to generate 120MHz for the CPU. */
  #if (VARIANT_MCK == 120000000ul)
    GCLK->PCHCTRL[GENERIC_CLOCK_MULTIPLEXER_FDPLL_0].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_DFLL );
    while ( (GCLK->PCHCTRL[GENERIC_CLOCK_MULTIPLEXER_FDPLL_0].reg & GCLK_PCHCTRL_CHEN) != GCLK_PCHCTRL_CHEN );      // wait for sync

    OSCCTRL->Dpll[0].DPLLRATIO.reg = ( OSCCTRL_DPLLRATIO_LDR(59) | OSCCTRL_DPLLRATIO_LDRFRAC(0) ) ;  /* set PLL multiplier */
    waitForPLL();

    OSCCTRL->Dpll[0].DPLLCTRLB.reg = (OSCCTRL_DPLLCTRLB_REFCLK_GCLK | OSCCTRL_DPLLCTRLB_LBYPASS) ;  /* select GCLK input, must use LBYPASS (see errata) */

    OSCCTRL->Dpll[0].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE ;
    waitForPLL();

    while ( (OSCCTRL->Dpll[0].DPLLSTATUS.reg & OSCCTRL_DPLLSTATUS_LOCK) != OSCCTRL_DPLLSTATUS_LOCK );
  #endif

  /* Configure PLL1 to generate 192MHz. */
  GCLK->PCHCTRL[GENERIC_CLOCK_MULTIPLEXER_FDPLL_1].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_DFLL );
  while ( (GCLK->PCHCTRL[GENERIC_CLOCK_MULTIPLEXER_FDPLL_1].reg & GCLK_PCHCTRL_CHEN) != GCLK_PCHCTRL_CHEN );      // wait for sync

  OSCCTRL->Dpll[1].DPLLRATIO.reg = ( OSCCTRL_DPLLRATIO_LDR(95) | OSCCTRL_DPLLRATIO_LDRFRAC(0) );  /* set PLL multiplier */
  waitForPLL();

  OSCCTRL->Dpll[1].DPLLCTRLB.reg = (OSCCTRL_DPLLCTRLB_REFCLK_GCLK | OSCCTRL_DPLLCTRLB_LBYPASS) ;  /* select GCLK input, must use LBYPASS (see errata) */

  OSCCTRL->Dpll[1].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE;
  waitForPLL();

  while ( (OSCCTRL->Dpll[1].DPLLSTATUS.reg & OSCCTRL_DPLLSTATUS_LOCK) != OSCCTRL_DPLLSTATUS_LOCK );

#endif
#else
  #error "startup.c: Clock source must be selected in the boards.txt file (normally through the Tools menu)."
#endif


/* Set CPU and APB dividers before switching the CPU/APB clocks to the new clock source */
#if !defined(TRUST_RESET_DEFAULTS)
#if (SAMD21 || SAMD11)
  PM->CPUSEL.reg  = PM_CPUSEL_CPUDIV_DIV1 ;
  PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV1_Val ;
  PM->APBBSEL.reg = PM_APBBSEL_APBBDIV_DIV1_Val ;
  PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1_Val ;
#elif (SAML21 || SAMC21)
  MCLK->CPUDIV.reg  = MCLK_CPUDIV_CPUDIV_DIV1 ;
#elif (SAMD51)
  MCLK->CPUDIV.reg  = MCLK_CPUDIV_DIV_DIV1 ;
#endif
#endif


/* Setup GCLK0 (GENERIC_CLOCK_GENERATOR_MAIN) which is used for the CPU/APB, as well as other required GCLKs. */
#if (SAMD21 || SAMD11)
  #if (defined(CLOCKCONFIG_32768HZ_CRYSTAL) || defined(CLOCKCONFIG_HS_CRYSTAL))
    /* Switch Generic Clock Generator 0 to 96MHz PLL output. The output is divided by two to obtain a 48MHz CPU clock. */
    GCLK->GENDIV.reg = ( GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_MAIN ) | GCLK_GENDIV_DIV(2) );
    waitForSync();
    GCLK->GENCTRL.reg = ( GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_MAIN ) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
    waitForSync();
  #elif (defined(CLOCKCONFIG_INTERNAL) || defined(CLOCKCONFIG_INTERNAL_USB))
    /* Switch Generic Clock Generator 0 to 48MHz DFLL48M output. The output is undivided. */
    GCLK->GENDIV.reg = ( GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_MAIN ) | GCLK_GENDIV_DIV(1) );
    waitForSync();
    GCLK->GENCTRL.reg = ( GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_MAIN ) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
    waitForSync();
  #endif

#elif (SAML21 || SAMC21)
  /* Setup GCLK0 (GENERIC_CLOCK_GENERATOR_MAIN) which is used for the CPU. */
  #if (defined(CLOCKCONFIG_32768HZ_CRYSTAL) || defined(CLOCKCONFIG_HS_CRYSTAL))
    /* Switch Generic Clock Generator 0 to 96MHz PLL output. The output is divided by two to obtain a 48MHz CPU clock. */
    GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_MAIN].reg = ( GCLK_GENCTRL_DIV(2) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
    waitForSync();
  #elif (defined(CLOCKCONFIG_INTERNAL) || defined(CLOCKCONFIG_INTERNAL_USB))
    /* Note that the C21 is already setup above */
    #if (SAML21)
      /* Switch Generic Clock Generator 0 to 48MHz DFLL48M output. The output is undivided. */
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_MAIN].reg = ( GCLK_GENCTRL_DIV(1) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      waitForSync();
    #endif
  #endif

#elif (SAMD51)
  #if (defined(CLOCKCONFIG_32768HZ_CRYSTAL) || defined(CLOCKCONFIG_HS_CRYSTAL))
    /* If the CPU will run at 120MHz using the first PLL, setup the second PLL to generate 192MHz, which is divided down to 96MHz and 48MHz for use by USB, etc. */
    #if (VARIANT_MCK == 120000000ul)
      /* Switch Generic Clock Generator 0 to PLL0. The CPU will run at 120MHz. */
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_MAIN].reg = ( GCLK_GENCTRL_DIV(1) | GCLK_GENCTRL_SRC_DPLL0 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      waitForSync();
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_96MHz].reg = ( GCLK_GENCTRL_DIV(2) | GCLK_GENCTRL_SRC_DPLL1 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      waitForSync();
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_48MHz].reg = ( GCLK_GENCTRL_DIV(4) | GCLK_GENCTRL_SRC_DPLL1 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      waitForSync();
    #else
      /* Set CPU/AHB/APB dividers before switching them to GCLK0, which runs at 96MHz, so that the CPU will operate at 48MHz. Note that HSDIV is
       * set to 1 by default so that CLK_APB_HS is 96MHz, which is double the CPU clock of 48MHz, for use with the QSPI 2X clock for DDR support.
       */
      MCLK->CPUDIV.reg = MCLK_CPUDIV_DIV_DIV2;

      /* Switch Generic Clock Generator 0 to PLL0. The CPU will run at 48MHz. */
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_MAIN].reg = ( GCLK_GENCTRL_DIV(2) | GCLK_GENCTRL_SRC_DPLL0 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      waitForSync();
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_96MHz].reg = ( GCLK_GENCTRL_DIV(2) | GCLK_GENCTRL_SRC_DPLL0 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      waitForSync();
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_48MHz].reg = ( GCLK_GENCTRL_DIV(4) | GCLK_GENCTRL_SRC_DPLL0 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      waitForSync();
    #endif
  #elif (defined(CLOCKCONFIG_INTERNAL) || defined(CLOCKCONFIG_INTERNAL_USB))
    /* PLL1 is used to generate 192MHz. If the CPU will run at 120MHz, use PLL0 as well. */
    #if (VARIANT_MCK == 120000000ul)
      /* Switch Generic Clock Generator 0 to PLL0. CPU will run at 120MHz. */
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_MAIN].reg = ( GCLK_GENCTRL_DIV(1) | GCLK_GENCTRL_SRC_DPLL0 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      waitForSync();
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_96MHz].reg = ( GCLK_GENCTRL_DIV(2) | GCLK_GENCTRL_SRC_DPLL1 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      waitForSync();
    #else
      /* Set CPU/AHB/APB dividers before switching them to GCLK0, which runs at 96MHz, so that the CPU will operate at 48MHz. Note that HSDIV is
       * set to 1 by default so that CLK_APB_HS is 96MHz, which is double the CPU clock of 48MHz, for use with the QSPI 2X clock for DDR support.
       */
      MCLK->CPUDIV.reg = MCLK_CPUDIV_DIV_DIV2;

      /* Switch Generic Clock Generator 0 to PLL0. CPU will run at 48MHz. */
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_MAIN].reg = ( GCLK_GENCTRL_DIV(2) | GCLK_GENCTRL_SRC_DPLL1 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      waitForSync();
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_96MHz].reg = ( GCLK_GENCTRL_DIV(2) | GCLK_GENCTRL_SRC_DPLL1 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      waitForSync();
    #endif

    GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_48MHz].reg = ( GCLK_GENCTRL_DIV(1) | GCLK_GENCTRL_SRC_DFLL | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
    waitForSync();
  #endif
#endif


/* Setup GCLK5 (GENERIC_CLOCK_GENERATOR_TIMERS) which is used by the timers for setting PWM output frequency,
 * unless using 732Hz or 187500Hz with the D21/D11/L21/C21 (in this case, the timers connect to GCLK0 (MAIN)).
 */
#if (defined(TIMER_93750Hz) || defined(TIMER_366Hz))
  #define TIMER_GCLK_DIVIDER_FACTOR    2
#elif (defined(TIMER_62500Hz) || defined(TIMER_244Hz))
  #define TIMER_GCLK_DIVIDER_FACTOR    3
#elif defined(TIMER_183Hz)
  #define TIMER_GCLK_DIVIDER_FACTOR    4
#elif (defined(TIMER_37500Hz) || defined(TIMER_146Hz))
  #define TIMER_GCLK_DIVIDER_FACTOR    5
#elif defined(TIMER_122Hz)
  #define TIMER_GCLK_DIVIDER_FACTOR    6
#elif defined(TIMER_105Hz)
  #define TIMER_GCLK_DIVIDER_FACTOR    7
#elif (defined(TIMER_20833Hz) || defined(TIMER_81Hz))
  #define TIMER_GCLK_DIVIDER_FACTOR    9
#elif defined(TIMER_61Hz)
  #define TIMER_GCLK_DIVIDER_FACTOR    12
#elif defined(TIMER_12500Hz)
  #define TIMER_GCLK_DIVIDER_FACTOR    15
#elif defined(TIMER_31Hz)
  #define TIMER_GCLK_DIVIDER_FACTOR    24
#elif defined(TIMER_7500Hz)
  #define TIMER_GCLK_DIVIDER_FACTOR    25
#elif defined(TIMER_4166Hz)
  #define TIMER_GCLK_DIVIDER_FACTOR    45
#elif defined(TIMER_2930Hz)
  #if (SAMD51)
    // This will use the power-of-two divider, rather than the integer divider
    #define TIMER_GCLK_DIVIDER_FACTOR    7
  #else
    #define TIMER_GCLK_DIVIDER_FACTOR    64
  #endif
#elif defined(TIMER_1465Hz)
  #if (SAMD51)
    // This will use 16-bit
    #define TIMER_GCLK_DIVIDER_FACTOR    2
  #elif (defined(CLOCKCONFIG_32768HZ_CRYSTAL) || defined(CLOCKCONFIG_HS_CRYSTAL))
    // This will use the power-of-two divider, rather than the integer divider
    #define TIMER_GCLK_DIVIDER_FACTOR    7
  #else
    #define TIMER_GCLK_DIVIDER_FACTOR    128
  #endif
#else
  #define TIMER_GCLK_DIVIDER_FACTOR    1
#endif

#if (SAMD51 || (!defined(TIMER_732Hz) && !defined(TIMER_187500Hz)))
#if (SAMD21 || SAMD11)
  #if (defined(CLOCKCONFIG_32768HZ_CRYSTAL) || defined(CLOCKCONFIG_HS_CRYSTAL))
  /* Switch GENERIC_CLOCK_GENERATOR_TIMERS to 96MHz PLL output and divide down to the selected frequency. Use the power-of-two divider with TIMER_1465Hz. */
    #if defined(TIMER_1465Hz)
      GCLK->GENDIV.reg = ( GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_TIMERS ) | GCLK_GENDIV_DIV(TIMER_GCLK_DIVIDER_FACTOR) );
      waitForSync();
      GCLK->GENCTRL.reg = ( GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_TIMERS ) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_DIVSEL );
    #else
      GCLK->GENDIV.reg = ( GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_TIMERS ) | GCLK_GENDIV_DIV(TIMER_GCLK_DIVIDER_FACTOR * 2) );
      waitForSync();
      GCLK->GENCTRL.reg = ( GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_TIMERS ) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
    #endif
    waitForSync();
  #elif (defined(CLOCKCONFIG_INTERNAL) || defined(CLOCKCONFIG_INTERNAL_USB))
    /* Switch GENERIC_CLOCK_GENERATOR_TIMERS to 48MHz DFLL48M output and divide down to the selected frequency. */
    GCLK->GENDIV.reg = ( GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_TIMERS ) | GCLK_GENDIV_DIV(TIMER_GCLK_DIVIDER_FACTOR) );
    waitForSync();
    GCLK->GENCTRL.reg = ( GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_TIMERS ) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
    waitForSync();
  #endif

#elif (SAML21 || SAMC21)
  #if (defined(CLOCKCONFIG_32768HZ_CRYSTAL) || defined(CLOCKCONFIG_HS_CRYSTAL))
    /* Switch GENERIC_CLOCK_GENERATOR_TIMERS to 96MHz PLL output and divide down to the selected frequency. Use the power-of-two divider with TIMER_1465Hz. */
    #if defined(TIMER_1465Hz)
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_TIMERS].reg = ( GCLK_GENCTRL_DIV(TIMER_GCLK_DIVIDER_FACTOR) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_DIVSEL );
    #else
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_TIMERS].reg = ( GCLK_GENCTRL_DIV(TIMER_GCLK_DIVIDER_FACTOR * 2) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
    #endif
    waitForSync();
  #elif (defined(CLOCKCONFIG_INTERNAL) || defined(CLOCKCONFIG_INTERNAL_USB))
    /* Switch GENERIC_CLOCK_GENERATOR_TIMERS to 48MHz DFLL48M output and divide down to the selected frequency. */
    GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_TIMERS].reg = ( GCLK_GENCTRL_DIV(TIMER_GCLK_DIVIDER_FACTOR) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
    waitForSync();
  #endif

#elif (SAMD51)
  #if (defined(CLOCKCONFIG_32768HZ_CRYSTAL) || defined(CLOCKCONFIG_HS_CRYSTAL))
    #if (VARIANT_MCK == 120000000ul)
      #if defined(TIMER_2930Hz)
        GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_TIMERS].reg = ( GCLK_GENCTRL_DIV(TIMER_GCLK_DIVIDER_FACTOR) | GCLK_GENCTRL_SRC_DPLL1 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_DIVSEL );
      #elif defined(TIMER_1465Hz)
        GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_TIMERS].reg = ( GCLK_GENCTRL_DIV(TIMER_GCLK_DIVIDER_FACTOR) | GCLK_GENCTRL_SRC_DPLL1 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      #else
        GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_TIMERS].reg = ( GCLK_GENCTRL_DIV(TIMER_GCLK_DIVIDER_FACTOR * 4) | GCLK_GENCTRL_SRC_DPLL1 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      #endif
    #else
      #if defined(TIMER_2930Hz)
        GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_TIMERS].reg = ( GCLK_GENCTRL_DIV(TIMER_GCLK_DIVIDER_FACTOR) | GCLK_GENCTRL_SRC_DPLL0 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_DIVSEL );
      #elif defined(TIMER_1465Hz)
        GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_TIMERS].reg = ( GCLK_GENCTRL_DIV(TIMER_GCLK_DIVIDER_FACTOR) | GCLK_GENCTRL_SRC_DPLL0 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      #else
        GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_TIMERS].reg = ( GCLK_GENCTRL_DIV(TIMER_GCLK_DIVIDER_FACTOR * 4) | GCLK_GENCTRL_SRC_DPLL0 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      #endif
    #endif
    waitForSync();
  #elif (defined(CLOCKCONFIG_INTERNAL) || defined(CLOCKCONFIG_INTERNAL_USB))
    #if defined(TIMER_2930Hz)
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_TIMERS].reg = ( GCLK_GENCTRL_DIV(TIMER_GCLK_DIVIDER_FACTOR) | GCLK_GENCTRL_SRC_DPLL1 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_DIVSEL );
    #elif defined(TIMER_1465Hz)
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_TIMERS].reg = ( GCLK_GENCTRL_DIV(TIMER_GCLK_DIVIDER_FACTOR) | GCLK_GENCTRL_SRC_DPLL1 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
    #else
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_TIMERS].reg = ( GCLK_GENCTRL_DIV(TIMER_GCLK_DIVIDER_FACTOR * 4) | GCLK_GENCTRL_SRC_DPLL1 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
    #endif
    waitForSync();
  #endif
#endif
#endif


/* Setup additional GCLKs */
#if !defined(NO_ADDITIONAL_GCLKS)
#if (SAMD21 || SAMD11)
  #if (defined(CLOCKCONFIG_32768HZ_CRYSTAL) || defined(CLOCKCONFIG_HS_CRYSTAL))
    /* Switch GENERIC_CLOCK_GENERATOR_48MHz to 96MHz PLL output. The output is divided by two to obtain 48MHz. */
    GCLK->GENDIV.reg = ( GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_48MHz ) | GCLK_GENDIV_DIV(2) );
    waitForSync();
    GCLK->GENCTRL.reg = ( GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_48MHz ) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
    waitForSync();
  #elif (defined(CLOCKCONFIG_INTERNAL) || defined(CLOCKCONFIG_INTERNAL_USB))
    /* Switch GENERIC_CLOCK_GENERATOR_48MHz to 48MHz DFLL48M output. The output is undivided. */
    GCLK->GENDIV.reg = ( GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_48MHz ) | GCLK_GENDIV_DIV(1) );
    waitForSync();
    GCLK->GENCTRL.reg = ( GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_48MHz ) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
    waitForSync();
  #endif

#elif (SAML21 || SAMC21)
  #if (defined(CLOCKCONFIG_32768HZ_CRYSTAL) || defined(CLOCKCONFIG_HS_CRYSTAL))
    /* Switch GENERIC_CLOCK_GENERATOR_48MHz to 96MHz PLL output. The output is divided by two to obtain 48MHz. */
    GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_48MHz].reg = ( GCLK_GENCTRL_DIV(2) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
    waitForSync();
  #elif (defined(CLOCKCONFIG_INTERNAL) || defined(CLOCKCONFIG_INTERNAL_USB))
    /* Switch GENERIC_CLOCK_GENERATOR_48MHz to 48MHz DFLL48M output. The output is undivided. */
    GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_48MHz].reg = ( GCLK_GENCTRL_DIV(1) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
    waitForSync();
  #endif

#elif (SAMD51)
  #if (defined(CLOCKCONFIG_32768HZ_CRYSTAL) || defined(CLOCKCONFIG_HS_CRYSTAL))
    #if (VARIANT_MCK == 120000000ul)
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_192MHz].reg = ( GCLK_GENCTRL_DIV(1) | GCLK_GENCTRL_SRC_DPLL1 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      waitForSync();
    #else
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_192MHz].reg = ( GCLK_GENCTRL_DIV(1) | GCLK_GENCTRL_SRC_DPLL0 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      waitForSync();
    #endif
  #elif (defined(CLOCKCONFIG_INTERNAL) || defined(CLOCKCONFIG_INTERNAL_USB))
    #if (VARIANT_MCK == 120000000ul)
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_192MHz].reg = ( GCLK_GENCTRL_DIV(1) | GCLK_GENCTRL_SRC_DPLL1 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      waitForSync();
    #else
      GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_192MHz].reg = ( GCLK_GENCTRL_DIV(1) | GCLK_GENCTRL_SRC_DPLL1 | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
      waitForSync();
    #endif
  #endif
#endif
#endif  //  !defined(NO_ADDITIONAL_GCLKS)


#if !defined(NO_OSC_HS_GCLK)
#if (SAMD21 || SAMD11)
  /* Modify PRESCaler value of OSC8M to have 8MHz */
  SYSCTRL->OSC8M.bit.PRESC = SYSCTRL_OSC8M_PRESC_0_Val ;  // recent versions of CMSIS from Atmel changed the prescaler defines
  //SYSCTRL->OSC8M.bit.ONDEMAND = 0 ;

  /* Put OSC8M as source for Generic Clock Generator 3 */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_OSC_HS ) ; // Generic Clock Generator 3
  waitForSync();

  GCLK->GENCTRL.reg = ( GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_OSC_HS ) | GCLK_GENCTRL_SRC_OSC8M | GCLK_GENCTRL_GENEN );
  waitForSync();

#elif (SAML21)
  /* Note that after reset, the L21 starts with the OSC16M set to 4MHz, NOT the DFLL@48MHz as stated in some documentation. */
  /* Modify FSEL value of OSC16M to have 8MHz */
  OSCCTRL->OSC16MCTRL.bit.FSEL = OSCCTRL_OSC16MCTRL_FSEL_8_Val;

  /* Put OSC16M as source for Generic Clock Generator 3 */
  GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_OSC_HS].reg = ( GCLK_GENCTRL_DIV(1) | GCLK_GENCTRL_SRC_OSC16M | GCLK_GENCTRL_GENEN );
  waitForSync();
#endif
#endif

  SystemCoreClock=VARIANT_MCK;
}
