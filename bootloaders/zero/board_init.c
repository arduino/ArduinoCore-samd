/*
  Copyright (c) 2017 MattairTech LLC. All right reserved.
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.

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

#include <sam.h>
#include "board_definitions.h"

/**
 * \brief SystemInit() configures the needed clocks and according Flash Read Wait States.
 */

// Constants for Clock generators
#define GENERIC_CLOCK_GENERATOR_MAIN      (0u)
#define GENERIC_CLOCK_GENERATOR_XOSC      (1u)
#define GENERIC_CLOCK_GENERATOR_OSCULP32K (2u) /* Initialized at reset for WDT (D21/D11) */
#define GENERIC_CLOCK_GENERATOR_OSC_HS    (3u)

// Constants for Clock multiplexers
#if (SAMD || SAML21)
  #define GENERIC_CLOCK_MULTIPLEXER_DFLL    (0u)
  #define GENERIC_CLOCK_MULTIPLEXER_FDPLL   (1u)
  #define GENERIC_CLOCK_MULTIPLEXER_FDPLL96M_32K (2u)
#elif (SAMC21)
  #define GENERIC_CLOCK_MULTIPLEXER_FDPLL   (0u)
  #define GENERIC_CLOCK_MULTIPLEXER_FDPLL96M_32K (1u)
#else
  #error "startup.c: Unsupported chip"
#endif

void waitForSync( void )
{
#if (SAMD)
  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );
#elif (SAML21 || SAMC21)
  while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK );
#endif
}

#if (SAMD || SAML21)
void waitForDFLL( void )
{
#if (SAMD)
  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 );
#elif (SAML21)
  while ( (OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY) == 0 );
#endif
}
#endif

#if (SAML21 || SAMC21)
void waitForPLL( void )
{
  while ( OSCCTRL->DPLLSYNCBUSY.reg & OSCCTRL_DPLLSYNCBUSY_MASK );
}
#endif

void board_init( void )
{
  /* Set 1 Flash Wait State for 48MHz (2 for the L21 and C21), cf tables 20.9 and 35.27 in SAMD21 Datasheet */
#if (SAMD)
  NVMCTRL->CTRLB.bit.RWS = NVMCTRL_CTRLB_RWS_HALF_Val ;	// one wait state
#elif (SAML21 || SAMC21)
  NVMCTRL->CTRLB.reg |= NVMCTRL_CTRLB_RWS_DUAL ; // two wait states
#endif

  /* Turn on the digital interface clock */
#if (SAMD)
  PM->APBAMASK.reg |= PM_APBAMASK_GCLK ;
#elif (SAML21 || SAMC21)
  MCLK->APBAMASK.reg |= MCLK_APBAMASK_GCLK ;
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
#if (SAMD)
  GCLK->CTRL.reg = GCLK_CTRL_SWRST ;

  while ( (GCLK->CTRL.reg & GCLK_CTRL_SWRST) && (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) );	/* Wait for reset to complete */
#elif (SAML21 || SAMC21)
  GCLK->CTRLA.reg = GCLK_CTRLA_SWRST ;

  while ( (GCLK->CTRLA.reg & GCLK_CTRLA_SWRST) && (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK) );	/* Wait for reset to complete */
#endif

#if defined(CLOCKCONFIG_CLOCK_SOURCE) && (CLOCKCONFIG_CLOCK_SOURCE == CLOCKCONFIG_32768HZ_CRYSTAL)
  /* ----------------------------------------------------------------------------------------------
   * Enable XOSC32K clock (External on-board 32.768Hz crystal oscillator)
   */

#if defined(PLL_FRACTIONAL_ENABLE)
  #define DPLLRATIO_LDR		2928u
  #define DPLLRATIO_LDRFRAC	11u
#else
  #define DPLLRATIO_LDR		2929u
  #define DPLLRATIO_LDRFRAC	0u
#endif

#if (SAMD)
  SYSCTRL->XOSC32K.reg = (SYSCTRL_XOSC32K_STARTUP( 0x6u ) | SYSCTRL_XOSC32K_XTALEN | SYSCTRL_XOSC32K_EN32K) ;
  SYSCTRL->XOSC32K.bit.ENABLE = 1 ; /* separate call, as described in chapter 15.6.3 */

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_XOSC32KRDY) == 0 );	/* Wait for oscillator stabilization */
  
  SYSCTRL->DPLLRATIO.reg = ( SYSCTRL_DPLLRATIO_LDR(DPLLRATIO_LDR) | SYSCTRL_DPLLRATIO_LDRFRAC(DPLLRATIO_LDRFRAC) ) ;  /* set PLL multiplier */
  
  SYSCTRL->DPLLCTRLB.reg = SYSCTRL_DPLLCTRLB_REFCLK(0) ;  /* select 32KHz crystal input */
  
  SYSCTRL->DPLLCTRLA.reg = SYSCTRL_DPLLCTRLA_ENABLE ;
  
  while ( (SYSCTRL->DPLLSTATUS.reg & SYSCTRL_DPLLSTATUS_CLKRDY) != SYSCTRL_DPLLSTATUS_CLKRDY );
  
  /* Switch Generic Clock Generator 0 to PLL. Divide by two and the CPU will run at 48MHz */
  GCLK->GENDIV.reg = ( GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_MAIN ) | GCLK_GENDIV_DIV(2) ) ;
  waitForSync();

  GCLK->GENCTRL.reg = ( GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_MAIN ) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
  waitForSync();
  
#elif (SAML21 || SAMC21)
  OSC32KCTRL->XOSC32K.reg = (OSC32KCTRL_XOSC32K_STARTUP( 0x4u ) | OSC32KCTRL_XOSC32K_XTALEN | OSC32KCTRL_XOSC32K_EN32K | OSC32KCTRL_XOSC32K_EN1K) ;
  OSC32KCTRL->XOSC32K.bit.ENABLE = 1 ;
  
  while ( (OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY) == 0 );	/* Wait for oscillator stabilization */
  
  OSCCTRL->DPLLRATIO.reg = ( OSCCTRL_DPLLRATIO_LDR(DPLLRATIO_LDR) | OSCCTRL_DPLLRATIO_LDRFRAC(DPLLRATIO_LDRFRAC) ) ;  /* set PLL multiplier */
  waitForPLL();
  
  OSCCTRL->DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_REFCLK(0) ;  /* select 32KHz crystal input */
  
  OSCCTRL->DPLLPRESC.reg = 0;
  waitForPLL();
  
  OSCCTRL->DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE ;
  waitForPLL();
  
  while ( (OSCCTRL->DPLLSTATUS.reg & OSCCTRL_DPLLSTATUS_CLKRDY) != OSCCTRL_DPLLSTATUS_CLKRDY );
  
  /* Switch Generic Clock Generator 0 to PLL. Divide by two and the CPU will run at 48MHz */
  GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_MAIN].reg = ( GCLK_GENCTRL_DIV(2) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
  waitForSync();
#endif

#elif defined(CLOCKCONFIG_CLOCK_SOURCE) && (CLOCKCONFIG_CLOCK_SOURCE == CLOCKCONFIG_HS_CRYSTAL)
  /* ----------------------------------------------------------------------------------------------
   * Enable XOSC clock (External on-board high speed crystal oscillator)
   */

#if ((HS_CRYSTAL_FREQUENCY_HERTZ < 400000UL) || (HS_CRYSTAL_FREQUENCY_HERTZ > 32000000UL))
  #error "board.init.c: HS_CRYSTAL_FREQUENCY_HERTZ must be between 4000000UL and 32000000UL"
#endif

#if defined(PLL_FAST_STARTUP)
  #if (HS_CRYSTAL_FREQUENCY_HERTZ < 1000000UL)
    #error "board.init.c: HS_CRYSTAL_FREQUENCY_HERTZ must be at least 1000000UL when PLL_FAST_STARTUP is defined"
  #else
    #define HS_CRYSTAL_DIVISOR	1000000UL
  #endif
#else
  #define HS_CRYSTAL_DIVISOR	32000UL
#endif

#define HS_CRYSTAL_DIVIDER	(HS_CRYSTAL_FREQUENCY_HERTZ / HS_CRYSTAL_DIVISOR)
#define DPLLRATIO_FLOAT		(96000000.0 / ((float)HS_CRYSTAL_FREQUENCY_HERTZ / HS_CRYSTAL_DIVIDER))

#if defined(PLL_FRACTIONAL_ENABLED)
  #define DPLLRATIO_LDR		(uint16_t)DPLLRATIO_FLOAT
  #define DPLLRATIO_LDRFRAC	(uint8_t)((DPLLRATIO_FLOAT - (uint16_t)DPLLRATIO_FLOAT) * 16.0)
#else
  #define DPLLRATIO_LDR		(uint16_t)DPLLRATIO_FLOAT
  #define DPLLRATIO_LDRFRAC	0
#endif

#if (SAMD)
  SYSCTRL->XOSC.reg = (SYSCTRL_XOSC_STARTUP( 0x8u ) | SYSCTRL_XOSC_GAIN( 0x4u ) | SYSCTRL_XOSC_XTALEN | SYSCTRL_XOSC_ENABLE) ;	// startup time is 8ms
  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_XOSCRDY) == 0 );	/* Wait for oscillator stabilization */

  SYSCTRL->XOSC.reg |= SYSCTRL_XOSC_AMPGC ;	// set only after startup time
  
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
  
  /* Switch Generic Clock Generator 0 to PLL. Divide by two and the CPU will run at 48MHz */
  GCLK->GENDIV.reg = ( GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_MAIN ) | GCLK_GENDIV_DIV(2) ) ;
  waitForSync();

  GCLK->GENCTRL.reg = ( GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_MAIN ) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
  waitForSync();
  
#elif (SAML21 || SAMC21)
  OSCCTRL->XOSCCTRL.reg = (OSCCTRL_XOSCCTRL_STARTUP( 0x8u ) | OSCCTRL_XOSCCTRL_GAIN( 0x4u ) | OSCCTRL_XOSCCTRL_XTALEN | OSCCTRL_XOSCCTRL_ENABLE) ;	// startup time is 8ms
  while ( (OSCCTRL->STATUS.reg & OSCCTRL_STATUS_XOSCRDY) == 0 );	/* Wait for oscillator stabilization */

  OSCCTRL->XOSCCTRL.reg |= OSCCTRL_XOSCCTRL_AMPGC ;	// set only after startup time
  
  /* Connect GCLK1 to XOSC and set prescaler */
  GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_XOSC].reg = ( GCLK_GENCTRL_DIV(HS_CRYSTAL_DIVIDER) | GCLK_GENCTRL_SRC_XOSC | GCLK_GENCTRL_GENEN );
  waitForSync();
  
  /* Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 1 (FDPLL reference) */
  GCLK->PCHCTRL[GENERIC_CLOCK_MULTIPLEXER_FDPLL].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK1 );
  while ( (GCLK->PCHCTRL[GENERIC_CLOCK_MULTIPLEXER_FDPLL].reg & GCLK_PCHCTRL_CHEN) != GCLK_PCHCTRL_CHEN );	// wait for sync
  
  /* Configure PLL */
  OSCCTRL->DPLLRATIO.reg = ( OSCCTRL_DPLLRATIO_LDR(DPLLRATIO_LDR) | OSCCTRL_DPLLRATIO_LDRFRAC(DPLLRATIO_LDRFRAC) ) ;  /* set PLL multiplier */
  waitForPLL();
  
  OSCCTRL->DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_REFCLK(2) ;  /* select GCLK input */
  
  OSCCTRL->DPLLPRESC.reg = 0;
  waitForPLL();
  
  OSCCTRL->DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE ;
  waitForPLL();
  
  while ( (OSCCTRL->DPLLSTATUS.reg & OSCCTRL_DPLLSTATUS_CLKRDY) != OSCCTRL_DPLLSTATUS_CLKRDY );
  
  /* Switch Generic Clock Generator 0 to PLL. Divide by two and the CPU will run at 48MHz */
  GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_MAIN].reg = ( GCLK_GENCTRL_DIV(2) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
  waitForSync();
#endif

#elif defined(CLOCKCONFIG_CLOCK_SOURCE) && ((CLOCKCONFIG_CLOCK_SOURCE == CLOCKCONFIG_INTERNAL) || (CLOCKCONFIG_CLOCK_SOURCE == CLOCKCONFIG_INTERNAL_USB))
  /* ----------------------------------------------------------------------------------------------
   * Enable DFLL48M clock (D21/L21) or RC oscillator (C21)
   */
#if (SAMD)
  /* Remove the OnDemand mode, Bug http://avr32.icgroup.norway.atmel.com/bugzilla/show_bug.cgi?id=9905 */
  SYSCTRL->DFLLCTRL.bit.ONDEMAND = 0 ;
  waitForDFLL();
  
  /* Load NVM Coarse calibration value */
  uint32_t calib = (*((uint32_t *) FUSES_DFLL48M_COARSE_CAL_ADDR) & FUSES_DFLL48M_COARSE_CAL_Msk) >> FUSES_DFLL48M_COARSE_CAL_Pos;
  SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE(calib) | SYSCTRL_DFLLVAL_FINE(512);
  
  /* Write full configuration to DFLL control register */
  #if defined(CLOCKCONFIG_CLOCK_SOURCE) && (CLOCKCONFIG_CLOCK_SOURCE == CLOCKCONFIG_INTERNAL_USB)
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
  
  /* Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_MAIN ) ; // Generic Clock Generator 0
  waitForSync();

  GCLK->GENCTRL.reg = ( GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_MAIN ) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
  waitForSync();
  
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
  #if defined(CLOCKCONFIG_CLOCK_SOURCE) && (CLOCKCONFIG_CLOCK_SOURCE == CLOCKCONFIG_INTERNAL_USB)
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
  
  /* Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz */
  GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_MAIN].reg = ( GCLK_GENCTRL_DIV(1) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
  waitForSync();
  
#elif (SAMC21)
  #if defined(CLOCKCONFIG_CLOCK_SOURCE) && (CLOCKCONFIG_CLOCK_SOURCE == CLOCKCONFIG_INTERNAL_USB)
    #error "startup.c: CLOCKCONFIG_INTERNAL_USB setting invalid for C21 chips as they lack USB."
  #endif
  
  /* Change OSC48M divider to /1. CPU will run at 48MHz */
  OSCCTRL->OSC48MDIV.reg = OSCCTRL_OSC48MDIV_DIV(0);
  while ( OSCCTRL->OSC48MSYNCBUSY.reg & OSCCTRL_OSC48MSYNCBUSY_OSC48MDIV );
#endif
#else
  #error "board_init.c: CLOCKCONFIG_CLOCK_SOURCE must be defined in the board_definitions file"
#endif

#if (SAMD21)
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

  /* Set CPU and APB dividers before switching the CPU/APB clocks to the new clock source */
#if (SAMD)
  PM->CPUSEL.reg  = PM_CPUSEL_CPUDIV_DIV1 ;
  PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV1_Val ;
  PM->APBBSEL.reg = PM_APBBSEL_APBBDIV_DIV1_Val ;
  PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1_Val ;
#elif (SAML21 || SAMC21)
  MCLK->CPUDIV.reg  = MCLK_CPUDIV_CPUDIV_DIV1 ;
#endif

  /*
   * Disable automatic NVM write operations (errata reference 13134, applies to D21/D11/L21, but not C21)
   */
  NVMCTRL->CTRLB.bit.MANW = 1;
}
