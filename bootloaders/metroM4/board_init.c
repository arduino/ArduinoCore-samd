/*
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
 * \brief system_init() configures the needed clocks and according Flash Read Wait States.
 * We need to:
 * 1) Enable XOSC32K clock (External on-board 32.768Hz oscillator), will be used as DFLL48M reference.
 * 2) Put XOSC32K as source of Generic Clock Generator 3
 * 3) Put Generic Clock Generator 3 as source for Generic Clock Multiplexer 0 (DFLL48M reference)
 * 4) Enable DFLL48M clock
 * 5) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
 */
// Constants for Clock generators
#define GENERIC_CLOCK_GENERATOR_MAIN      (0u)
#define GENERIC_CLOCK_GENERATOR_XOSC32K   (3u)
#define GENERIC_CLOCK_GENERATOR_OSCULP32K (2u) /* Initialized at reset for WDT */
//#define GENERIC_CLOCK_GENERATOR_OSC8M     (3u)
// Constants for Clock multiplexers
#define GENERIC_CLOCK_MULTIPLEXER_DFLL48M (0u)

void board_init(void)
{
	
  /* Set 1 Flash Wait State for 48MHz */
  NVMCTRL->CTRLA.reg |= NVMCTRL_CTRLA_RWS(0);

  /* ----------------------------------------------------------------------------------------------
   * 1) Enable XOSC32K clock (External on-board 32.768Hz oscillator)
   */
	OSC32KCTRL->XOSC32K.reg = OSC32KCTRL_XOSC32K_ENABLE | OSC32KCTRL_XOSC32K_EN32K | OSC32KCTRL_XOSC32K_EN32K | OSC32KCTRL_XOSC32K_CGM_XT | OSC32KCTRL_XOSC32K_XTALEN;
	
	while( (OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY) == 0 ){
		/* Wait for oscillator to be ready */
	}
	
	OSC32KCTRL->RTCCTRL.bit.RTCSEL = OSC32KCTRL_RTCCTRL_RTCSEL_ULP1K;


  /* Software reset the module to ensure it is re-initialized correctly */
  /* Note: Due to synchronization, there is a delay from writing CTRL.SWRST until the reset is complete.
   * CTRL.SWRST and STATUS.SYNCBUSY will both be cleared when the reset is complete
   */
  GCLK->CTRLA.bit.SWRST = 1;
  while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_SWRST ){
	  /* wait for reset to complete */
  }
  
	/* ----------------------------------------------------------------------------------------------
	* 2) Put XOSC32K as source of Generic Clock Generator 3
	*/
	GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_XOSC32K].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_XOSC32K) | //generic clock gen 3
									GCLK_GENCTRL_GENEN;

	while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL3 ){
		/* Wait for synchronization */
	}
  
	/* ----------------------------------------------------------------------------------------------
	* 3) Put Generic Clock Generator 3 as source for Generic Clock Gen 0 (DFLL48M reference)
	*/
	GCLK->GENCTRL[0].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSCULP32K) | GCLK_GENCTRL_GENEN;
	
	/* ----------------------------------------------------------------------------------------------
	* 4) Enable DFLL48M clock
	*/

	/* DFLL Configuration in Open Loop mode */

	OSCCTRL->DFLLCTRLA.reg = 0;
	//GCLK->PCHCTRL[OSCCTRL_GCLK_ID_DFLL48].reg = (1 << GCLK_PCHCTRL_CHEN_Pos) | GCLK_PCHCTRL_GEN(GCLK_PCHCTRL_GEN_GCLK3_Val);

	OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_CSTEP( 0x1 ) |
						OSCCTRL_DFLLMUL_FSTEP( 0x1 ) |
						OSCCTRL_DFLLMUL_MUL( 0 );

	while ( OSCCTRL->DFLLSYNC.reg & OSCCTRL_DFLLSYNC_DFLLMUL )
	{
	/* Wait for synchronization */
	}
  
	OSCCTRL->DFLLCTRLB.reg = 0;
	while ( OSCCTRL->DFLLSYNC.reg & OSCCTRL_DFLLSYNC_DFLLCTRLB )
	{
		/* Wait for synchronization */
	}
  
	OSCCTRL->DFLLCTRLA.reg |= OSCCTRL_DFLLCTRLA_ENABLE;
	while ( OSCCTRL->DFLLSYNC.reg & OSCCTRL_DFLLSYNC_ENABLE )
	{
		/* Wait for synchronization */
	}
	
	OSCCTRL->DFLLVAL.reg = OSCCTRL->DFLLVAL.reg;
	while( OSCCTRL->DFLLSYNC.bit.DFLLVAL );
  
  OSCCTRL->DFLLCTRLB.reg = OSCCTRL_DFLLCTRLB_WAITLOCK |
  OSCCTRL_DFLLCTRLB_CCDIS | OSCCTRL_DFLLCTRLB_USBCRM ;
  
	while ( !OSCCTRL->STATUS.bit.DFLLRDY )
	{
		/* Wait for synchronization */
	}

  /* ----------------------------------------------------------------------------------------------
   * 5) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
   */
	GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_MAIN].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL) |
					GCLK_GENCTRL_IDC |
					GCLK_GENCTRL_OE |
					GCLK_GENCTRL_GENEN;

  while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL0 )
  {
    /* Wait for synchronization */
  }
  
  
  /* Turn on the digital interface clock */
  //MCLK->APBAMASK.reg |= MCLK_APBAMASK_GCLK;

  /*
   * Now that all system clocks are configured, we can set CLKDIV .
   * These values are normally the ones present after Reset.
   */
	MCLK->CPUDIV.reg = MCLK_CPUDIV_DIV_DIV1;
}
