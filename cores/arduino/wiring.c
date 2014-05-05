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

//void __libc_init_array(void);

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

  // Initialize C library
//  __libc_init_array();

  // Setup PORT for Digital I/O
	PM->APBBMASK.reg |= PM_APBBMASK_PORT ;

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
	// Todo
}

#ifdef __cplusplus
}
#endif

