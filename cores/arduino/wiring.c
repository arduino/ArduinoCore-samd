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

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * System Core Clock is at 1MHz at Reset.
 * It is switched to 48MHz in the Reset Handler (startup.c)
 */
extern uint32_t SystemCoreClock=1000000ul ;

void __libc_init_array(void);

/*
 * Arduino Zero board initialization
 *
 * Good to know:
 *   - Watchdog is disabled by default, unless someone plays with NVM User page
 *   -
 */
void init( void )
{
  // Set Systick to 1ms interval, common to all SAM3 variants
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    // Capture error
    while (true);
  }


  // Initialize C library
//  __libc_init_array();

  // Disable pull-up on every pin
  for ( int i = 0 ; i < PINS_COUNT ; i++ )
	{
	  digitalWrite( i, LOW ) ;
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

