/*
  Copyright (c) 2014 Arduino.  All right reserved.

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

static uint8_t brightness = 0;    // how bright the LED is
static int fadeAmount = 5;    // how many points to fade the LED by

void setup( void )
{
  for ( int i=2 ; i < 14 ; i++ )
  {
    pinMode( i, OUTPUT ) ;
  }

  SERIAL_PORT_MONITOR.begin( 115200 ) ; // Output to EDBG Virtual COM Port
  SERIAL_PORT_MONITOR.println( "test") ;
  SERIAL_PORT_MONITOR.println( brightness ) ;
}

void loop( void )
{
  SERIAL_PORT_MONITOR.println( brightness ) ;

  for ( int i=2 ; i < 14 ; i++ )
  {
    analogWrite( i, brightness ) ;
  }

  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount ;

  // reverse the direction of the fading at the ends of the fade:
  if ( brightness == 0 || brightness == 255 )
  {
    fadeAmount = -fadeAmount ;
  }
  
  // wait for 30 milliseconds to see the dimming effect
  delay( 30 ) ;
}

