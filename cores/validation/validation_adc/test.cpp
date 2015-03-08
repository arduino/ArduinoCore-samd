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

volatile uint32_t t_start=0 ;
volatile uint32_t t_end=0 ;
volatile uint32_t t_delta=0 ;

#define NUM_SAMPLES (256ul)

void setup( void )
{
  int adc_value[NUM_SAMPLES] ;

  SERIAL_PORT_MONITOR.begin( 115200 ) ; // Output to EDBG Virtual COM Port

  SERIAL_PORT_MONITOR.print( "Analog pins: \r\n" ) ;

  t_start=millis() ;
  for ( uint32_t i = 0 ; i < NUM_SAMPLES ; i++ )
  {
    adc_value[i] = analogRead( A1 ) ;
  }
  t_end=millis() ;
  t_delta=t_end-t_start ;

  SERIAL_PORT_MONITOR.print( t_delta, DEC ) ;
  SERIAL_PORT_MONITOR.println() ;
}

void loop( void )
{
}
