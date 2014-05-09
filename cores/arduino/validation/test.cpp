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

void setup( void )
{
  // Initialize the digital pin as an output.
  // Pin PIN_LED has a LED connected on most Arduino boards:
  //pinMode( PIN_LED, OUTPUT ) ;
  //digitalWrite( PIN_LED, LOW ) ;

  // Initialize the PIN_LED2 digital pin as an output.
  pinMode( PIN_LED2, OUTPUT ) ;
  digitalWrite( PIN_LED2, HIGH ) ;

  // Initialize the PIN_LED2 digital pin as an output.
  pinMode( PIN_LED3, OUTPUT ) ;
  digitalWrite( PIN_LED3, LOW ) ;

//**********************************************
// Clock output on pin 4 for measure

  pinPeripheral( 4, PIO_AC_CLK ) ; // Clock Gen 0
  pinPeripheral( 5, PIO_AC_CLK ) ; // Clock Gen 1
  pinPeripheral( 13, PIO_AC_CLK ) ; // Clock Gen 3

//**********************************************

/*
  Serial1.begin( 115200 ) ;
*/
}

static void led_step1( void )
{
//  digitalWrite( PIN_LED, LOW ) ;  // set the LED on
  digitalWrite( PIN_LED2, LOW ) ;   // set the red LED off
  digitalWrite( PIN_LED3, HIGH ) ;   // set the red LED off
}

static void led_step2( void )
{
//  digitalWrite( PIN_LED, HIGH ) ;   // set the LED off
  digitalWrite( PIN_LED2, HIGH ) ;  // set the red LED on
  digitalWrite( PIN_LED3, LOW ) ;  // set the red LED on
}

static void analog_write_step (void)
{
	// test PWM generation on all PWM pins (duty cycle from 0x00 to 0xFF)
	for( uint8_t duty_cycle = 0x00;duty_cycle<=0xFF;duty_cycle++)
	{
		analogWrite(13,duty_cycle);
		analogWrite(12,duty_cycle);
		analogWrite(11,duty_cycle);
		analogWrite(10,duty_cycle);
		analogWrite(9,duty_cycle);
		analogWrite(8,duty_cycle);
		analogWrite(7,duty_cycle);
		analogWrite(6,duty_cycle);
		analogWrite(5,duty_cycle);
		analogWrite(4,duty_cycle);
		analogWrite(3,duty_cycle);
		analogWrite(2,duty_cycle);
		delay( 10 ) ;
	}


}

void loop( void )
{
  led_step1() ;
  delay( 1000 ) ;              // wait for a second
  led_step2() ;
  delay( 1000 ) ;              // wait for a second
  
  analog_write_step();
	
/*
  Serial1.write( '-' ) ;   // send a char
  Serial1.write( "test1\n" ) ;   // send a string
  Serial1.write( "test2" ) ;   // send another string
*/

/*
  Serial1.print("Analog pins: ");

  for ( int i = A1 ; i <= A0+NUM_ANALOG_INPUTS ; i++ )
  {
    int a = analogRead(i);
    Serial1.print(a, DEC);
    Serial1.print(" ");
  }
  Serial1.println();
  delay(100);
*/
}
