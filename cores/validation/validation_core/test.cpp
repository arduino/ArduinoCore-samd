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

  // Initialize the PIN 2 digital pin as an input.
  pinMode( 2, INPUT ) ;

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

void loop( void )
{
  volatile int pin_value=0 ;

  // Test digitalWrite
  led_step1() ;
  delay( 1000 ) ;              // wait for a second
  led_step2() ;
  delay( 1000 ) ;              // wait for a second

  // Test Serial output
  Serial.write( '-' ) ;   // send a char
  Serial.write( "test1\n" ) ;   // send a string
  Serial.write( "test2" ) ;   // send another string

  // Test digitalRead: connect pin 2 to either GND or 3.3V. !!!! NOT on 5V pin !!!!
  pin_value=digitalRead( 2 ) ;
  Serial.write( "pin 2 value is %s\n", (pin_value == LOW)?"LOW":"HIGH" ) ;
  delay( 1000 ) ;              // wait for a second


/*
  Serial.print("Analog pins: ");

  for ( int i = A1 ; i <= A0+NUM_ANALOG_INPUTS ; i++ )
  {
    int a = analogRead(i);
    Serial.print(a, DEC);
    Serial.print(" ");
  }
  Serial.println();
  delay(100);
*/

}
