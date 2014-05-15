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
  // Pin PIN_LED has an LED connected on most Arduino boards:
  //pinMode( PIN_LED, OUTPUT ) ;
  //digitalWrite( PIN_LED, LOW ) ;

  // Initialize the PIN_LED2 digital pin as an output.
  pinMode( PIN_LED2, OUTPUT ) ;
  digitalWrite( PIN_LED2, HIGH ) ;

  // Initialize the PIN_LED3 digital pin as an output.
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
  Serial5.begin( 115200 ) ; // Output to EDBG Virtual COM Port
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
  static volatile uint8_t duty_cycle=0 ;

  // Test digitalWrite
  led_step1() ;
  delay( 500 ) ;              // wait for a second
  led_step2() ;
  delay( 500 ) ;              // wait for a second

  // Test Serial output
  Serial5.write( '-' ) ;   // send a char
  Serial5.write( "test1\n" ) ;   // send a string
  Serial5.write( "test2" ) ;   // send another string

  // Test digitalRead: connect pin 2 to either GND or 3.3V. !!!! NOT on 5V pin !!!!
  pin_value=digitalRead( 2 ) ;
  Serial5.write( "pin 2 value is " ) ;
  Serial5.write( (pin_value == LOW)?"LOW\n":"HIGH\n" ) ;

  duty_cycle+=8 ;//=(uint8_t)(millis() & 0xff) ;
  analogWrite( 13, duty_cycle ) ;
  analogWrite( 12, duty_cycle ) ;
  analogWrite( 11, duty_cycle ) ;
  analogWrite( 10 ,duty_cycle ) ;
  analogWrite(  9, duty_cycle ) ;
  analogWrite(  8, duty_cycle ) ;
  analogWrite(  7, duty_cycle ) ;
  analogWrite(  6, duty_cycle ) ;
  analogWrite(  5, duty_cycle ) ;
  analogWrite(  4, duty_cycle ) ;
  analogWrite(  3, duty_cycle ) ;

  Serial5.print("Analog pins: ");

  for ( int i = A1 ; i <= A0+NUM_ANALOG_INPUTS ; i++ )
  {
/*
    int a = analogRead(i);
    Serial5.print(a, DEC);
    Serial5.print(" ");
*/
  }
  Serial5.println();
}
