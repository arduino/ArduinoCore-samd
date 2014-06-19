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

static void Interrupt_Pin3( void ) ; // Ext Int Callback in LOW mode
static void Interrupt_Pin4( void ) ; // Ext Int Callback in HIGH mode
static void Interrupt_Pin5( void ) ; // Ext Int Callback in FALLING mode
static void Interrupt_Pin6( void ) ; // Ext Int Callback in RISING mode
static void Interrupt_Pin7( void ) ; // Ext Int Callback in CHANGE mode

static uint32_t ul_Interrupt_Pin3 = 0 ;
static uint32_t ul_Interrupt_Pin4 = 0 ;
static uint32_t ul_Interrupt_Pin5 = 0 ;
static uint32_t ul_Interrupt_Pin6 = 0 ;
static uint32_t ul_Interrupt_Pin7 = 0 ;

int temps = 0;
int valX = 0;
int valY = 0;


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

 /* pinPeripheral( 4, PIO_AC_CLK ) ; // Clock Gen 0
  pinPeripheral( 5, PIO_AC_CLK ) ; // Clock Gen 1
  pinPeripheral( 13, PIO_AC_CLK ) ; // Clock Gen 3*/

//**********************************************
  Serial5.begin( 115200 ) ; // Output to EDBG Virtual COM Port

  // Test External Interrupt
  attachInterrupt( 3, Interrupt_Pin3, LOW ) ;
  attachInterrupt( 4, Interrupt_Pin4, HIGH ) ;
  attachInterrupt( 5, Interrupt_Pin5, FALLING ) ;
  attachInterrupt( 6, Interrupt_Pin6, RISING ) ;
  attachInterrupt( 7, Interrupt_Pin7, CHANGE) ;
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
  static volatile uint16_t dac_value=0 ;

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

  dac_value += 64;
  analogWrite(A0, dac_value);



  Serial5.print("\r\nAnalog pins: ");

  for ( uint32_t i = A0 ; i <= A0+NUM_ANALOG_INPUTS ; i++ )
  {

    int a = analogRead(i);
    Serial5.print(a, DEC);
    Serial5.print(" ");

  }
  Serial5.println();

  Serial5.println("External interrupt pins:");
  if ( ul_Interrupt_Pin3 == 1 )
  {
    Serial5.println( "Pin 3 triggered (LOW)" ) ;
    ul_Interrupt_Pin3 = 0 ;
  }

  if ( ul_Interrupt_Pin4 == 1 )
  {
    Serial5.println( "Pin 4 triggered (HIGH)" ) ;
    ul_Interrupt_Pin4 = 0 ;
  }

  if ( ul_Interrupt_Pin5 == 1 )
  {
    Serial5.println( "Pin 5 triggered (FALLING)" ) ;
    ul_Interrupt_Pin5 = 0 ;
  }

  if ( ul_Interrupt_Pin6 == 1 )
  {
    Serial5.println( "Pin 6 triggered (RISING)" ) ;
    ul_Interrupt_Pin6 = 0 ;
  }

  if ( ul_Interrupt_Pin7 == 1 )
  {
    Serial5.println( "Pin 7 triggered (CHANGE)" ) ;
    ul_Interrupt_Pin7 = 0 ;
  }
}

// Ext Int Callback in LOW mode
static void Interrupt_Pin3( void )
{
  ul_Interrupt_Pin3 = 1 ;
}

// Ext Int Callback in HIGH mode
static void Interrupt_Pin4( void )
{
  ul_Interrupt_Pin4 = 1 ;
}

// Ext Int Callback in CHANGE mode
static void Interrupt_Pin5( void )
{
  ul_Interrupt_Pin5 = 1 ;
}

// Ext Int Callback in FALLING mode
static void Interrupt_Pin6( void )
{
  ul_Interrupt_Pin6 = 1 ;
}

// Ext Int Callback in RISING mode
static void Interrupt_Pin7( void )
{
  ul_Interrupt_Pin7 = 1 ;
}
