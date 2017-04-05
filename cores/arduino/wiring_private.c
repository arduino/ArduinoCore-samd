/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2017 MattairTech LLC. All right reserved.

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
#include "wiring_private.h"

int pinPeripheral( uint32_t ulPin, uint32_t ulPeripheral )
{
  // Prevent out of bounds access
  if (ulPin >= NUM_PIN_DESCRIPTION_ENTRIES)
  {
    return -1 ;
  }

  uint32_t pinAttribute = g_APinDescription[ulPin].ulPinAttribute;
  uint8_t peripheralAttribute = g_APinDescription[ulPin].ulPeripheralAttribute;
  uint8_t pinType = g_APinDescription[ulPin].ulPinType;

  // Handle the case the pin isn't usable as PIO
  if ( pinType == PIO_NOT_A_PIN )
  {
    return -1 ;
  }

  // If pinType is not PIO_MULTI or PIO_STARTUP in the pinDescription table, then it must match ulPeripheral
  if ( pinType != PIO_MULTI && pinType != PIO_STARTUP && pinType != ulPeripheral )
  {
    return -1 ;
  }

  // Make sure ulPeripheral is listed in the attributes
  if ( !(pinAttribute & (1UL << ulPeripheral)) && pinType != PIO_STARTUP )
  {
    return -1 ;
  }

  // Determine hardware peripheral to use
  EPioPeripheral peripheral = PER_PORT;
  switch ( ulPeripheral )
  {
    case PIO_EXTINT:
      if ( digitalPinToInterrupt( ulPin ) == NOT_AN_INTERRUPT )
      {
        return -1 ;
      }
      peripheral = PER_EXTINT;
    break ;

    case PIO_ANALOG_ADC:
      if ( g_APinDescription[ulPin].ulADCChannelNumber == No_ADC_Channel )
      {
        return -1 ;
      }
      peripheral = PER_ANALOG;
    break ;

    case PIO_ANALOG_DAC:
    case PIO_ANALOG_REF:
      peripheral = PER_ANALOG;
    break ;

    case PIO_TIMER_PWM:
    case PIO_TIMER_CAPTURE:
      if ( g_APinDescription[ulPin].ulTCChannel == NOT_ON_TIMER )
      {
        return -1 ;
      }

      if ( (peripheralAttribute & PER_ATTR_TIMER_MASK) == PER_ATTR_TIMER_STD )
      {
        peripheral = PER_TIMER;
      }
      else
      {
        peripheral = PER_TIMER_ALT;
      }
    break ;

    case PIO_SERCOM:
      if ( (peripheralAttribute & PER_ATTR_SERCOM_MASK) == PER_ATTR_SERCOM_STD )
      {
        peripheral = PER_SERCOM;
      }
      else
      {
        peripheral = PER_SERCOM_ALT;
      }
    break ;

    case PIO_COM:
      peripheral = PER_COM;
    break ;

    case PIO_AC_GCLK:
      peripheral = PER_AC_CLK;
    break ;

    case PIO_CCL:
      peripheral = PER_CCL;
    break ;

    case PIO_NOT_A_PIN:
    case PIO_MULTI:
      return -1l ;
    break ;

    default:
    break ;
  }

  noInterrupts(); // Avoid possible invalid interim pin state

  uint8_t pinPort = g_APinDescription[ulPin].ulPort;
  uint8_t pinNum = g_APinDescription[ulPin].ulPin;
  uint8_t pinCfg = PORT_PINCFG_INEN;	// INEN should be enabled for both input and output (but not analog)

  switch ( ulPeripheral )
  {
    case PIO_STARTUP:
      PORT->Group[pinPort].PINCFG[pinNum].reg=(uint8_t)pinCfg ; // Enable INEN. If pin is floating, you should enable pull resistor to minimize input buffer power consumption
      PORT->Group[pinPort].OUTSET.reg = (uint32_t)(1<<pinNum) ;	// set default pull direction to pullup (will not be enabled)
    break;
    
    // Set pin mode according to chapter '22.6.3 I/O Pin Configuration', the out register stores the pull direction
    case PIO_INPUT:
    case PIO_INPUT_PULLUP:
    case PIO_INPUT_PULLDOWN:
      PORT->Group[pinPort].DIRCLR.reg = (uint32_t)(1<<pinNum) ;
      PORT->Group[pinPort].PINCFG[pinNum].reg=(uint8_t)pinCfg ;
      
      if (ulPeripheral == PIO_INPUT_PULLUP || ulPeripheral == PIO_INPUT)	// default pull direction for PIO_INPUT is pullup
      {
        // Enable pull level (cf '22.6.3.2 Input Configuration' and '22.8.7 Data Output Value Set')
        PORT->Group[pinPort].OUTSET.reg = (uint32_t)(1<<pinNum) ;
      }
      else
      {
        // Enable pull level (cf '22.6.3.2 Input Configuration' and '22.8.6 Data Output Value Clear')
        PORT->Group[pinPort].OUTCLR.reg = (uint32_t)(1<<pinNum) ;
      }
      if (ulPeripheral != PIO_INPUT)
      {
	      PORT->Group[pinPort].PINCFG[pinNum].reg=(uint8_t)(pinCfg | PORT_PINCFG_PULLEN) ;
      }
    break ;

    case PIO_OUTPUT:
      if ( (peripheralAttribute & PER_ATTR_DRIVE_MASK) == PER_ATTR_DRIVE_STRONG )
      {
        pinCfg |= PORT_PINCFG_DRVSTR;
      }
      // Set pin to output mode, set pin drive strength, disable the port mux, INEN will be set
      PORT->Group[pinPort].PINCFG[pinNum].reg = (uint8_t)pinCfg ;
      PORT->Group[pinPort].DIRSET.reg = (uint32_t)(1<<pinNum) ;
    break ;


    case PIO_ANALOG_ADC:
    case PIO_ANALOG_DAC:
    case PIO_ANALOG_REF:
	pinCfg = 0;	// Disable INEN with analog
	
    case PIO_EXTINT:
    case PIO_TIMER_PWM:
    case PIO_TIMER_CAPTURE:
    case PIO_SERCOM:
    case PIO_COM:
    case PIO_AC_GCLK:
    case PIO_CCL:

      if ( pinNum & 1 ) // is pin odd?
      {
        uint32_t temp ;

        // Get whole current setup for both odd and even pins and remove odd one
	temp = (PORT->Group[pinPort].PMUX[pinNum >> 1].reg) & PORT_PMUX_PMUXE( 0xF ) ;
        // Set new muxing
	PORT->Group[pinPort].PMUX[pinNum >> 1].reg = temp|PORT_PMUX_PMUXO( peripheral ) ;
        // Enable port mux
	PORT->Group[pinPort].PINCFG[pinNum].reg |= PORT_PINCFG_PMUXEN ;
      }
      else // even pin
      {
        uint32_t temp ;

	temp = (PORT->Group[pinPort].PMUX[pinNum >> 1].reg) & PORT_PMUX_PMUXO( 0xF ) ;
	PORT->Group[pinPort].PMUX[pinNum >> 1].reg = temp|PORT_PMUX_PMUXE( peripheral ) ;
	PORT->Group[pinPort].PINCFG[pinNum].reg |= PORT_PINCFG_PMUXEN ; // Enable port mux
      }

    break ;

    default:
    break ;
  }

  interrupts();
  return 0l ;
}
