/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

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
  uint32_t pinAttribute = g_APinDescription[ulPin].ulPinAttribute;
  uint8_t peripheralAttribute = g_APinDescription[ulPin].ulPeripheralAttribute;
  EPioType pinType = g_APinDescription[ulPin].ulPinType;

  // Handle the case the pin isn't usable as PIO
  if ( pinType == PIO_NOT_A_PIN )
  {
    return -1 ;
  }

  // If pinType is not PIO_MULTI in the pinDescription table, then it must match ulPeripheral
  if ( pinType != PIO_MULTI && pinType != ulPeripheral )
  {
    return -1 ;
  }

  // Make sure ulPeripheral is listed in the attributes
  if ( !(pinAttribute & (1UL << ulPeripheral)) )
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

    default:
    break ;
  }

  switch ( ulPeripheral )
  {
    // Set pin mode according to chapter '22.6.3 I/O Pin Configuration'
    case PIO_INPUT:
      // Set pin to input mode, disable the port mux
      PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg=(uint8_t)(PORT_PINCFG_INEN) ;
      PORT->Group[g_APinDescription[ulPin].ulPort].DIRCLR.reg = (uint32_t)(1<<g_APinDescription[ulPin].ulPin) ;
    break ;

    case PIO_OUTPUT:
      // Set pin to output mode, set pin drive strength, disable the port mux
      PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg &= ~(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PMUXEN) ;
      if ( (peripheralAttribute & PER_ATTR_DRIVE_MASK) == PER_ATTR_DRIVE_STRONG )
      {
        PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg |= (uint8_t)(PORT_PINCFG_DRVSTR) ;
      }
      PORT->Group[g_APinDescription[ulPin].ulPort].DIRSET.reg = (uint32_t)(1<<g_APinDescription[ulPin].ulPin) ;
    break ;

    case PIO_INPUT_PULLUP:
      // Set pin to input mode with pull-up resistor enabled, disable the port mux
      PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN) ;
      PORT->Group[g_APinDescription[ulPin].ulPort].DIRCLR.reg = (uint32_t)(1<<g_APinDescription[ulPin].ulPin) ;

      // Enable pull level (cf '22.6.3.2 Input Configuration' and '22.8.7 Data Output Value Set')
      PORT->Group[g_APinDescription[ulPin].ulPort].OUTSET.reg = (uint32_t)(1<<g_APinDescription[ulPin].ulPin) ;
    break ;

    case PIO_INPUT_PULLDOWN:
      // Set pin to input mode with pull-down resistor enabled, disable the port mux
      PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN) ;
      PORT->Group[g_APinDescription[ulPin].ulPort].DIRCLR.reg = (uint32_t)(1<<g_APinDescription[ulPin].ulPin) ;

      // Enable pull level (cf '22.6.3.2 Input Configuration' and '22.8.6 Data Output Value Clear')
      PORT->Group[g_APinDescription[ulPin].ulPort].OUTCLR.reg = (uint32_t)(1<<g_APinDescription[ulPin].ulPin) ;
    break ;


    case PIO_EXTINT:
    case PIO_ANALOG_ADC:
    case PIO_ANALOG_DAC:
    case PIO_ANALOG_REF:
    case PIO_TIMER_PWM:
    case PIO_TIMER_CAPTURE:
    case PIO_SERCOM:
    case PIO_COM:
    case PIO_AC_GCLK:
#if 0
      // Is the pio pin in the lower 16 ones?
      // The WRCONFIG register allows update of only 16 pin max out of 32
      if ( g_APinDescription[ulPin].ulPin < 16 )
      {
        PORT->Group[g_APinDescription[ulPin].ulPort].WRCONFIG.reg = PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PMUX( ulPeripheral ) |
                                                                    PORT_WRCONFIG_WRPINCFG |
                                                                    PORT_WRCONFIG_PINMASK( g_APinDescription[ulPin].ulPin ) ;
      }
      else
      {
        PORT->Group[g_APinDescription[ulPin].ulPort].WRCONFIG.reg = PORT_WRCONFIG_HWSEL |
                                                                    PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PMUX( ulPeripheral ) |
                                                                    PORT_WRCONFIG_WRPINCFG |
                                                                    PORT_WRCONFIG_PINMASK( g_APinDescription[ulPin].ulPin - 16 ) ;
      }
#else
      if ( g_APinDescription[ulPin].ulPin & 1 ) // is pin odd?
      {
        uint32_t temp ;

        // Get whole current setup for both odd and even pins and remove odd one
        temp = (PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg) & PORT_PMUX_PMUXE( 0xF ) ;
        // Set new muxing
	PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg = temp|PORT_PMUX_PMUXO( peripheral ) ;
        // Enable port mux
        PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg |= PORT_PINCFG_PMUXEN ;
      }
      else // even pin
      {
        uint32_t temp ;

        temp = (PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg) & PORT_PMUX_PMUXO( 0xF ) ;
	PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg = temp|PORT_PMUX_PMUXE( peripheral ) ;
        PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg |= PORT_PINCFG_PMUXEN ; // Enable port mux
      }
#endif
    break ;

    case PIO_NOT_A_PIN:
    case PIO_MULTI:
    default:
      return -1l ;
    break ;
  }

  return 0l ;
}

