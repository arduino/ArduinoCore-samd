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

#ifdef __cplusplus
 extern "C" {
#endif

void pinMode( uint32_t ulPin, uint32_t ulMode )
{
  // Set pin mode according to chapter '22.6.3 I/O Pin Configuration'
  pinPeripheral(ulPin, ulMode);
}

void digitalWrite( uint32_t ulPin, uint32_t ulVal )
{
  // Handle the case the pin isn't usable as PIO
  if ( g_APinDescription[ulPin].ulPinType == PIO_NOT_A_PIN )
  {
    return ;
  }

  uint32_t pinAttribute = g_APinDescription[ulPin].ulPinAttribute;
  uint8_t pinPort = g_APinDescription[ulPin].ulPort;
  uint8_t pinNum = g_APinDescription[ulPin].ulPin;
  uint8_t pullConfig = PORT->Group[pinPort].PINCFG[pinNum].reg;
  
  // Enable pull resistor if pin attributes allow
  if ( (ulVal == HIGH && (pinAttribute & PIN_ATTR_INPUT_PULLUP)) || (ulVal == LOW && (pinAttribute & PIN_ATTR_INPUT_PULLDOWN)) )
  {
    pullConfig |= (uint8_t)(PORT_PINCFG_PULLEN) ;
  }
  else
  {
    pullConfig &= ~(uint8_t)(PORT_PINCFG_PULLEN) ;
  }

  PORT->Group[pinPort].PINCFG[pinNum].reg = pullConfig ;

  switch ( ulVal )
  {
    case LOW:
      PORT->Group[pinPort].OUTCLR.reg = (1ul << pinNum) ;
    break ;

    case HIGH:
      PORT->Group[pinPort].OUTSET.reg = (1ul << pinNum) ;
    break ;

    default:
    break ;
  }

  return ;
}

int digitalRead( uint32_t ulPin )
{
  // Handle the case the pin isn't usable as PIO
  if ( g_APinDescription[ulPin].ulPinType == PIO_NOT_A_PIN )
  {
    return LOW ;
  }

  if ( (PORT->Group[g_APinDescription[ulPin].ulPort].IN.reg & (1ul << g_APinDescription[ulPin].ulPin)) != 0 )
  {
    return HIGH ;
  }

  return LOW ;
}

#ifdef __cplusplus
}
#endif

