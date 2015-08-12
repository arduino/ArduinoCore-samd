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

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

static struct
{
  uint32_t _ulPin ;
  voidFuncPtr _callback ;
} callbacksInt[EXTERNAL_NUM_INTERRUPTS] ;

/* Configure I/O interrupt sources */
static void __initialize()
{
  memset( callbacksInt, 0, sizeof( callbacksInt ) ) ;

  NVIC_DisableIRQ( EIC_IRQn ) ;
  NVIC_ClearPendingIRQ( EIC_IRQn ) ;
  NVIC_SetPriority( EIC_IRQn, 0 ) ;
  NVIC_EnableIRQ( EIC_IRQn ) ;

  // Enable GCLK for IEC (External Interrupt Controller)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_EIC )) ;

/* Shall we do that?
  // Do a software reset on EIC
  EIC->CTRL.SWRST.bit = 1 ;

  while ( (EIC->CTRL.SWRST.bit == 1) && (EIC->STATUS.SYNCBUSY.bit == 1) )
  {
    // Waiting for synchronisation
  }
*/

  // Enable EIC
  EIC->CTRL.bit.ENABLE = 1 ;

  while ( EIC->STATUS.bit.SYNCBUSY == 1 )
  {
    // Waiting for synchronisation
  }

}

/*
 * \brief Specifies a named Interrupt Service Routine (ISR) to call when an interrupt occurs.
 *        Replaces any previous function that was attached to the interrupt.
 */
//void attachInterrupt( uint32_t ulPin, void (*callback)(void), EExt_IntMode mode )
//void attachInterrupt( uint32_t ulPin, voidFuncPtr callback, EExt_IntMode mode )
void attachInterrupt( uint32_t ulPin, voidFuncPtr callback, uint32_t ulMode )
{
  static int enabled = 0 ;
  uint32_t ulConfig ;
  uint32_t ulPos ;

  if ( digitalPinToInterrupt( ulPin ) == NOT_AN_INTERRUPT )
  {
    return ;
  }

  if ( !enabled )
  {
    __initialize() ;
    enabled = 1 ;
  }

  // Assign pin to EIC
  pinPeripheral( ulPin, PIO_EXTINT ) ;

  // Assign callback to interrupt
  callbacksInt[digitalPinToInterrupt( ulPin )]._ulPin = ulPin ;
  callbacksInt[digitalPinToInterrupt( ulPin )]._callback = callback ;

  // Check if normal interrupt or NMI
  if ( ulPin != 2 )
  {
    // Look for right CONFIG register to be addressed
    if ( digitalPinToInterrupt( ulPin ) > EXTERNAL_INT_7 )
    {
      ulConfig = 1 ;
    }
    else
    {
      ulConfig = 0 ;
    }

    // Configure the interrupt mode
    ulPos = ((digitalPinToInterrupt( ulPin ) - (8*ulConfig) ) << 2) ;
    switch ( ulMode )
    {
      case LOW:
        EIC->CONFIG[ulConfig].reg |= EIC_CONFIG_SENSE0_LOW_Val << ulPos ;
      break ;

      case HIGH:
        // EIC->CONFIG[ulConfig].reg = EIC_CONFIG_SENSE0_HIGH_Val << ((digitalPinToInterrupt( ulPin ) >> ulConfig ) << ulPos) ;
        EIC->CONFIG[ulConfig].reg |= EIC_CONFIG_SENSE0_HIGH_Val << ulPos ;
      break ;

      case CHANGE:
        // EIC->CONFIG[ulConfig].reg = EIC_CONFIG_SENSE0_BOTH_Val << ((digitalPinToInterrupt( ulPin ) >> ulConfig ) << ulPos) ;
        EIC->CONFIG[ulConfig].reg |= EIC_CONFIG_SENSE0_BOTH_Val << ulPos ;
      break ;

      case FALLING:
        // EIC->CONFIG[ulConfig].reg = EIC_CONFIG_SENSE0_FALL_Val << ((digitalPinToInterrupt( ulPin ) >> ulConfig ) << ulPos) ;
        EIC->CONFIG[ulConfig].reg |= EIC_CONFIG_SENSE0_FALL_Val << ulPos ;
      break ;

      case RISING:
        // EIC->CONFIG[ulConfig].reg = EIC_CONFIG_SENSE0_RISE_Val << ((digitalPinToInterrupt( ulPin ) >> ulConfig ) << ulPos) ;
        EIC->CONFIG[ulConfig].reg |= EIC_CONFIG_SENSE0_RISE_Val << ulPos ;
      break ;
    }

    // Enable the interrupt
    EIC->INTENSET.reg = EIC_INTENSET_EXTINT( 1 << digitalPinToInterrupt( ulPin ) ) ;
  }
  else // Handles NMI on pin 2
  {
    // Configure the interrupt mode
    switch ( ulMode )
    {
      case LOW:
        EIC->NMICTRL.reg = EIC_NMICTRL_NMISENSE_LOW ;
      break ;

      case HIGH:
        EIC->NMICTRL.reg = EIC_NMICTRL_NMISENSE_HIGH ;
      break ;

      case CHANGE:
        EIC->NMICTRL.reg = EIC_NMICTRL_NMISENSE_BOTH ;
      break ;

      case FALLING:
        EIC->NMICTRL.reg = EIC_NMICTRL_NMISENSE_FALL ;
      break ;

      case RISING:
        EIC->NMICTRL.reg= EIC_NMICTRL_NMISENSE_RISE ;
      break ;
    }
  }
}

/*
 * \brief Turns off the given interrupt.
 */
void detachInterrupt( uint32_t ulPin )
{
/*
  // Retrieve pin information
  Pio *pio = g_APinDescription[pin].pPort;
  uint32_t mask = g_APinDescription[pin].ulPin;

  // Disable interrupt
  pio->PIO_IDR = mask;
*/
  if ( digitalPinToInterrupt( ulPin ) == NOT_AN_INTERRUPT )
  {
    return ;
  }

  EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT( 1 << digitalPinToInterrupt( ulPin ) ) ;
}

/*
 * External Interrupt Controller NVIC Interrupt Handler
 */
void EIC_Handler( void )
{
  uint32_t ul ;

  // Test the 16 normal interrupts
  for ( ul = EXTERNAL_INT_0 ; ul <= EXTERNAL_INT_15 ; ul++ )
  {
    if ( (EIC->INTFLAG.reg & ( 1 << ul ) ) != 0 )
    {
      // Call the callback function if assigned
      if ( callbacksInt[ul]._callback != NULL )
      {
        callbacksInt[ul]._callback() ;
      }

      // Clear the interrupt
      EIC->INTFLAG.reg = 1 << ul ;
    }
  }
}

/*
 * External Non-Maskable Interrupt Controller NVIC Interrupt Handler
 */
void NMI_Handler( void )
{
  // Call the callback function if assigned
  if ( callbacksInt[EXTERNAL_INT_NMI]._callback != NULL )
  {
    callbacksInt[EXTERNAL_INT_NMI]._callback() ;
  }

  // Clear the interrupt
  EIC->NMIFLAG.reg = EIC_NMIFLAG_NMI ;
}

#ifdef __cplusplus
}
#endif
