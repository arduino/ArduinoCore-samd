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

static voidFuncPtr callbacksInt[EXTERNAL_NUM_INTERRUPTS];

/* Configure I/O interrupt sources */
static void __initialize()
{
  memset(callbacksInt, 0, sizeof(callbacksInt));

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
void attachInterrupt(uint32_t pin, voidFuncPtr callback, uint32_t mode)
{
  static int enabled = 0;
  uint32_t config;
  uint32_t pos;

  if (digitalPinToInterrupt(pin) == NOT_AN_INTERRUPT)
    return;
  if (digitalPinToInterrupt(pin) == EXTERNAL_INT_NMI)
    return;

  if (!enabled) {
    __initialize();
    enabled = 1;
  }

  // Assign pin to EIC
  pinPeripheral(pin, PIO_EXTINT);

  // Assign callback to interrupt
  callbacksInt[digitalPinToInterrupt(pin)] = callback;

  // Look for right CONFIG register to be addressed
  if (digitalPinToInterrupt(pin) > EXTERNAL_INT_7) {
    config = 1;
  } else {
    config = 0;
  }

  // Configure the interrupt mode
  pos = ((digitalPinToInterrupt(pin) - (8 * config)) << 2);
  switch (mode)
  {
    case LOW:
      EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_LOW_Val << pos;
      break;

    case HIGH:
      EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_HIGH_Val << pos;
      break;

    case CHANGE:
      EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_BOTH_Val << pos;
      break;

    case FALLING:
      EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_FALL_Val << pos;
      break;

    case RISING:
      EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_RISE_Val << pos;
      break;
  }

  // Enable the interrupt
  EIC->INTENSET.reg = EIC_INTENSET_EXTINT(1 << digitalPinToInterrupt(pin));
}

/*
 * \brief Turns off the given interrupt.
 */
void detachInterrupt(uint32_t pin)
{
  if (digitalPinToInterrupt(pin) == NOT_AN_INTERRUPT)
    return;
  if (digitalPinToInterrupt(pin) == EXTERNAL_INT_NMI)
    return;

  EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT(1 << digitalPinToInterrupt(pin));
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
      if (callbacksInt[ul]) {
        callbacksInt[ul]();
      }

      // Clear the interrupt
      EIC->INTFLAG.reg = 1 << ul ;
    }
  }
}

#ifdef __cplusplus
}
#endif
