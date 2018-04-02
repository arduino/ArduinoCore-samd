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

static voidFuncPtr callbacksInt[EXTERNAL_NUM_INTERRUPTS];

/* Configure I/O interrupt sources */
static void __initialize()
{
  memset(callbacksInt, 0, sizeof(callbacksInt));

  NVIC_DisableIRQ(EIC_IRQn);
  NVIC_ClearPendingIRQ(EIC_IRQn);
  NVIC_SetPriority(EIC_IRQn, 0);
  NVIC_EnableIRQ(EIC_IRQn);

  // Enable GCLK for IEC (External Interrupt Controller)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_EIC));

/* Shall we do that?
  // Do a software reset on EIC
  EIC->CTRL.SWRST.bit = 1 ;
  while ((EIC->CTRL.SWRST.bit == 1) && (EIC->STATUS.SYNCBUSY.bit == 1)) { }
*/

  // Enable EIC
  EIC->CTRL.bit.ENABLE = 1;
  while (EIC->STATUS.bit.SYNCBUSY == 1) { }
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

#if ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10606
  EExt_Interrupts in = g_APinDescription[pin].ulExtInt;
#else
  EExt_Interrupts in = digitalPinToInterrupt(pin);
#endif
  if (in == NOT_AN_INTERRUPT) return;

  if (!enabled) {
    __initialize();
    enabled = 1;
  }

  if (in == EXTERNAL_INT_NMI) {
    EIC->NMIFLAG.bit.NMI = 1; // Clear flag
    switch (mode) {
      case LOW:
        EIC->NMICTRL.bit.NMISENSE = EIC_NMICTRL_NMISENSE_LOW;
        break;

      case HIGH:
        EIC->NMICTRL.bit.NMISENSE = EIC_NMICTRL_NMISENSE_HIGH;
        break;

      case CHANGE:
        EIC->NMICTRL.bit.NMISENSE = EIC_NMICTRL_NMISENSE_BOTH;
        break;

      case FALLING:
        EIC->NMICTRL.bit.NMISENSE = EIC_NMICTRL_NMISENSE_FALL;
        break;

      case RISING:
        EIC->NMICTRL.bit.NMISENSE = EIC_NMICTRL_NMISENSE_RISE;
        break;
    }

    // Assign callback to interrupt
    callbacksInt[EXTERNAL_INT_NMI] = callback;

  } else { // Not NMI, is external interrupt

    // Enable wakeup capability on pin in case being used during sleep
    EIC->WAKEUP.reg |= (1 << in);

    // Assign pin to EIC
    pinPeripheral(pin, PIO_EXTINT);

    // Look for right CONFIG register to be addressed
    if (in > EXTERNAL_INT_7) {
      config = 1;
    } else {
      config = 0;
    }

    // Configure the interrupt mode
    pos = (in - (8 * config)) << 2;
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
    EIC->INTENSET.reg = EIC_INTENSET_EXTINT(1 << in);

    // Assign callback to interrupt
    callbacksInt[in] = callback;
  }
}

/*
 * \brief Turns off the given interrupt.
 */
void detachInterrupt(uint32_t pin)
{
#if (ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10606)
  EExt_Interrupts in = g_APinDescription[pin].ulExtInt;
#else
  EExt_Interrupts in = digitalPinToInterrupt(pin);
#endif 
  if (in == NOT_AN_INTERRUPT) return;

  if(in == EXTERNAL_INT_NMI) {
    EIC->NMICTRL.bit.NMISENSE = 0; // Turn off detection
  } else {
    EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT(1 << in);
  
    // Disable wakeup capability on pin during sleep
    EIC->WAKEUP.reg &= ~(1 << in);
  }
}

/*
 * External Interrupt Controller NVIC Interrupt Handler
 */
void EIC_Handler(void)
{
  // Test the 16 normal interrupts
  for (uint32_t i=EXTERNAL_INT_0; i<=EXTERNAL_INT_15; i++)
  {
    if ((EIC->INTFLAG.reg & (1 << i)) != 0)
    {
      // Call the callback function if assigned
      if (callbacksInt[i]) {
        callbacksInt[i]();
      }

      // Clear the interrupt
      EIC->INTFLAG.reg = 1 << i;
    }
  }
}

/*
 * NMI Interrupt Handler
 */
void NMI_Handler(void)
{
  if (callbacksInt[EXTERNAL_INT_NMI]) callbacksInt[EXTERNAL_INT_NMI]();
  EIC->NMIFLAG.bit.NMI = 1; // Clear interrupt
}
