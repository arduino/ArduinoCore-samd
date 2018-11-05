/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  SAMD51 support added by Adafruit - Copyright (c) 2018 Dean Miller for Adafruit Industries

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

static voidFuncPtr ISRcallback[EXTERNAL_NUM_INTERRUPTS];
static uint32_t    ISRlist[EXTERNAL_NUM_INTERRUPTS];
static uint32_t    nints; // Stores total number of attached interrupts


/* Configure I/O interrupt sources */
static void __initialize()
{
  memset(ISRlist,     0, sizeof(ISRlist));
  memset(ISRcallback, 0, sizeof(ISRcallback));
  nints = 0;

#if defined(__SAMD51__)
  ///EIC MCLK is enabled by default
  for (uint32_t i = 0; i <= 15; i++)     // EIC_0_IRQn = 12 ... EIC_15_IRQn = 27
  {
    uint8_t irqn = EIC_0_IRQn + i;
    NVIC_DisableIRQ(irqn);
    NVIC_ClearPendingIRQ(irqn);
    NVIC_SetPriority(irqn, 0);
    NVIC_EnableIRQ(irqn);
  }
  
  GCLK->PCHCTRL[EIC_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK2_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
#else
  NVIC_DisableIRQ(EIC_IRQn);
  NVIC_ClearPendingIRQ(EIC_IRQn);
  NVIC_SetPriority(EIC_IRQn, 0);
  NVIC_EnableIRQ(EIC_IRQn);

  // Enable GCLK for IEC (External Interrupt Controller)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_EIC));
#endif

/* Shall we do that?
  // Do a software reset on EIC
  EIC->CTRL.SWRST.bit = 1 ;
  while ((EIC->CTRL.SWRST.bit == 1) && (EIC->STATUS.SYNCBUSY.bit == 1)) { }
*/

  // Enable EIC
#if defined(__SAMD51__)
  EIC->CTRLA.bit.ENABLE = 1;
  while (EIC->SYNCBUSY.bit.ENABLE == 1) { }
#else
  EIC->CTRL.bit.ENABLE = 1;
  while (EIC->STATUS.bit.SYNCBUSY == 1) { }
#endif
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
	uint32_t inMask = (1UL << in);
	// Enable wakeup capability on pin in case being used during sleep
	#if defined(__SAMD51__)
	//I believe this is done automatically
	#else
	EIC->WAKEUP.reg |= (1 << in);
	#endif

	// Only store when there is really an ISR to call.
	// This allow for calling attachInterrupt(pin, NULL, mode), we set up all needed register
	// but won't service the interrupt, this way we also don't need to check it inside the ISR.
	if (callback)
	{
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
			ISRcallback[EXTERNAL_INT_NMI] = callback;

		} else { // Not NMI, is external interrupt

			// Assign pin to EIC
			pinPeripheral(pin, PIO_EXTINT);

			// Store interrupts to service in order of when they were attached
			// to allow for first come first serve handler
			uint32_t current = 0;

			// Check if we already have this interrupt
			for (current=0; current<nints; current++) {
			  if (ISRlist[current] == inMask) {
				break;
			  }
			}
			if (current == nints) {
			  // Need to make a new entry
			  nints++;
			}
			ISRlist[current] = inMask;       // List of interrupt in order of when they were attached
			ISRcallback[current] = callback; // List of callback adresses

			// Look for right CONFIG register to be addressed
			if (in > EXTERNAL_INT_7) {
			  config = 1;
			  pos = (in - 8) << 2;
			} else {
			  config = 0;
			  pos = in << 2;
			}

			#if defined (__SAMD51__)
			EIC->CTRLA.bit.ENABLE = 0;
			while (EIC->SYNCBUSY.bit.ENABLE == 1) { }
			#endif

			EIC->CONFIG[config].reg &=~ (EIC_CONFIG_SENSE0_Msk << pos); // Reset sense mode, important when changing trigger mode during runtime
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
		}
		// Enable the interrupt
		EIC->INTENSET.reg = EIC_INTENSET_EXTINT(1 << in);
	}

	#if defined (__SAMD51__)
	EIC->CTRLA.bit.ENABLE = 1;
	while (EIC->SYNCBUSY.bit.ENABLE == 1) { }
	#endif
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
#if defined(__SAMD51__)
//I believe this is done automatically
#else
    // Disable wakeup capability on pin during sleep
  EIC->WAKEUP.reg &= ~(1 << in);
#endif
  }

  // Remove callback from the ISR list
  uint32_t current;
  for (current=0; current<nints; current++) {
    if (ISRlist[current] == (1UL << in)) {
      break;
    }
  }
  if (current == nints) return; // We didn't have it

  // Shift the reminder down
  for (; current<nints-1; current++) {
    ISRlist[current]     = ISRlist[current+1];
    ISRcallback[current] = ISRcallback[current+1];
  }
  nints--;
}

/*
 * External Interrupt Controller NVIC Interrupt Handler
 */
#if defined(__SAMD51__)
void InterruptHandler(uint32_t i)
{
  // Calling the routine directly from -here- takes about 1us
  // Depending on where you are in the list it will take longer

  // Loop over all enabled interrupts in the list
  for (uint32_t i=0; i<nints; i++)
  {
	if ((EIC->INTFLAG.reg & ISRlist[i]) != 0)
	{
	  // Call the callback function
	  ISRcallback[i]();
	  // Clear the interrupt
	  EIC->INTFLAG.reg = ISRlist[i];
	}
  }
}

void EIC_0_Handler(void)
{
  InterruptHandler(EXTERNAL_INT_0);
}

void EIC_1_Handler(void)
{
  InterruptHandler(EXTERNAL_INT_1);
}

void EIC_2_Handler(void)
{
  InterruptHandler(EXTERNAL_INT_2);
}

void EIC_3_Handler(void)
{
  InterruptHandler(EXTERNAL_INT_3);
}

void EIC_4_Handler(void)
{
  InterruptHandler(EXTERNAL_INT_4);
}

void EIC_5_Handler(void)
{
  InterruptHandler(EXTERNAL_INT_5);
}

void EIC_6_Handler(void)
{
  InterruptHandler(EXTERNAL_INT_6);
}

void EIC_7_Handler(void)
{
  InterruptHandler(EXTERNAL_INT_7);
}

void EIC_8_Handler(void)
{
  InterruptHandler(EXTERNAL_INT_8);
}

void EIC_9_Handler(void)
{
  InterruptHandler(EXTERNAL_INT_9);
}

void EIC_10_Handler(void)
{
  InterruptHandler(EXTERNAL_INT_10);
}

void EIC_11_Handler(void)
{
  InterruptHandler(EXTERNAL_INT_11);
}

void EIC_12_Handler(void)
{
  InterruptHandler(EXTERNAL_INT_12);
}

void EIC_13_Handler(void)
{
  InterruptHandler(EXTERNAL_INT_13);
}

void EIC_14_Handler(void)
{
  InterruptHandler(EXTERNAL_INT_14);
}

void EIC_15_Handler(void)
{
  InterruptHandler(EXTERNAL_INT_15);
}
#else

void EIC_Handler(void)
{
  // Calling the routine directly from -here- takes about 1us
  // Depending on where you are in the list it will take longer

  // Loop over all enabled interrupts in the list
  for (uint32_t i=0; i<nints; i++)
  {
    if ((EIC->INTFLAG.reg & ISRlist[i]) != 0)
    {
      // Call the callback function
      ISRcallback[i]();
      // Clear the interrupt
      EIC->INTFLAG.reg = ISRlist[i];
    }
  }
}

/*
 * NMI Interrupt Handler
 */
void NMI_Handler(void)
{
  if (ISRcallback[EXTERNAL_INT_NMI]) ISRcallback[EXTERNAL_INT_NMI]();
  EIC->NMIFLAG.bit.NMI = 1; // Clear interrupt
}
#endif
