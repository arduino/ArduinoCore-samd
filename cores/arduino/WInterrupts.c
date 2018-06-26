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
#include "sam.h"

#include <string.h>

static voidFuncPtr ISRcallback[EXTERNAL_NUM_INTERRUPTS];

#if !(SAMD51)
static uint32_t    ISRlist[EXTERNAL_NUM_INTERRUPTS];
static uint32_t nints; // Stores total number of attached interrupts
#endif

/* Configure I/O interrupt sources */
static void __initialize()
{
  memset(ISRcallback, 0, sizeof(ISRcallback));

#if (SAMD51)
  uint8_t irq;
  for (uint8_t i=0; i<EXTERNAL_NUM_INTERRUPTS; i++) {
    irq = (EIC_0_IRQn + i);
    NVIC_DisableIRQ(irq);
    NVIC_ClearPendingIRQ(irq);
    NVIC_SetPriority(irq, 0);
    NVIC_EnableIRQ(irq);
  }
#else
  memset(ISRlist,     0, sizeof(ISRlist));
  nints = 0;

  NVIC_DisableIRQ(EIC_IRQn);
  NVIC_ClearPendingIRQ(EIC_IRQn);
  NVIC_SetPriority(EIC_IRQn, 0);
  NVIC_EnableIRQ(EIC_IRQn);
#endif

  // Enable GCLK for IEC (External Interrupt Controller)
#if (SAMD21 || SAMD11)
  GCLK->CLKCTRL.reg = ( GCLK_CLKCTRL_ID( GCM_EIC ) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN );
  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );
#elif (SAML21 || SAMC21)
  MCLK->APBAMASK.reg |= MCLK_APBAMASK_EIC;
  GCLK->PCHCTRL[GCM_EIC].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0 );
  while ( (GCLK->PCHCTRL[GCM_EIC].reg & GCLK_PCHCTRL_CHEN) != GCLK_PCHCTRL_CHEN );	// wait for sync
#elif (SAMD51)
  GCLK->PCHCTRL[GCM_EIC].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK10 );  // use 96MHz clock (100MHz max for EIC) from GCLK10, which was setup in startup.c
  while ( (GCLK->PCHCTRL[GCM_EIC].reg & GCLK_PCHCTRL_CHEN) != GCLK_PCHCTRL_CHEN );      // wait for sync
#else
  #error "WInterrupts.c: Unsupported chip"
#endif

// SAMD51, SAML, and SAMC EIC.CONFIG registers are enable-protected, thus must be disabled/enabled for each config change (see below)
#if (SAMD21 || SAMD11)
/* Shall we do that?
  // Do a software reset on EIC
  EIC->CTRL.SWRST.bit = 1 ;
  while ((EIC->CTRL.SWRST.bit == 1) && (EIC->STATUS.SYNCBUSY.bit == 1)) { }
*/

  // Enable EIC
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

#if (SAML21)
  // The CHANGE and RISING interrupt modes on pin A31 on the SAML21 do not seem to work properly
  if ((GetPort(pin) == 0) && (GetPin(pin) == 31) && ((mode == CHANGE) || (mode == RISING)))
    return;
#endif

  EExt_Interrupts in = GetExtInt(pin);

#if defined(EXTERNAL_INT_NMI)
  if (in == NOT_AN_INTERRUPT || in == EXTERNAL_INT_NMI) return;
#else
  if (in == NOT_AN_INTERRUPT) return;
#endif

  if (!enabled) {
    __initialize();
    enabled = 1;
  }

  uint32_t inMask = 1 << in;

  // Enable wakeup capability on pin in case being used during sleep (WAKEUP always enabled on SAML and SAMC)
#if (SAMD21 || SAMD11)
  EIC->WAKEUP.reg |= (inMask);
#endif

  // Assign pin to EIC
  if (pinPeripheral(pin, PIO_EXTINT) != RET_STATUS_OK)
    return;

  // Only store when there is really an ISR to call.
  // This allow for calling attachInterrupt(pin, NULL, mode), we set up all needed register
  // but won't service the interrupt, this way we also don't need to check it inside the ISR.
  if (callback)
  {
#if (SAMD51)
    ISRcallback[in] = callback; // List of callback adresses
#else
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
#endif

    // Look for right CONFIG register to be addressed
    if (in > EXTERNAL_INT_7) {
      config = 1;
    } else {
      config = 0;
    }

    // Configure the interrupt mode
    pos = (in - (8 * config)) << 2;                               // compute position (ie: 0, 4, 8, 12, ...)

    #if (SAML21 || SAMC21 || SAMD51)
    EIC->CTRLA.reg = 0;   // disable EIC before changing CONFIG
    while (EIC->SYNCBUSY.reg & EIC_SYNCBUSY_MASK) { }
    #endif

    uint32_t regConfig = (~(EIC_CONFIG_SENSE0_Msk << pos) & EIC->CONFIG[config].reg);             // copy register to variable, clearing mode bits
    // insert new mode and write to register (the hardware numbering for the 5 interrupt modes is in reverse order to the arduino numbering, so using '5-mode').
    EIC->CONFIG[config].reg = (regConfig | ((5-mode) << pos));

    #if (SAML21 || SAMC21 || SAMD51)
    EIC->CTRLA.reg = EIC_CTRLA_ENABLE;    // enable EIC
    while (EIC->SYNCBUSY.reg & EIC_SYNCBUSY_MASK) { }
    #endif
  }

  // Clear the interrupt flag
  EIC->INTFLAG.reg = (inMask);

  // Enable the interrupt
  EIC->INTENSET.reg = (inMask);
}

/*
 * \brief Turns off the given interrupt.
 */
void detachInterrupt(uint32_t pin)
{
  EExt_Interrupts in = GetExtInt(pin);
#if defined(EXTERNAL_INT_NMI)
  if (in == NOT_AN_INTERRUPT || in == EXTERNAL_INT_NMI) return;
#else
  if (in == NOT_AN_INTERRUPT) return;
#endif

  uint32_t inMask = 1 << in;
  EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT(inMask);

  // Disable wakeup capability on pin during sleep (WAKEUP always enabled on SAMD51, SAML, and SAMC)
#if (SAMD21 || SAMD11)
  EIC->WAKEUP.reg &= ~(inMask);
#endif

#if !(SAMD51)
  // Remove callback from the ISR list
  uint32_t current;
  for (current=0; current<nints; current++) {
    if (ISRlist[current] == inMask) {
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
#endif
}

/*
 * External Interrupt Controller NVIC Interrupt Handler
 */
#if (SAMD51)
void EIC_0_Handler(void)
{
  ISRcallback[0]();  // Call the callback function
  EIC->INTFLAG.reg = (1 << 0);  // Clear the interrupt
}

void EIC_1_Handler(void)
{
  ISRcallback[1]();  // Call the callback function
  EIC->INTFLAG.reg = (1 << 1);  // Clear the interrupt
}

void EIC_2_Handler(void)
{
  ISRcallback[2]();  // Call the callback function
  EIC->INTFLAG.reg = (1 << 2);  // Clear the interrupt
}

void EIC_3_Handler(void)
{
  ISRcallback[3]();  // Call the callback function
  EIC->INTFLAG.reg = (1 << 3);  // Clear the interrupt
}

void EIC_4_Handler(void)
{
  ISRcallback[4]();  // Call the callback function
  EIC->INTFLAG.reg = (1 << 4);  // Clear the interrupt
}

void EIC_5_Handler(void)
{
  ISRcallback[5]();  // Call the callback function
  EIC->INTFLAG.reg = (1 << 5);  // Clear the interrupt
}

void EIC_6_Handler(void)
{
  ISRcallback[6]();  // Call the callback function
  EIC->INTFLAG.reg = (1 << 6);  // Clear the interrupt
}

void EIC_7_Handler(void)
{
  ISRcallback[7]();  // Call the callback function
  EIC->INTFLAG.reg = (1 << 7);  // Clear the interrupt
}

void EIC_8_Handler(void)
{
  ISRcallback[8]();  // Call the callback function
  EIC->INTFLAG.reg = (1 << 8);  // Clear the interrupt
}

void EIC_9_Handler(void)
{
  ISRcallback[9]();  // Call the callback function
  EIC->INTFLAG.reg = (1 << 9);  // Clear the interrupt
}

void EIC_10_Handler(void)
{
  ISRcallback[10]();  // Call the callback function
  EIC->INTFLAG.reg = (1 << 10);  // Clear the interrupt
}

void EIC_11_Handler(void)
{
  ISRcallback[11]();  // Call the callback function
  EIC->INTFLAG.reg = (1 << 11);  // Clear the interrupt
}

void EIC_12_Handler(void)
{
  ISRcallback[12]();  // Call the callback function
  EIC->INTFLAG.reg = (1 << 12);  // Clear the interrupt
}

void EIC_13_Handler(void)
{
  ISRcallback[13]();  // Call the callback function
  EIC->INTFLAG.reg = (1 << 13);  // Clear the interrupt
}

void EIC_14_Handler(void)
{
  ISRcallback[14]();  // Call the callback function
  EIC->INTFLAG.reg = (1 << 14);  // Clear the interrupt
}

void EIC_15_Handler(void)
{
  ISRcallback[15]();  // Call the callback function
  EIC->INTFLAG.reg = (1 << 15);  // Clear the interrupt
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
#endif
