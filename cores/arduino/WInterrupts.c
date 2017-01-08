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

static voidFuncPtr ISRcallback[EXTERNAL_NUM_INTERRUPTS];
static EExt_Interrupts ISRlist[EXTERNAL_NUM_INTERRUPTS];
static uint32_t nints; //stores total nr of attached interrupts

//for debug
voidFuncPtr* getISRcallback(){
    return ISRcallback;
}
EExt_Interrupts* getISRlist(){
    return ISRlist;
}
//end debug



/* Configure I/O interrupt sources */
static void __initialize()
{
  memset(ISRlist, 0, sizeof(ISRlist));
  memset(ISRcallback, 0, sizeof(ISRcallback));
  nints=0;

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
  if (in == NOT_AN_INTERRUPT || in == EXTERNAL_INT_NMI)
    return;

  if (!enabled) {
    __initialize();
    enabled = 1;
  }

  // Enable wakeup capability on pin in case being used during sleep
  EIC->WAKEUP.reg |= (1 << in);

  // Assign pin to EIC
  pinPeripheral(pin, PIO_EXTINT);

  if (callback){ //only store when there is really an ISR to call
      //this allow for calling attachInterrupt(pin, NULL, mode), we set up all needed register
      //but won't service the interrupt, this way we also don't need to check it inside the ISR

    // Store interrupts to service in order of when they were attached
    // to allow for first come first serve handler
    uint32_t current=0;
    //check if we already have this interrupt
    int id=-1;
    for (uint32_t i=0; i<nints; i++){
      if (ISRlist[i]==in) id=in;
    }

    if (id==-1){
      //need to make a new entry
      current=nints;
      nints++;
    }
    else{
      //we already have an entry for this pin
      current=id;
    }
    ISRlist[current]=in; //list with nr of interrupt in order of when they were attached
    ISRcallback[current]=callback; //list of callback adresses

    // Look for right CONFIG register to be addressed
    if (in > EXTERNAL_INT_7) {
        config = 1;
    } else {
        config = 0;
    }

    // Configure the interrupt mode
    pos = (in - (8 * config)) << 2;
    EIC->CONFIG[config].reg &=~ (EIC_CONFIG_SENSE0_Msk << pos);//reset sense mode, important when changing trigger mode during runtime
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
  if (in == NOT_AN_INTERRUPT || in == EXTERNAL_INT_NMI)
    return;

  EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT(1 << in);
  
  // Disable wakeup capability on pin during sleep
  EIC->WAKEUP.reg &= ~(1 << in);

  //and take it out of the ISR list
  int id=-1; //find its location first
  for (uint32_t i=0; i<nints; i++){
      if (ISRlist[i]==in) id=in;
  }
  if (id==-1) return; //apparently we didn't have it
  for (uint32_t i=id; i<nints-1; i++){ //and shift the ones above one down
      ISRlist[i]=ISRlist[i+1];
      ISRcallback[i]=ISRcallback[i+1];
  }
  //and remove the top item
  ISRlist[nints]=0;
  ISRcallback[nints]=NULL;
  nints--;
}

/*
 * External Interrupt Controller NVIC Interrupt Handler
 */
void EIC_Handler(void)
{
  //calling the routine directly from -here- takes about 1us
  //depending on where you are in the list it will take longer
  for (uint32_t i=0; i<nints; i++) // Loop over all enabled interrupts in the list
  {  
    if ((EIC->INTFLAG.reg & 1<<ISRlist[i]) != 0)
    {  
      ISRcallback[i]();// Call the callback function if assigned
      EIC->INTFLAG.reg = 1<<ISRlist[i]; // Clear the interrupt
    }
  }

}
