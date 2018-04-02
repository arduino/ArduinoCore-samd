/*
  Copyright (c) 2016 Arduino LLC.  All right reserved.

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
  
#include <Arduino.h>

#include "DMA.h"

int DMAClass::_beginCount = 0;

DMAClass::DMAClass() :
  _channelMask(0)
{
  memset(_transferCompleteCallbacks, 0x00, sizeof(_transferCompleteCallbacks));
  memset(_transferErrorCallbacks, 0x00, sizeof(_transferErrorCallbacks));

  memset(_descriptors, 0x00, sizeof(_descriptors));
  memset(_descriptorsWriteBack, 0x00, sizeof(_descriptorsWriteBack));
}

DMAClass::~DMAClass()
{
}

void DMAClass::begin()
{
  if (_beginCount == 0) {

#if defined(__SAMD51__)
    MCLK->AHBMASK.bit.DMAC_ = 1;
#else
    // enable the DMA interface
    PM->AHBMASK.bit.DMAC_ = 1;
    PM->APBBMASK.bit.DMAC_ = 1;
#endif

    // perform a reset
    DMAC->CTRL.bit.SWRST = 1;

    // configure the descriptor addresses
    DMAC->BASEADDR.bit.BASEADDR = (uint32_t)_descriptors;
    DMAC->WRBADDR.bit.WRBADDR = (uint32_t)_descriptorsWriteBack;

    // enable with all levels
    DMAC->CTRL.bit.LVLEN0 = 1;
    DMAC->CTRL.bit.LVLEN1 = 1;
    DMAC->CTRL.bit.LVLEN2 = 1;
    DMAC->CTRL.bit.LVLEN3 = 1;
    DMAC->CTRL.bit.DMAENABLE = 1;

#if defined(__SAMD51__)
    NVIC_DisableIRQ(DMAC_0_IRQn);
    NVIC_ClearPendingIRQ(DMAC_0_IRQn);
    NVIC_EnableIRQ(DMAC_0_IRQn);
    NVIC_SetPriority(DMAC_0_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

    NVIC_DisableIRQ(DMAC_1_IRQn);
    NVIC_ClearPendingIRQ(DMAC_1_IRQn);
    NVIC_EnableIRQ(DMAC_1_IRQn);
    NVIC_SetPriority(DMAC_1_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

    NVIC_DisableIRQ(DMAC_2_IRQn);
    NVIC_ClearPendingIRQ(DMAC_2_IRQn);
    NVIC_EnableIRQ(DMAC_2_IRQn);
    NVIC_SetPriority(DMAC_2_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

    NVIC_DisableIRQ(DMAC_3_IRQn);
    NVIC_ClearPendingIRQ(DMAC_3_IRQn);
    NVIC_EnableIRQ(DMAC_3_IRQn);
    NVIC_SetPriority(DMAC_3_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

    NVIC_DisableIRQ(DMAC_4_IRQn);
    NVIC_ClearPendingIRQ(DMAC_4_IRQn);
    NVIC_EnableIRQ(DMAC_4_IRQn);
    NVIC_SetPriority(DMAC_4_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
#else
    NVIC_EnableIRQ(DMAC_IRQn);
    NVIC_SetPriority(DMAC_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
#endif
  }

  _beginCount++;
}

void DMAClass::end()
{
  _beginCount--;

  if (_beginCount == 0) {
    // disable the interrupt
#if defined(__SAMD51__)
      NVIC_DisableIRQ(DMAC_0_IRQn);
      NVIC_DisableIRQ(DMAC_1_IRQn);
      NVIC_DisableIRQ(DMAC_2_IRQn);
      NVIC_DisableIRQ(DMAC_3_IRQn);
      NVIC_DisableIRQ(DMAC_4_IRQn);
#else
    NVIC_DisableIRQ(DMAC_IRQn);
#endif

    // disable
    DMAC->CTRL.bit.DMAENABLE = 0;

    // disable the DMA interface
#if defined(__SAMD51__)
    MCLK->AHBMASK.bit.DMAC_ = 0;
#else
    // enable the DMA interface
    PM->AHBMASK.bit.DMAC_ = 0;
    PM->APBBMASK.bit.DMAC_ = 0;
#endif
  }
}

int DMAClass::allocateChannel()
{
  int channel = -1;

  // try to find a free DMA channel
  for (int i = 0; i < NUM_DMA_CHANNELS; i++) {
    if ((_channelMask & (1 << i)) == 0) {
      // found one, set the mask bit to indicate it is allocated
      _channelMask |= (1 << i);

      // clear the descriptor for the channel
      memset((void*)&_descriptors[i], 0x00, sizeof(_descriptors[i]));

      // select the channel and reset it
#if defined(__SAMD51__)
      DMAC->Channel[i].CHCTRLA.bit.ENABLE = 0;
      DMAC->Channel[i].CHCTRLA.bit.SWRST = 1;
#else
      DMAC->CHID.bit.ID = i;
      DMAC->CHCTRLA.bit.ENABLE = 0;
      DMAC->CHCTRLA.bit.SWRST = 1;
#endif

      channel = i;
      break;
    }
  }

  return channel;
}

void DMAClass::freeChannel(int channel)
{
  // select the channel and disable it
#if defined(__SAMD51__)
      DMAC->Channel[channel].CHCTRLA.bit.ENABLE = 0;
#else
      DMAC->CHID.bit.ID = channel;
      DMAC->CHCTRLA.bit.ENABLE = 0;
#endif

  _channelMask &= ~(1 << channel);
}

void DMAClass::setPriorityLevel(int channel, int level)
{
  // select the channel and set priority level
#if defined(__SAMD51__)
      
      DMAC->Channel[channel].CHPRILVL.reg = level;
#else
      DMAC->CHID.bit.ID = channel;
      DMAC->CHCTRLB.bit.LVL = level;
#endif
}

void DMAClass::setTriggerSource(int channel, int source)
{
#if defined(__SAMD51__)
  DMAC->Channel[channel].CHCTRLA.bit.TRIGSRC = source;
  DMAC->Channel[channel].CHCTRLA.bit.TRIGACT = DMAC_CHCTRLA_TRIGACT_BLOCK_Val;
#else
  // select the channel and set a trigger source
  DMAC->CHID.bit.ID = channel;
  DMAC->CHCTRLB.bit.TRIGSRC = source;

  if (DMAC->CHCTRLB.bit.TRIGSRC) {
    // if it's not a software source (0), set trigger action a a beat
    DMAC->CHCTRLB.bit.TRIGACT = DMAC_CHCTRLB_TRIGACT_BEAT_Val;
  } else {
    DMAC->CHCTRLB.bit.TRIGACT = DMAC_CHCTRLB_TRIGACT_BLOCK_Val;
  }
#endif
}

void DMAClass::setTransferWidth(int channel, int transferWidth)
{
  // select the channel and set transfer width
  switch (transferWidth) {
    case 8:
    default:
      _descriptors[channel].BTCTRL.bit.BEATSIZE = DMAC_BTCTRL_BEATSIZE_BYTE_Val;
      break;

    case 16:
      _descriptors[channel].BTCTRL.bit.BEATSIZE = DMAC_BTCTRL_BEATSIZE_HWORD_Val;
      break;

    case 32:
      _descriptors[channel].BTCTRL.bit.BEATSIZE = DMAC_BTCTRL_BEATSIZE_WORD_Val;
      break;
  }
}

void DMAClass::incSrc(int channel)
{
  // select the channel and enable source increment
  _descriptors[channel].BTCTRL.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_SRC_Val;
  _descriptors[channel].BTCTRL.bit.SRCINC = 1;
}

void DMAClass::incDst(int channel)
{
  // select the channel and enable destination increment
  _descriptors[channel].BTCTRL.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_DST_Val;
  _descriptors[channel].BTCTRL.bit.DSTINC = 1;
}

int DMAClass::transfer(int channel, void* src, void* dst, uint16_t size)
{
  if (_descriptors[channel].BTCTRL.bit.VALID) {
    // transfer in progress, fail
    return 1;
  }


#if !defined(__SAMD51__)
  // select the channel
  DMAC->CHID.bit.ID = channel;
#endif

  // disable event output generation and block actions
  _descriptors[channel].BTCTRL.bit.EVOSEL = DMAC_BTCTRL_EVOSEL_DISABLE_Val;
  _descriptors[channel].BTCTRL.bit.BLOCKACT = DMAC_BTCTRL_BLOCKACT_NOACT_Val;

  // map beat size to transfer width in bytes
  int transferWidth;

  switch (_descriptors[channel].BTCTRL.bit.BEATSIZE) {
    case DMAC_BTCTRL_BEATSIZE_BYTE_Val:
    default:
      transferWidth = 1;
      break;

    case DMAC_BTCTRL_BEATSIZE_HWORD_Val:
      transferWidth = 2;
      break;

    case DMAC_BTCTRL_BEATSIZE_WORD_Val:
      transferWidth = 4;
      break;
  }

  // set step size to 1, source + destination addresses, no next descriptor block count
  _descriptors[channel].BTCTRL.bit.STEPSIZE = DMAC_BTCTRL_STEPSIZE_X1_Val;
  _descriptors[channel].SRCADDR.bit.SRCADDR = (uint32_t)src;
  _descriptors[channel].DSTADDR.bit.DSTADDR = (uint32_t)dst;
  _descriptors[channel].DESCADDR.bit.DESCADDR = 0;
  _descriptors[channel].BTCNT.bit.BTCNT = size / transferWidth;

  if (_descriptors[channel].BTCTRL.bit.SRCINC) {
    // if increment source is set, the source address must be the end address
    _descriptors[channel].SRCADDR.bit.SRCADDR += size;
  }

  if (_descriptors[channel].BTCTRL.bit.DSTINC) {
    // if increment destination is set, the destination address must be the end address
    _descriptors[channel].DSTADDR.bit.DSTADDR += size;
  }

  // validate the descriptor
  _descriptors[channel].BTCTRL.bit.VALID = 1;

#if defined(__SAMD51__)
  DMAC->Channel[channel].CHINTENSET.bit.TERR = 1;
  DMAC->Channel[channel].CHINTENSET.bit.TCMPL = 1;
  DMAC->Channel[channel].CHCTRLA.bit.ENABLE = 1;

  if (DMAC->Channel[channel].CHCTRLA.bit.TRIGSRC == 0) {
    // uses software trigger, so trigger it
    DMAC->SWTRIGCTRL.reg |= (1 << channel);
  }
#else
  // enable channel and transfer error + complete interrupts
  DMAC->CHINTENSET.bit.TERR = 1;
  DMAC->CHINTENSET.bit.TCMPL = 1;
  DMAC->CHCTRLA.bit.ENABLE = 1;

  if (DMAC->CHCTRLB.bit.TRIGSRC == 0) {
    // uses software trigger, so trigger it
    DMAC->SWTRIGCTRL.reg |= (1 << channel);
  }
#endif

  return 0;
}

void DMAClass::onTransferComplete(int channel, void(*function)(int))
{
  _transferCompleteCallbacks[channel] = function;
}

void DMAClass::onTransferError(int channel, void(*function)(int))
{
  _transferErrorCallbacks[channel] = function;
}

void DMAClass::onService()
{
  // get the channel and select it
  int channel = DMAC->INTPEND.bit.ID;
#if !defined(__SAMD51__)
  DMAC->CHID.bit.ID = channel;
#endif

  // invalidate the channel
  _descriptors[channel].BTCTRL.bit.VALID = 0;

#if defined(__SAMD51__)
  if (DMAC->Channel[channel].CHINTFLAG.bit.TERR) {
    // clear the error interrupt and call the error callback if there is one
    DMAC->Channel[channel].CHINTFLAG.bit.TERR = 1;

    if (_transferErrorCallbacks[channel]) {
      _transferErrorCallbacks[channel](channel);
    }
  }

  if (DMAC->Channel[channel].CHINTFLAG.bit.TCMPL) {
    // clear the complete interrupt and call the callback if there is one
    DMAC->Channel[channel].CHINTFLAG.bit.TCMPL = 1;

    if (_transferCompleteCallbacks[channel]) {
      _transferCompleteCallbacks[channel](channel);
    }
  }
#else
  if (DMAC->CHINTFLAG.bit.TERR) {
    // clear the error interrupt and call the error callback if there is one
    DMAC->CHINTFLAG.bit.TERR = 1;

    if (_transferErrorCallbacks[channel]) {
      _transferErrorCallbacks[channel](channel);
    }
  }

  if (DMAC->CHINTFLAG.bit.TCMPL) {
    // clear the complete interrupt and call the callback if there is one
    DMAC->CHINTFLAG.bit.TCMPL = 1;

    if (_transferCompleteCallbacks[channel]) {
      _transferCompleteCallbacks[channel](channel);
    }
  }
#endif
}

extern "C" {
#if defined(__SAMD51__)
  static void _dmac_handler(void)
{
  DMA.onService();
}
/**
* \brief DMAC interrupt handler
*/
void DMAC_0_Handler(void)
{
  _dmac_handler();
}
/**
* \brief DMAC interrupt handler
*/
void DMAC_1_Handler(void)
{
  _dmac_handler();
}
/**
* \brief DMAC interrupt handler
*/
void DMAC_2_Handler(void)
{
  _dmac_handler();
}
/**
* \brief DMAC interrupt handler
*/
void DMAC_3_Handler(void)
{
  _dmac_handler();
}
/**
* \brief DMAC interrupt handler
*/
void DMAC_4_Handler(void)
{
  _dmac_handler();
}
#else
  void DMAC_Handler() {
    DMA.onService();
  }
#endif
}

DMAClass DMA;
