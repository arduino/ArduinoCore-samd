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
#pragma once

#if defined(__SAMD51__)
#define NUM_DMA_CHANNELS 4
#else
#define NUM_DMA_CHANNELS 1
#endif

/*
  WARNING: The API for this class may change and it's not intended for public use!
*/
class DMAClass
{
  public:
    DMAClass();
    virtual ~DMAClass();

    void begin();
    void end();

    int allocateChannel();
    void freeChannel(int channel);

    void setPriorityLevel(int channel, int level);
    void setTriggerSource(int channel, int source);
    void setTransferWidth(int channel, int transferWidth);
    void incSrc(int channel);
    void incDst(int channel);
    int transfer(int channel, void* src, void* dst, uint16_t size);

    void onTransferComplete(int channel, void(*function)(int));
    void onTransferError(int channel, void(*function)(int));

    void onService();

  private:
    static int _beginCount;
    uint32_t _channelMask;

    void (*_transferCompleteCallbacks[NUM_DMA_CHANNELS])(int);
    void (*_transferErrorCallbacks[NUM_DMA_CHANNELS])(int);

    DmacDescriptor _descriptors[NUM_DMA_CHANNELS] __attribute__ ((aligned (16)));
    DmacDescriptor _descriptorsWriteBack[NUM_DMA_CHANNELS]  __attribute__ ((aligned (16)));
};

extern DMAClass DMA;
