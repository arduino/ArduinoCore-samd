/*
  Copyright (c) 2016 Arduino LLC. 2017 Adafruit  All right reserved.

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

#include <Arduino.h>

class I2SDevice_SAMD51 {
public:
  I2SDevice_SAMD51(I2s& _i2s) :
    i2s(_i2s)
  {
    // Empty
  }

  inline void reset() {
    while(i2s.SYNCBUSY.bit.SWRST);
    i2s.CTRLA.bit.SWRST = 1;
  }

  inline void disable() {
    while(i2s.SYNCBUSY.bit.ENABLE);
    i2s.CTRLA.bit.ENABLE = 0;
  }

  inline void enable() {
    while(i2s.SYNCBUSY.bit.ENABLE);
    i2s.CTRLA.bit.ENABLE = 1;
  }

  inline int glckId(int index) {
    return (index == 0) ? I2S_GCLK_ID_0 : I2S_GCLK_ID_1;
  }

  inline void setSerialClockSelectMasterClockDiv(int index) {
    i2s.CLKCTRL[index].bit.SCKSEL = I2S_CLKCTRL_SCKSEL_MCKDIV_Val;
  }

  inline void setSerialClockSelectPin(int index) {
    i2s.CLKCTRL[index].bit.SCKSEL = I2S_CLKCTRL_SCKSEL_SCKPIN_Val;
  }

  inline void setFrameSyncSelectSerialClockDiv(int index) {
    i2s.CLKCTRL[index].bit.FSSEL = I2S_CLKCTRL_FSSEL_SCKDIV_Val;
  }

  inline void setFrameSyncSelectPin(int index) {
    i2s.CLKCTRL[index].bit.FSSEL = I2S_CLKCTRL_FSSEL_FSPIN_Val;
  }

  inline void set0BitDelay(int index) {
    i2s.CLKCTRL[index].bit.BITDELAY = I2S_CLKCTRL_BITDELAY_LJ_Val;
  }

  inline void set1BitDelay(int index) {
    i2s.CLKCTRL[index].bit.BITDELAY = I2S_CLKCTRL_BITDELAY_I2S_Val;
  }

  inline void setNumberOfSlots(int index, int nbslots) {
    i2s.CLKCTRL[index].bit.NBSLOTS = nbslots;
  }

  inline void setSlotSize(int index, int size) {
    switch (size) {
     case 32:
       i2s.CLKCTRL[index].bit.SLOTSIZE = I2S_CLKCTRL_SLOTSIZE_32_Val;
       break;

     case 24:
       i2s.CLKCTRL[index].bit.SLOTSIZE = I2S_CLKCTRL_SLOTSIZE_24_Val;
       break;

     case 16:
       i2s.CLKCTRL[index].bit.SLOTSIZE = I2S_CLKCTRL_SLOTSIZE_16_Val;
       break;

     case 8:
       i2s.CLKCTRL[index].bit.SLOTSIZE = I2S_CLKCTRL_SLOTSIZE_8_Val;
       break;
    }
  }

  inline void setDataSize(int index, int size) {
    switch (size) {
     case 32:
       i2s.RXCTRL.bit.DATASIZE = I2S_RXCTRL_DATASIZE_32_Val;
       i2s.TXCTRL.bit.DATASIZE = I2S_TXCTRL_DATASIZE_32_Val;
       break;

     case 24:
       i2s.RXCTRL.bit.DATASIZE = I2S_RXCTRL_DATASIZE_24_Val;
       i2s.TXCTRL.bit.DATASIZE = I2S_TXCTRL_DATASIZE_24_Val;
       break;

     case 16:
       i2s.RXCTRL.bit.DATASIZE = I2S_RXCTRL_DATASIZE_16_Val;
       i2s.TXCTRL.bit.DATASIZE = I2S_TXCTRL_DATASIZE_16_Val;
       break;

     case 8:
       i2s.RXCTRL.bit.DATASIZE = I2S_RXCTRL_DATASIZE_8_Val;
       i2s.TXCTRL.bit.DATASIZE = I2S_TXCTRL_DATASIZE_8_Val;
       break;
    }
  }

  inline void setSlotAdjustedRight(int index) {
    i2s.RXCTRL.bit.SLOTADJ = I2S_RXCTRL_SLOTADJ_RIGHT_Val;
    i2s.TXCTRL.bit.SLOTADJ = I2S_TXCTRL_SLOTADJ_RIGHT_Val;
  }

  inline void setSlotAdjustedLeft(int index) {
    i2s.RXCTRL.bit.SLOTADJ = I2S_RXCTRL_SLOTADJ_LEFT_Val;
    i2s.TXCTRL.bit.SLOTADJ = I2S_TXCTRL_SLOTADJ_LEFT_Val;
  }

  inline void setClockUnit(int index) {
    i2s.RXCTRL.bit.CLKSEL = (index == 0) ? I2S_RXCTRL_CLKSEL_CLK0_Val : I2S_RXCTRL_CLKSEL_CLK1_Val;
  }

  inline void setTxMode(int index) {
    i2s.RXCTRL.bit.SERMODE = 0x01;
    i2s.TXCTRL.reg &= ~(0x03); //TODO: why is this not in CMSIS...
    i2s.TXCTRL.reg |= 0x01;
  }

  inline void setRxMode(int index) {
    i2s.RXCTRL.bit.SERMODE = 0x00;
    i2s.TXCTRL.reg &= ~(0x03); //TODO: why is this not in CMSIS...
    i2s.TXCTRL.reg |= 0x00;
  }

  inline void enableClockUnit(int index) {
    if (index == 0) {
      while(i2s.SYNCBUSY.bit.CKEN0);
      i2s.CTRLA.bit.CKEN0 = 1;
    } else {
      while(i2s.SYNCBUSY.bit.CKEN1);
      i2s.CTRLA.bit.CKEN1 = 1;
    }
  }

  inline void disableClockUnit(int index) {
    if (index == 0) {
      while(i2s.SYNCBUSY.bit.CKEN0);
      i2s.CTRLA.bit.CKEN0 = 0;
    } else {
      while(i2s.SYNCBUSY.bit.CKEN1);
      i2s.CTRLA.bit.CKEN1 = 0;
    }
  }

  inline void enableSerializer(int index) {
    i2s.CTRLA.bit.RXEN = 1;
    i2s.CTRLA.bit.TXEN = 1;
  }

  inline void disableSerializer(int index) {
    i2s.CTRLA.bit.RXEN = 0;
    i2s.CTRLA.bit.TXEN = 0;
  }

  inline int dmaTriggerSource(int index) {
    /*
    if (i2s.SERCTRL[index].bit.SERMODE == I2S_SERCTRL_SERMODE_TX_Val) {
      return (index == 0) ? I2S_DMAC_ID_TX_0 : I2S_DMAC_ID_TX_1;
    } else {
      return (index == 0) ? I2S_DMAC_ID_RX_0 : I2S_DMAC_ID_RX_1;
    }
    */
  }

  inline int txReady(int index) {
    return (index == 0) ? i2s.INTFLAG.bit.TXRDY0 :i2s.INTFLAG.bit.TXRDY1;
  }

  inline void writeData(int index, int32_t value) {
    while (i2s.SYNCBUSY.bit.TXDATA);

    i2s.TXDATA.bit.DATA = value;
  }

  inline void clearTxReady(int index) {
    if (index == 0) {
     i2s.INTFLAG.bit.TXRDY0 = 1;
    } else {
     i2s.INTFLAG.bit.TXRDY1 = 1;
    }
  }

  inline int rxReady(int index) {
    return (index == 0) ? i2s.INTFLAG.bit.RXRDY0 :i2s.INTFLAG.bit.RXRDY1;
  }

  inline int32_t readData(int index) {
    while(i2s.SYNCBUSY.bit.RXDATA)
    return i2s.RXDATA.bit.DATA;
  }

  inline void clearRxReady(int index) {
    if (index == 0) {
     i2s.INTFLAG.bit.RXRDY0 = 1;
    } else {
     i2s.INTFLAG.bit.RXRDY1 = 1;
    }
  }

  inline void* data(int index) {
    return (void*)&i2s.RXDATA.reg;
  }

private:
  volatile I2s &i2s;
};
