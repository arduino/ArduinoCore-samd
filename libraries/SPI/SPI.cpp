/*
 * SPI Master library for Arduino Zero.
 * Copyright (c) 2015 Arduino LLC
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "SPI.h"
#include <Arduino.h>
#include <wiring_private.h>
#include <assert.h>

#ifdef USE_TINYUSB
// For Serial when selecting TinyUSB
#include <Adafruit_TinyUSB.h>
#endif

#define SPI_IMODE_NONE   0
#define SPI_IMODE_EXTINT 1
#define SPI_IMODE_GLOBAL 2

const SPISettings DEFAULT_SPI_SETTINGS = SPISettings();

SPIClass::SPIClass(SERCOM *p_sercom, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI, SercomSpiTXPad PadTx, SercomRXPad PadRx)
{
  initialized = false;
  assert(p_sercom != NULL);
  _p_sercom = p_sercom;

  // pins
  _uc_pinMiso = uc_pinMISO;
  _uc_pinSCK = uc_pinSCK;
  _uc_pinMosi = uc_pinMOSI;

  // SERCOM pads
  _padTx=PadTx;
  _padRx=PadRx;
}

void SPIClass::begin()
{
  if(!initialized) {
    interruptMode = SPI_IMODE_NONE;
    interruptSave = 0;
    interruptMask = 0;
    initialized = true;
  }

  if(!use_dma) {
    dmaAllocate();
  }

  // PIO init
  pinPeripheral(_uc_pinMiso, g_APinDescription[_uc_pinMiso].ulPinType);
  pinPeripheral(_uc_pinSCK, g_APinDescription[_uc_pinSCK].ulPinType);
  pinPeripheral(_uc_pinMosi, g_APinDescription[_uc_pinMosi].ulPinType);

  config(DEFAULT_SPI_SETTINGS);
}

void SPIClass::config(SPISettings settings)
{
  _p_sercom->disableSPI();

  _p_sercom->initSPI(_padTx, _padRx, SPI_CHAR_SIZE_8_BITS, settings.bitOrder);
  _p_sercom->initSPIClock(settings.dataMode, settings.clockFreq);

  _p_sercom->enableSPI();
}

void SPIClass::end()
{
  _p_sercom->resetSPI();
  initialized = false;
  // Add DMA deallocation here
}

#ifndef interruptsStatus
#define interruptsStatus() __interruptsStatus()
static inline unsigned char __interruptsStatus(void) __attribute__((always_inline, unused));
static inline unsigned char __interruptsStatus(void)
{
  // See http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0497a/CHDBIBGJ.html
  return (__get_PRIMASK() ? 0 : 1);
}
#endif

void SPIClass::usingInterrupt(int interruptNumber)
{
  if ((interruptNumber == NOT_AN_INTERRUPT) || (interruptNumber == EXTERNAL_INT_NMI))
    return;

  uint8_t irestore = interruptsStatus();
  noInterrupts();

  if (interruptNumber >= EXTERNAL_NUM_INTERRUPTS)
    interruptMode = SPI_IMODE_GLOBAL;
  else
  {
    interruptMode |= SPI_IMODE_EXTINT;
    interruptMask |= (1 << g_APinDescription[interruptNumber].ulExtInt);
  }

  if (irestore)
    interrupts();
}

void SPIClass::notUsingInterrupt(int interruptNumber)
{
  if ((interruptNumber == NOT_AN_INTERRUPT) || (interruptNumber == EXTERNAL_INT_NMI))
    return;

  if (interruptMode & SPI_IMODE_GLOBAL)
    return; // can't go back, as there is no reference count

  uint8_t irestore = interruptsStatus();
  noInterrupts();

  interruptMask &= ~(1 << g_APinDescription[interruptNumber].ulExtInt);

  if (interruptMask == 0)
    interruptMode = SPI_IMODE_NONE;

  if (irestore)
    interrupts();
}

void SPIClass::beginTransaction(SPISettings settings)
{
  if (interruptMode != SPI_IMODE_NONE)
  {
    if (interruptMode & SPI_IMODE_GLOBAL)
    {
      interruptSave = interruptsStatus();
      noInterrupts();
    }
    else if (interruptMode & SPI_IMODE_EXTINT)
      EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT(interruptMask);
  }

  config(settings);
}

void SPIClass::endTransaction(void)
{
  if (interruptMode != SPI_IMODE_NONE)
  {
    if (interruptMode & SPI_IMODE_GLOBAL)
    {
      if (interruptSave)
        interrupts();
    }
    else if (interruptMode & SPI_IMODE_EXTINT)
      EIC->INTENSET.reg = EIC_INTENSET_EXTINT(interruptMask);
  }
}

void SPIClass::setBitOrder(BitOrder order)
{
  if (order == LSBFIRST) {
    _p_sercom->setDataOrderSPI(LSB_FIRST);
  } else {
    _p_sercom->setDataOrderSPI(MSB_FIRST);
  }
}

void SPIClass::setDataMode(uint8_t mode)
{
  switch (mode)
  {
    case SPI_MODE0:
      _p_sercom->setClockModeSPI(SERCOM_SPI_MODE_0);
      break;

    case SPI_MODE1:
      _p_sercom->setClockModeSPI(SERCOM_SPI_MODE_1);
      break;

    case SPI_MODE2:
      _p_sercom->setClockModeSPI(SERCOM_SPI_MODE_2);
      break;

    case SPI_MODE3:
      _p_sercom->setClockModeSPI(SERCOM_SPI_MODE_3);
      break;

    default:
      break;
  }
}

void SPIClass::setClockDivider(uint8_t div)
{
  if(div < SPI_MIN_CLOCK_DIVIDER) {
    _p_sercom->setBaudrateSPI(SPI_MIN_CLOCK_DIVIDER);
  } else {
    _p_sercom->setBaudrateSPI(div);
  }
}

byte SPIClass::transfer(uint8_t data)
{
  return _p_sercom->transferDataSPI(data);
}

uint16_t SPIClass::transfer16(uint16_t data) {
  union { uint16_t val; struct { uint8_t lsb; uint8_t msb; }; } t;

  t.val = data;

  if (_p_sercom->getDataOrderSPI() == LSB_FIRST) {
    t.lsb = transfer(t.lsb);
    t.msb = transfer(t.msb);
  } else {
    t.msb = transfer(t.msb);
    t.lsb = transfer(t.lsb);
  }

  return t.val;
}

void SPIClass::transfer(void *buf, size_t count)
{
  uint8_t *buffer = reinterpret_cast<uint8_t *>(buf);
  for (size_t i=0; i<count; i++) {
    *buffer = transfer(*buffer);
    buffer++;
  }
}

// DMA-based SPI transfer() function ---------------------------------------

// IMPORTANT: references to 65535 throughout the DMA code are INTENTIONAL.
// DO NOT try to 'fix' by changing to 65536, or large transfers will fail!
// The BTCNT value of a DMA descriptor is an unsigned 16-bit value with a
// max of 65535. Larger transfers are handled by linked descriptors.

// Pointer to SPIClass object, one per DMA channel. This allows the
// DMA callback (which has to exist outside the class context) to have
// a reference back to the originating SPIClass object.
static SPIClass *spiPtr[DMAC_CH_NUM] = { 0 }; // Legit inits list to NULL

void SPIClass::dmaCallback(Adafruit_ZeroDMA *dma) {
  // dmaCallback() receives an Adafruit_ZeroDMA object. From this we can get
  // a channel number (0 to DMAC_CH_NUM-1, always unique per ZeroDMA object),
  // then locate the originating SPIClass object using array lookup, setting
  // the dma_busy element 'false' to indicate end of transfer. Doesn't matter
  // if it's a read or write transfer...both channels get pointers to it.
  spiPtr[dma->getChannel()]->dma_busy = false;
}

// For read-only and read+write transfers, a callback is assigned only
// to the read channel to indicate end-of-transfer, and the write channel's
// callback is assigned to this nonsense function (for reasons I'm not
// entirely sure of, setting the callback to NULL doesn't work).
static void dmaDoNothingCallback(Adafruit_ZeroDMA *dma) { (void)dma; }

// This could've gone in begin(), but for the sake of organization...
void SPIClass::dmaAllocate(void) {
  // In order to support fully non-blocking SPI transfers, DMA descriptor
  // lists must be created for the input and/or output data. Rather than
  // do this dynamically with every transfer, the lists are allocated once
  // on SPI init. Maximum list size is finite and knowable -- transfers to
  // or from RAM or from flash memory will never exceed the corresponding
  // memory size (if they do, you have bigger problems). Descriptors
  // aren't large and there's usually only a handful to a dozen, so this
  // isn't an excessive burden in exchange for big non-blocking transfers.
  uint32_t maxWriteBytes = FLASH_SIZE; // Writes can't exceed all of flash
#if defined(__SAMD51__)
  uint32_t maxReadBytes = HSRAM_SIZE;  // Reads can't exceed all of RAM
#else
  uint32_t maxReadBytes = HMCRAMC0_SIZE;
#endif
  if(maxReadBytes > maxWriteBytes) { // I don't think any SAMD devices
    maxWriteBytes = maxReadBytes;    // have RAM > flash, but just in case
  }

  // VITAL to alloc read channel first, assigns it a higher DMA priority!
  if(readChannel.allocate() == DMA_STATUS_OK) {
    if(writeChannel.allocate() == DMA_STATUS_OK) {

      // Both DMA channels (read and write) allocated successfully,
      // set up transfer triggers and other basics...

      // readChannel callback only needs to be set up once.
      // Unlike the write callback which may get switched on or off,
      // read callback stays put. In certain cases the read DMA job
      // just isn't started and the callback is a non-issue then.
      readChannel.setTrigger(getDMAC_ID_RX());
      readChannel.setAction(DMA_TRIGGER_ACTON_BEAT);
      readChannel.setCallback(dmaCallback);
      spiPtr[readChannel.getChannel()] = this;

      writeChannel.setTrigger(getDMAC_ID_TX());
      writeChannel.setAction(DMA_TRIGGER_ACTON_BEAT);
      spiPtr[writeChannel.getChannel()] = this;

      // One descriptor per channel has already been allocated
      // in Adafruit_ZeroDMA, this just gets pointers to them...
      firstReadDescriptor = readChannel.addDescriptor(
        (void *)getDataRegister(), // Source address (SPI data reg)
        NULL,                      // Dest address (set later)
        0,                         // Count (set later)
        DMA_BEAT_SIZE_BYTE,        // Bytes/hwords/words
        false,                     // Don't increment source address
        true);                     // Increment dest address
      firstWriteDescriptor = writeChannel.addDescriptor(
        NULL,                      // Source address (set later)
        (void *)getDataRegister(), // Dest (SPI data register)
        0,                         // Count (set later)
        DMA_BEAT_SIZE_BYTE,        // Bytes/hwords/words
        true,                      // Increment source address
        false);                    // Don't increment dest address
      // This is the number of EXTRA descriptors beyond the first.
      int numReadDescriptors  = ((maxReadBytes  + 65534) / 65535) - 1;
      int numWriteDescriptors = ((maxWriteBytes + 65534) / 65535) - 1;
      int totalDescriptors    = numReadDescriptors + numWriteDescriptors;

      if(totalDescriptors <= 0) { // Don't need extra descriptors,
        use_dma = true;           // channels are allocated, we're good.
      } else {                    // Else allocate extra descriptor lists...
        // Although DMA descriptors are technically a linked list, we just
        // allocate a chunk all at once, and finesse the pointers later.
        if((extraReadDescriptors = (DmacDescriptor *)malloc(
          totalDescriptors * sizeof(DmacDescriptor)))) {
          use_dma = true; // Everything allocated successfully
          extraWriteDescriptors = &extraReadDescriptors[numReadDescriptors];

          // Initialize descriptors (copy from first ones)
          // cast to void* to suppress warning: with no trivial copy-assignment [-Wclass-memaccess]
          for(int i=0; i<numReadDescriptors; i++) {
            memcpy((void*) &extraReadDescriptors[i], firstReadDescriptor,
              sizeof(DmacDescriptor));
          }
          for(int i=0; i<numWriteDescriptors; i++) {
            memcpy((void*) &extraWriteDescriptors[i], firstWriteDescriptor,
              sizeof(DmacDescriptor));
          }
        } // end malloc
      } // end extra descriptor check

      if(use_dma) { // If everything allocated successfully,
        return;     // then we're done here.
      }             // Otherwise clean up interim allocations...
      writeChannel.free();
    } // end writeChannel alloc
    readChannel.free();
  } // end readChannel alloc

  // NOT FATAL if channel or descriptor allocation fails.
  // transfer() function will fall back on a manual byte-by-byte loop.
}

void SPIClass::transfer(const void *txbuf, void *rxbuf, size_t count,
  bool block) {

  if((!txbuf && !rxbuf) || !count) { // Validate inputs
    return;
  }
  // OK to assume now that txbuf and/or rxbuf are non-NULL, an if/else is
  // often sufficient, don't need else-ifs for everything buffer related.

  uint8_t *txbuf8 = (uint8_t *)txbuf; // Must cast to byte size
  uint8_t *rxbuf8 = (uint8_t *)rxbuf; // for pointer math

  if(use_dma) { // DMA-BASED TRANSFER YAY ----------------------------------

    static const uint8_t dum = 0xFF; // Dummy byte for read-only xfers

    // Set up DMA descriptor lists -----------------------------------------

    DmacDescriptor *rDesc = firstReadDescriptor;
    DmacDescriptor *wDesc = firstWriteDescriptor;
    int descIdx = 0; // Index into extra descriptor lists

    while(count) { // Counts down to end of transfer
      uint32_t bytesThisDescriptor = count;
      if(bytesThisDescriptor > 65535) { // Limit each descriptor
        bytesThisDescriptor = 65535;    // to 65535 (not 65536) bytes
      }
      rDesc->BTCNT.reg = wDesc->BTCNT.reg = bytesThisDescriptor;
      if(rxbuf) { // Read-only or read+write
        // Auto-inc addresses in DMA descriptors must point to END of data.
        // Buf pointers would advance at end of loop anyway, do it now...
        rxbuf8 += bytesThisDescriptor;
        rDesc->DSTADDR.reg = (uint32_t)rxbuf8;
      }
      if(txbuf) { // Write-only or read+write
        txbuf8 += bytesThisDescriptor; // Same as above
        wDesc->SRCADDR.reg       = (uint32_t)txbuf8;
        wDesc->BTCTRL.bit.SRCINC = 1; // Increment source pointer
      } else { // Read-only requires dummy write
        wDesc->SRCADDR.reg       = (uint32_t)&dum;
        wDesc->BTCTRL.bit.SRCINC = 0; // Don't increment source pointer
      }
      count -= bytesThisDescriptor;
      if(count) { // Still more data?
        // Link to next descriptors. Extra descriptors are IN ADDITION
        // to first, so it's safe and correct that descIdx starts at 0.
        rDesc->DESCADDR.reg = (uint32_t)&extraReadDescriptors[descIdx];
        wDesc->DESCADDR.reg = (uint32_t)&extraWriteDescriptors[descIdx];
        rDesc = &extraReadDescriptors[descIdx];  // Update pointers to
        wDesc = &extraWriteDescriptors[descIdx]; // next descriptors
        descIdx++;
        // A write-only transfer doesn't use the read descriptor list, but
        // it's quicker to build it (full of nonsense) anyway than to check.
      } else { // No more data, end descriptor linked lists
        rDesc->DESCADDR.reg = wDesc->DESCADDR.reg = 0;
      }
    }

    // Set up DMA transfer job(s) ------------------------------------------

    if(rxbuf) { // Read+write or read-only
      // End-of-read callback is already set up, disable write CB, start job
      writeChannel.setCallback(dmaDoNothingCallback);
      readChannel.startJob();
    } else { // Write-only, use end-of-write callback
      writeChannel.setCallback(dmaCallback);
    }

    // Run DMA jobs, blocking if requested ---------------------------------

    dma_busy = true;
    writeChannel.startJob(); // All xfers, even read-only, need write job.
    if(block) {              // If blocking transfer requested,
      while(dma_busy);       // wait for job to finish
    }

  } else { // NON-DMA FALLBACK ---------------------------------------------

    if(txbuf8) {
      if(rxbuf8) { // Write + read simultaneously
        while(count--) {
          *rxbuf8++ = _p_sercom->transferDataSPI(*txbuf8++);
        }
      } else {     // Write only
        while(count--) {
          (void)_p_sercom->transferDataSPI(*txbuf8++);
        }
      }
    } else {       // Read only
      while(count--) {
        *rxbuf8++ = _p_sercom->transferDataSPI(0xFF);
      }
    }

  } // end non-DMA
}

// Waits for a prior in-background DMA transfer to complete.
void SPIClass::waitForTransfer(void) {
  while(dma_busy);
}

/* returns the current DMA transfer status to allow non-blocking polling */
bool SPIClass::isBusy(void) {
  return dma_busy;
}


// End DMA-based SPI transfer() code ---------------------------------------

void SPIClass::attachInterrupt() {
  // Should be enableInterrupt()
}

void SPIClass::detachInterrupt() {
  // Should be disableInterrupt()
}

// SPI DMA lookup works on both SAMD21 and SAMD51

static const struct {
  volatile uint32_t *data_reg;
  int                dmac_id_tx;
  int                dmac_id_rx;
} sercomData[] = {
  { &SERCOM0->SPI.DATA.reg, SERCOM0_DMAC_ID_TX, SERCOM0_DMAC_ID_RX },
  { &SERCOM1->SPI.DATA.reg, SERCOM1_DMAC_ID_TX, SERCOM1_DMAC_ID_RX },
  { &SERCOM2->SPI.DATA.reg, SERCOM2_DMAC_ID_TX, SERCOM2_DMAC_ID_RX },
  { &SERCOM3->SPI.DATA.reg, SERCOM3_DMAC_ID_TX, SERCOM3_DMAC_ID_RX },
#if defined(SERCOM4)
  { &SERCOM4->SPI.DATA.reg, SERCOM4_DMAC_ID_TX, SERCOM4_DMAC_ID_RX },
#endif
#if defined(SERCOM5)
  { &SERCOM5->SPI.DATA.reg, SERCOM5_DMAC_ID_TX, SERCOM5_DMAC_ID_RX },
#endif
#if defined(SERCOM6)
  { &SERCOM6->SPI.DATA.reg, SERCOM6_DMAC_ID_TX, SERCOM6_DMAC_ID_RX },
#endif
#if defined(SERCOM7)
  { &SERCOM7->SPI.DATA.reg, SERCOM7_DMAC_ID_TX, SERCOM7_DMAC_ID_RX },
#endif
};

volatile uint32_t *SPIClass::getDataRegister(void) {
  int8_t idx = _p_sercom->getSercomIndex();
  return (idx >= 0) ? sercomData[idx].data_reg: NULL;
}

int SPIClass::getDMAC_ID_TX(void) {
  int8_t idx = _p_sercom->getSercomIndex();
  return (idx >= 0) ? sercomData[idx].dmac_id_tx : -1;
}

int SPIClass::getDMAC_ID_RX(void) {
  int8_t idx = _p_sercom->getSercomIndex();
  return (idx >= 0) ? sercomData[idx].dmac_id_rx : -1;
}

#if defined(__SAMD51__)

// Set the SPI device's SERCOM clock CORE and SLOW clock sources.
// SercomClockSource values are an enumeration in SERCOM.h.
// This works on SAMD51 only.  On SAMD21, a dummy function is declared
// in SPI.h which compiles to nothing, so user code doesn't need to check
// and conditionally compile lines for different architectures.
void SPIClass::setClockSource(SercomClockSource clk) {
  int8_t idx = _p_sercom->getSercomIndex();
  _p_sercom->setClockSource(idx, clk, true);  // true  = set core clock
  _p_sercom->setClockSource(idx, clk, false); // false = set slow clock
}

#endif // end __SAMD51__

#if SPI_INTERFACES_COUNT > 0
  /* In case new variant doesn't define these macros,
   * we put here the ones for Arduino Zero.
   *
   * These values should be different on some variants!
   *
   * The SPI PAD values can be found in cores/arduino/SERCOM.h:
   *   - SercomSpiTXPad
   *   - SercomRXPad
   */
  #ifndef PERIPH_SPI
    #define PERIPH_SPI           sercom4
    #define PAD_SPI_TX           SPI_PAD_2_SCK_3
    #define PAD_SPI_RX           SERCOM_RX_PAD_0
  #endif // PERIPH_SPI
  SPIClass SPI (&PERIPH_SPI,  PIN_SPI_MISO,  PIN_SPI_SCK,  PIN_SPI_MOSI,  PAD_SPI_TX,  PAD_SPI_RX);
#endif
#if SPI_INTERFACES_COUNT > 1
  SPIClass SPI1(&PERIPH_SPI1, PIN_SPI1_MISO, PIN_SPI1_SCK, PIN_SPI1_MOSI, PAD_SPI1_TX, PAD_SPI1_RX);
#endif
#if SPI_INTERFACES_COUNT > 2
  SPIClass SPI2(&PERIPH_SPI2, PIN_SPI2_MISO, PIN_SPI2_SCK, PIN_SPI2_MOSI, PAD_SPI2_TX, PAD_SPI2_RX);
#endif
#if SPI_INTERFACES_COUNT > 3
  SPIClass SPI3(&PERIPH_SPI3, PIN_SPI3_MISO, PIN_SPI3_SCK, PIN_SPI3_MOSI, PAD_SPI3_TX, PAD_SPI3_RX);
#endif
#if SPI_INTERFACES_COUNT > 4
  SPIClass SPI4(&PERIPH_SPI4, PIN_SPI4_MISO, PIN_SPI4_SCK, PIN_SPI4_MOSI, PAD_SPI4_TX, PAD_SPI4_RX);
#endif
#if SPI_INTERFACES_COUNT > 5
  SPIClass SPI5(&PERIPH_SPI5, PIN_SPI5_MISO, PIN_SPI5_SCK, PIN_SPI5_MOSI, PAD_SPI5_TX, PAD_SPI5_RX);
#endif
