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

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <Arduino.h>
#include <WVariant.h>
#include "sam.h"

// SPI_HAS_TRANSACTION means SPI has
//   - beginTransaction()
//   - endTransaction()
//   - usingInterrupt()
//   - SPISetting(clock, bitOrder, dataMode)
#define SPI_HAS_TRANSACTION 1

// SPI_HAS_NOTUSINGINTERRUPT means that SPI has notUsingInterrupt() method
#define SPI_HAS_NOTUSINGINTERRUPT 1

#define SPI_MODE0 0x02
#define SPI_MODE1 0x00
#define SPI_MODE2 0x03
#define SPI_MODE3 0x01

// Recent D21 datasheets specify a typical SPI SCK period (tSCK) of 84 ns,
// see "Table 37-59. SPI Timing Characteristics and Requirements",
// which translates into a maximum SPI clock of 11.9 MHz.
// The divider is set for a 12 MHz maximum SPI clock.
// Note that while SCK and MOSI may be operable at a higher clock speed,
// MISO should not operate above 12MHz.
// The SAMD51 SERCOM runs at 96MHz when the cpu runs at 120MHz
#if (SAMD51 || SAMD21 || SAMD11 || SAML21)
  #define SPI_MAX_FREQUENCY     12000000ul
#elif (SAMC21)
  #define SPI_MAX_FREQUENCY     6000000ul
#else
  #error "SPI.h: Unsupported chip"
#endif

#define SPI_MIN_CLOCK_DIVIDER (uint8_t)(1 + ((F_CPU - 1) / SPI_MAX_FREQUENCY))

class SPISettings {
  public:
  SPISettings(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) {
    if (__builtin_constant_p(clock)) {
      init_AlwaysInline(clock, bitOrder, dataMode);
    } else {
      init_MightInline(clock, bitOrder, dataMode);
    }
  }

  // Default speed set to 4MHz, SPI mode set to MODE 0 and Bit order set to MSB first.
  SPISettings() { init_AlwaysInline(4000000, MSBFIRST, SPI_MODE0); }

  private:
  void init_MightInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) {
    init_AlwaysInline(clock, bitOrder, dataMode);
  }

  void init_AlwaysInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) __attribute__((__always_inline__)) {
    // The SAMD51 SERCOM runs at 96MHz when the cpu runs at 120MHz
    #if F_CPU == 120000000
      this->clockFreq = (clock >= (F_CPU / SPI_MIN_CLOCK_DIVIDER) ? 96000000ul / SPI_MIN_CLOCK_DIVIDER : clock);
    #else
      this->clockFreq = (clock >= (F_CPU / SPI_MIN_CLOCK_DIVIDER) ? F_CPU / SPI_MIN_CLOCK_DIVIDER : clock);
    #endif

    this->bitOrder = (bitOrder == MSBFIRST ? MSB_FIRST : LSB_FIRST);

    switch (dataMode)
    {
      case SPI_MODE0:
        this->dataMode = SERCOM_SPI_MODE_0; break;
      case SPI_MODE1:
        this->dataMode = SERCOM_SPI_MODE_1; break;
      case SPI_MODE2:
        this->dataMode = SERCOM_SPI_MODE_2; break;
      case SPI_MODE3:
        this->dataMode = SERCOM_SPI_MODE_3; break;
      default:
        this->dataMode = SERCOM_SPI_MODE_0; break;
    }
  }

  uint32_t clockFreq;
  SercomSpiClockMode dataMode;
  SercomDataOrder bitOrder;

  friend class SPIClass;
};

class SPIClass {
  public:
  SPIClass(SERCOM *p_sercom, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI, SercomSpiTXPad, SercomRXPad);


  byte transfer(uint8_t data);
  uint16_t transfer16(uint16_t data);
  void transfer(void *buf, size_t count);

  // Transaction Functions
  void usingInterrupt(int interruptNumber);
  void notUsingInterrupt(int interruptNumber);
  void beginTransaction(SPISettings settings);
  void endTransaction(void);

  // SPI Configuration methods
  void attachInterrupt();
  void detachInterrupt();

  void begin();
  void end();

  void setBitOrder(BitOrder order);
  void setDataMode(uint8_t uc_mode);
  void setClockDivider(uint8_t uc_div);

  private:
  void init();
  void config(SPISettings settings);

  SERCOM *_p_sercom;
  uint8_t _uc_pinMiso;
  uint8_t _uc_pinMosi;
  uint8_t _uc_pinSCK;

  SercomSpiTXPad _padTx;
  SercomRXPad _padRx;

  bool initialized;
  uint8_t interruptMode;
  char interruptSave;
  uint32_t interruptMask;
};

#if SPI_INTERFACES_COUNT > 0
  extern SPIClass SPI;
#endif
#if SPI_INTERFACES_COUNT > 1
  extern SPIClass SPI1;
#endif
#if SPI_INTERFACES_COUNT > 2
  extern SPIClass SPI2;
#endif
#if SPI_INTERFACES_COUNT > 3
  extern SPIClass SPI3;
#endif
#if SPI_INTERFACES_COUNT > 4
  extern SPIClass SPI4;
#endif
#if SPI_INTERFACES_COUNT > 5
  extern SPIClass SPI5;
#endif
#if SPI_INTERFACES_COUNT > 6
  extern SPIClass SPI6;
#endif
#if SPI_INTERFACES_COUNT > 7
  extern SPIClass SPI7;
#endif

// For compatibility with sketches designed for AVR @ 16 MHz
// New programs should use SPI.beginTransaction to set the SPI clock
#if F_CPU == 48000000
  #define SPI_CLOCK_DIV2   6
  #define SPI_CLOCK_DIV4   12
  #define SPI_CLOCK_DIV8   24
  #define SPI_CLOCK_DIV16  48
  #define SPI_CLOCK_DIV32  96
  #define SPI_CLOCK_DIV64  192
  #define SPI_CLOCK_DIV128 255
// The SAMD51 SERCOM runs at 96MHz when the cpu runs at 120MHz
#elif F_CPU == 120000000
  #define SPI_CLOCK_DIV2   12
  #define SPI_CLOCK_DIV4   24
  #define SPI_CLOCK_DIV8   48
  #define SPI_CLOCK_DIV16  96
  #define SPI_CLOCK_DIV32  192
  #define SPI_CLOCK_DIV64  384
  #define SPI_CLOCK_DIV128 768
#endif

#endif
