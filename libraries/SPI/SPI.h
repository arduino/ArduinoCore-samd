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
#include <api/HardwareSPI.h>

// SPI_HAS_TRANSACTION means SPI has
//   - beginTransaction()
//   - endTransaction()
//   - usingInterrupt()
//   - SPISetting(clock, bitOrder, dataMode)
// #define SPI_HAS_TRANSACTION 1
// Every core implementing arduino API has SPI transaction

// SPI_HAS_NOTUSINGINTERRUPT means that SPI has notUsingInterrupt() method
#define SPI_HAS_NOTUSINGINTERRUPT 1

#if defined(ARDUINO_ARCH_SAMD)
  // The datasheet specifies a typical SPI SCK period (tSCK) of 42 ns,
  // see "Table 36-48. SPI Timing Characteristics and Requirements",
  // which translates into a maximum SPI clock of 23.8 MHz.
  // Conservatively, the divider is set for a 12 MHz maximum SPI clock.
  #define SPI_MIN_CLOCK_DIVIDER (uint8_t)(1 + ((F_CPU - 1) / 12000000))
#endif

class SPIClassSAMD : public arduino::HardwareSPI {
  public:
  SPIClassSAMD(SERCOM *p_sercom, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI, SercomSpiTXPad, SercomRXPad);

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

  SPISettings settings;

  bool initialized;
  uint8_t interruptMode;
  char interruptSave;
  uint32_t interruptMask;
};

//#define SPIClass SPIClassSAMD

#if SPI_INTERFACES_COUNT > 0
  extern SPIClassSAMD SPI;
#endif
#if SPI_INTERFACES_COUNT > 1
  extern SPIClassSAMD SPI1;
#endif
#if SPI_INTERFACES_COUNT > 2
  extern SPIClassSAMD SPI2;
#endif
#if SPI_INTERFACES_COUNT > 3
  extern SPIClassSAMD SPI3;
#endif
#if SPI_INTERFACES_COUNT > 4
  extern SPIClassSAMD SPI4;
#endif
#if SPI_INTERFACES_COUNT > 5
  extern SPIClassSAMD SPI5;
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
#endif

#endif
