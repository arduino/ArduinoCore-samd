/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <Arduino.h>

#define SPI_MODE0 0x02
#define SPI_MODE1 0x00
#define SPI_MODE2 0x03
#define SPI_MODE3 0x01

#if defined(__SAMD21G18A__)
  // Even if not specified on the datasheet, the SAMD21G18A MCU
  // doesn't operate correctly with clock dividers lower than 4.
  // This allows a theoretical maximum SPI clock speed of 12Mhz
  #define SPI_MIN_CLOCK_DIVIDER 4
  // Other SAMD21xxxxx MCU may be affected as well
#else
  #define SPI_MIN_CLOCK_DIVIDER 2
#endif

class SPISettings {
  public:
	SPISettings(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) {
		if (__builtin_constant_p(clock)) {
			init_AlwaysInline(clock, bitOrder, dataMode);
		} else {
			init_MightInline(clock, bitOrder, dataMode);
		}
	}
	SPISettings() { init_AlwaysInline(4000000, MSBFIRST, SPI_MODE0); }
  private:
	void init_MightInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) {
		init_AlwaysInline(clock, bitOrder, dataMode);
	}
	void init_AlwaysInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) __attribute__((__always_inline__)) {
		uint8_t div;
		if (clock < (F_CPU / 255)) {
			div = 255;
		} else if (clock >= (F_CPU / SPI_MIN_CLOCK_DIVIDER)) {
			div = SPI_MIN_CLOCK_DIVIDER;
		} else {
			div = (F_CPU / (clock + 1)) + 1;
		}
		this->clockDiv = div;
		this->dataMode = dataMode;
		this->bitOrder = bitOrder;
	}
	uint8_t clockDiv, dataMode;
	BitOrder bitOrder;
	friend class SPIClass;

};

class SPIClass {
  public:
	SPIClass(SERCOM *p_sercom, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI);

	byte transfer(uint8_t data);
	inline void transfer(void *buf, size_t count);

	// Transaction Functions
	void usingInterrupt(uint8_t interruptNumber);
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
	SERCOM *_p_sercom;
	uint8_t _uc_pinMiso;
	uint8_t _uc_pinMosi;
	uint8_t _uc_pinSCK;
};

void SPIClass::transfer(void *buf, size_t count)
{
	// TODO: Optimize for faster block-transfer
	uint8_t *buffer = reinterpret_cast<uint8_t *>(buf);
	for (size_t i=0; i<count; i++)
		buffer[i] = transfer(buffer[i]);
}

#if SPI_INTERFACES_COUNT > 0
  extern SPIClass SPI;
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
