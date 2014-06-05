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

#include "variant.h"
#include "wiring_constants.h"

#define SPI_MODE0 0x02
#define SPI_MODE1 0x00
#define SPI_MODE2 0x03
#define SPI_MODE3 0x01

class SPIClass {
  public:
	SPIClass(SERCOM *p_sercom, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI);

	byte transfer(uint8_t data);

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

#if SPI_INTERFACES_COUNT > 0
  extern SPIClass SPI;
#endif

#endif
