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
#include "SERCOM.h"

#include <stdio.h>

class SPIClass {
  public:
	SPIClass(SERCOM *sercom);

	byte transfer(uint8_t _data);

	// SPI Configuration methods
	void attachInterrupt();
	void detachInterrupt();

	void begin();
	void end();
	
	void setBitOrder(BitOrder order);
	void setDataMode(uint8_t mode);
	void setClockDivider(uint8_t div);

  private:
	SERCOM *sercom;
};

#if SPI_INTERFACES_COUNT > 0
extern SPIClass SPI;
#endif

#endif
