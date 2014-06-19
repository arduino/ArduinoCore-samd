/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "SPI.h"
#include "wiring_digital.h"
#include "assert.h"
#include "variant.h"

SPIClass::SPIClass(SERCOM *p_sercom, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI)
{
	assert(p_sercom != NULL );
	_p_sercom = p_sercom;

  _uc_pinMiso = uc_pinMISO;
  _uc_pinSCK = uc_pinSCK;
  _uc_pinMosi = uc_pinMOSI;
}

void SPIClass::begin()
{
  // PIO init
  pinPeripheral(_uc_pinMiso, g_APinDescription[_uc_pinMiso].ulPinType);
  pinPeripheral(_uc_pinSCK, g_APinDescription[_uc_pinSCK].ulPinType);
  pinPeripheral(_uc_pinMosi, g_APinDescription[_uc_pinMosi].ulPinType);

	// Default speed set to 4Mhz, SPI mode set to MODE 0 and Bit order set to MSB first.
	_p_sercom->initSPI(SPI_PAD_2_SCK_3, SERCOM_RX_PAD_0, SPI_CHAR_SIZE_8_BITS, MSB_FIRST);
	_p_sercom->initSPIClock(SERCOM_SPI_MODE_0, 4000000);
	
	_p_sercom->enableSPI();
}

void SPIClass::end()
{
	_p_sercom->resetSPI();
}

void SPIClass::setBitOrder(BitOrder order)
{
	if(order == LSBFIRST)
		_p_sercom->setDataOrderSPI(LSB_FIRST);
	else
		_p_sercom->setDataOrderSPI(MSB_FIRST);
}

void SPIClass::setDataMode(uint8_t mode)
{
	switch(mode)
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
	_p_sercom->setBaudrateSPI(div);
}

byte SPIClass::transfer(uint8_t data)
{
	//Writing the data
	_p_sercom->writeDataSPI(data);
	
	//Read data
	return _p_sercom->readDataSPI();
}

void SPIClass::attachInterrupt() {
	// Should be enableInterrupt()
}

void SPIClass::detachInterrupt() {
	// Should be disableInterrupt()
}

SPIClass SPI(&sercom4, 18, 20, 21);
