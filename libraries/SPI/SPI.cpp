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


SPIClass::SPIClass(SERCOM *s)
{
	sercom = s;
	
	pinPeripheral(18, g_APinDescription[18].ulPinType);
	pinPeripheral(20, g_APinDescription[20].ulPinType);
	pinPeripheral(21, g_APinDescription[21].ulPinType);
}

void SPIClass::begin() {
	// Default speed set to 4Mhz, SPI mode set to MODE 0 and Bit order set to MSB first.
	sercom->initSPI(SPI_PAD_2_SCK_3, SERCOM_RX_PAD_0, SPI_CHAR_SIZE_8_BITS, MSB_FIRST);
	sercom->initSPIClock(SERCOM_SPI_MODE_0, 4000000);
	
	sercom->enableSPI();
}

void SPIClass::end() {
	sercom->resetSPI();
}

void SPIClass::setBitOrder(BitOrder order)
{
	if(order == LSBFIRST)
		sercom->setDataOrderSPI(LSB_FIRST);
	else
		sercom->setDataOrderSPI(MSB_FIRST);
}

void SPIClass::setDataMode(uint8_t mode)
{
	switch(mode)
	{
		case SPI_MODE0:
			sercom->setClockModeSPI(SERCOM_SPI_MODE_0);
			break;
			
		case SPI_MODE1:
			sercom->setClockModeSPI(SERCOM_SPI_MODE_1);
			break;
			
		case SPI_MODE2:
			sercom->setClockModeSPI(SERCOM_SPI_MODE_2);
			break;
			
		case SPI_MODE3:
			sercom->setClockModeSPI(SERCOM_SPI_MODE_3);
			break;
		
		default:
			break;
	}
}

void SPIClass::setClockDivider(uint8_t div)
{
	sercom->setBaudrateSPI(div);
}

byte SPIClass::transfer(uint8_t data)
{
	//Can writing new data?
	while(!sercom->isDataRegisterEmptySPI());
	
	//Writing the data
	sercom->writeDataSPI(data);
	
	//Data sent? new data to read?
	while(!sercom->isTransmitCompleteSPI() || !sercom->isReceiveCompleteSPI());
	
	//Read data
	return sercom->readDataSPI();
}

void SPIClass::attachInterrupt() {
	// Should be enableInterrupt()
}

void SPIClass::detachInterrupt() {
	// Should be disableInterrupt()
}

SPIClass SPI(&sercom4);
