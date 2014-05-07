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

SPIClass::SPIClass(SERCOM *s)
{
	sercom = s;
}

void SPIClass::begin() {
	// Default speed set to 4Mhz, SPI mode set to MODE 0 and Bit order set to MSB first.
	sercom->initSPI(SPI_PAD_0_SCK_1, SERCOM_RX_PAD_2, SPI_CHAR_SIZE_8_BITS, MSB_FIRST);
	sercom->initClock(SERCOM_SPI_MODE_0, 4000000);
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

#if SPI_INTERFACES_COUNT > 0
static void SPI_0_Init(void) {
	PIO_Configure(
			g_APinDescription[PIN_SPI_MOSI].pPort,
			g_APinDescription[PIN_SPI_MOSI].ulPinType,
			g_APinDescription[PIN_SPI_MOSI].ulPin,
			g_APinDescription[PIN_SPI_MOSI].ulPinConfiguration);
	PIO_Configure(
			g_APinDescription[PIN_SPI_MISO].pPort,
			g_APinDescription[PIN_SPI_MISO].ulPinType,
			g_APinDescription[PIN_SPI_MISO].ulPin,
			g_APinDescription[PIN_SPI_MISO].ulPinConfiguration);
	PIO_Configure(
			g_APinDescription[PIN_SPI_SCK].pPort,
			g_APinDescription[PIN_SPI_SCK].ulPinType,
			g_APinDescription[PIN_SPI_SCK].ulPin,
			g_APinDescription[PIN_SPI_SCK].ulPinConfiguration);
}

SPIClass SPI(SPI_INTERFACE, SPI_INTERFACE_ID, SPI_0_Init);
#endif
