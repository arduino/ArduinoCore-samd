/*
 * TwoWire.h - TWI/I2C library for Arduino Due
 * Copyright (c) 2011 Cristian Maglie <c.maglie@bug.st>.
 * All rights reserved.
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

extern "C" {
#include <string.h>
}

#include "Wire.h"

TwoWire::TwoWire(SERCOM * sercom)
{
	this->sercom = sercom;
	transmissionBegun = false;
}

void TwoWire::begin(void) {
	//Master Mode
	sercom->initMasterWIRE(TWI_CLOCK);
	sercom->enableWIRE();
}

void TwoWire::begin(uint8_t address) {
	//Slave mode
	sercom->initSlaveWIRE(address);
	sercom->enableWIRE();
}

size_t SERCOM::resquestFrom(uint8_t address, size_t quantity, bool stopBit)
{
	//Quantity > 0 AND startTransmission worked ?
	if(quantity == 0 || !sercom->startTransmissionWIRE(address, READ_FLAG))
		return 0;
		
	for(size_t readed = 0; read < quantity; ++readed)
	{
		//Prepare stop bit ? user want stop bit ?
		if(quantity - read == 1 && stopBit)
			sercom->prepareStopBitWIRE();
			
		rxBuffer.store_char(sercom->readDataWIRE());
	}
	
	return quantity;
}

size_t SERCOM::resquestFrom(uint8_t address, size_t quantity)
{
	return requestFrom(address, quantity, true);
}

void TwoWire::beginTransmission(uint8_t address) {
	// save address of target and clear buffer
	txAddress = address;
	txBuffer.clear();
	
	transmissionBegun = true;
}

/*
void TwoWire::beginTransmission(int address) {
	beginTransmission((uint8_t) address);
}

//
//	Originally, 'endTransmission' was an f(void) function.
//	It has been modified to take one parameter indicating
//	whether or not a STOP should be performed on the bus.
//	Calling endTransmission(false) allows a sketch to
//	perform a repeated start.
//
//	WARNING: Nothing in the library keeps track of whether
//	the bus tenure has been properly ended with a STOP. It
//	is very possible to leave the bus in a hung state if
//	no call to endTransmission(true) is made. Some I2C
//	devices will behave oddly if they do not see a STOP.
//
uint8_t TwoWire::endTransmission(uint8_t sendStop)
{
	// transmit buffer (blocking)
	TWI_StartWrite(twi, txAddress, 0, 0, txBuffer[0]);
	TWI_WaitByteSent(twi, XMIT_TIMEOUT);
	int sent = 1;
	while (sent < txBufferLength) {
		TWI_WriteByte(twi, txBuffer[sent++]);
		TWI_WaitByteSent(twi, XMIT_TIMEOUT);
	}
	TWI_Stop( twi);
	TWI_WaitTransferComplete(twi, XMIT_TIMEOUT);

	// empty buffer
	txBufferLength = 0;

	status = MASTER_IDLE;
	return sent;
}
*/

// Errors:
//	0 : Success
//	1 : Data too long
//	2 : NACK on transmit of address
//	3 : NACK on transmit of data
//	4 : Other error
uint8_t TwoWire::endTransmission(bool stopBit)
{
	transmissionBegun = false;
	
	//Check if there are data to send
	if(!txBuffer.available())
		return 4;
		
	//Start I2C transmission
	if(!sercom->startTransmissionWIRE(txAddress, WRITE_FLAG))
		return 2;	//Address error
	
	//Send all buffer
	while(txBuffer.available())
	{
		//If is the last data, send STOP bit after it.
		if(txBuffer.available() == 1)
			sercom->prepareStopBitWIRE();
		
		//Trying to send data
		if(!sercom->sendDataMasterWIRE(txBuffer.read_char()))
			return 3;	//Nack or error
	}
	
	return 0;
}

uint8_t TwoWire::endTransmission()
{
	return endTransmission(true);
}

size_t TwoWire::write(uint8_t data) 
{
	if(sercom->isMasterWIRE())
	{
		//No writing, without begun transmission or a full buffer
		if(!transmissionBegun || txBuffer.isFull())
			return 0;
			
		txBuffer.store_char(data);
		return 1;
	}
	else
	{
		sercom->sendDataSlaveWIRE(data);
	}
}

size_t TwoWire::write(const uint8_t *data, size_t quantity)
{
	//Try to store all data
	for(size_t i = 0; i < quantity; ++i)
	{
		//Return the number of data stored, when the buffer is full (if write return 0)
		if(!write(data[i])
			return i;
	}
	
	//All data stored
	return quantity;
}

int TwoWire::available(void)
{
	return rxBuffer.available();
}

int TwoWire::read(void)
{
	return rxBuffer.read_char();
}

int TwoWire::peek(void)
{
	return rxBuffer.peek();
}

void TwoWire::flush(void)
{
	// Do nothing, use endTransmission(..) to force
	// data transfer.
}

void TwoWire::onReceive(void(*function)(int))
{
	onReceiveCallback = function;
}

void TwoWire::onRequest(void(*function)(void))
{
	onRequestCallback = function;
}


void TwoWire::onService(void)
{
	if(sercom->isSlaveWIRE())
	{
		//Received data
		if(sercom->isDataReadyWIRE())
		{
			//Store data
			rxBuffer.store_char(sercom->readDataWIRE());
			
			//Stop or Restart detected
			if(sercom->isStopDetectedWIRE() || sercom->isRestartDetectedWIRE())
			{
				//Calling onReceiveCallback, if exists
				if(onReceiveCallback)
				{
					onReceiveCallback(available());
				}
			}
		}
		
		//Address Match
		if(sercom->isAddressMatch())
		{
			//Is a request ?
			if(sercom->isMasterReadOperationWIRE())
			{
				//Calling onRequestCallback, if exists
				if(onRequestCallback)
				{
					onRequestCallback();
				}
			}
		}
	}
}

/*
void TwoWire::onService(void)
{
	// Retrieve interrupt status
	uint32_t sr = TWI_GetStatus(twi);

	if (status == SLAVE_IDLE && TWI_STATUS_SVACC(sr)) {
		TWI_DisableIt(twi, TWI_IDR_SVACC);
		TWI_EnableIt(twi, TWI_IER_RXRDY | TWI_IER_GACC | TWI_IER_NACK
				| TWI_IER_EOSACC | TWI_IER_SCL_WS | TWI_IER_TXCOMP);

		srvBufferLength = 0;
		srvBufferIndex = 0;

		// Detect if we should go into RECV or SEND status
		// SVREAD==1 means *master* reading -> SLAVE_SEND
		if (!TWI_STATUS_SVREAD(sr)) {
			status = SLAVE_RECV;
		} else {
			status = SLAVE_SEND;

			// Alert calling program to generate a response ASAP
			if (onRequestCallback)
				onRequestCallback();
			else
				// create a default 1-byte response
				write((uint8_t) 0);
		}
	}

	if (status != SLAVE_IDLE) {
		if (TWI_STATUS_TXCOMP(sr) && TWI_STATUS_EOSACC(sr)) {
			if (status == SLAVE_RECV && onReceiveCallback) {
				// Copy data into rxBuffer
				// (allows to receive another packet while the
				// user program reads actual data)
				for (uint8_t i = 0; i < srvBufferLength; ++i)
					rxBuffer[i] = srvBuffer[i];
				rxBufferIndex = 0;
				rxBufferLength = srvBufferLength;

				// Alert calling program
				onReceiveCallback( rxBufferLength);
			}

			// Transfer completed
			TWI_EnableIt(twi, TWI_SR_SVACC);
			TWI_DisableIt(twi, TWI_IDR_RXRDY | TWI_IDR_GACC | TWI_IDR_NACK
					| TWI_IDR_EOSACC | TWI_IDR_SCL_WS | TWI_IER_TXCOMP);
			status = SLAVE_IDLE;
		}
	}

	if (status == SLAVE_RECV) {
		if (TWI_STATUS_RXRDY(sr)) {
			if (srvBufferLength < BUFFER_LENGTH)
				srvBuffer[srvBufferLength++] = TWI_ReadByte(twi);
		}
	}

	if (status == SLAVE_SEND) {
		if (TWI_STATUS_TXRDY(sr) && !TWI_STATUS_NACK(sr)) {
			uint8_t c = 'x';
			if (srvBufferIndex < srvBufferLength)
				c = srvBuffer[srvBufferIndex++];
			TWI_WriteByte(twi, c);
		}
	}
}
*/

#if WIRE_INTERFACES_COUNT > 0
static void Wire_Init(void) {
	pmc_enable_periph_clk(WIRE_INTERFACE_ID);
	PIO_Configure(
			g_APinDescription[PIN_WIRE_SDA].pPort,
			g_APinDescription[PIN_WIRE_SDA].ulPinType,
			g_APinDescription[PIN_WIRE_SDA].ulPin,
			g_APinDescription[PIN_WIRE_SDA].ulPinConfiguration);
	PIO_Configure(
			g_APinDescription[PIN_WIRE_SCL].pPort,
			g_APinDescription[PIN_WIRE_SCL].ulPinType,
			g_APinDescription[PIN_WIRE_SCL].ulPin,
			g_APinDescription[PIN_WIRE_SCL].ulPinConfiguration);

	NVIC_DisableIRQ(WIRE_ISR_ID);
	NVIC_ClearPendingIRQ(WIRE_ISR_ID);
	NVIC_SetPriority(WIRE_ISR_ID, 0);
	NVIC_EnableIRQ(WIRE_ISR_ID);
}

TwoWire Wire = TwoWire(WIRE_INTERFACE, Wire_Init);

void WIRE_ISR_HANDLER(void) {
	Wire.onService();
}
#endif

#if WIRE_INTERFACES_COUNT > 1
static void Wire1_Init(void) {
	pmc_enable_periph_clk(WIRE1_INTERFACE_ID);
	PIO_Configure(
			g_APinDescription[PIN_WIRE1_SDA].pPort,
			g_APinDescription[PIN_WIRE1_SDA].ulPinType,
			g_APinDescription[PIN_WIRE1_SDA].ulPin,
			g_APinDescription[PIN_WIRE1_SDA].ulPinConfiguration);
	PIO_Configure(
			g_APinDescription[PIN_WIRE1_SCL].pPort,
			g_APinDescription[PIN_WIRE1_SCL].ulPinType,
			g_APinDescription[PIN_WIRE1_SCL].ulPin,
			g_APinDescription[PIN_WIRE1_SCL].ulPinConfiguration);

	NVIC_DisableIRQ(WIRE1_ISR_ID);
	NVIC_ClearPendingIRQ(WIRE1_ISR_ID);
	NVIC_SetPriority(WIRE1_ISR_ID, 0);
	NVIC_EnableIRQ(WIRE1_ISR_ID);
}

TwoWire Wire1 = TwoWire(WIRE1_INTERFACE, Wire1_Init);

void WIRE1_ISR_HANDLER(void) {
	Wire1.onService();
}
#endif
