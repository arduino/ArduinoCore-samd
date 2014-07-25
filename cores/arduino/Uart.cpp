/*
  Copyright (c) 2014 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Uart.h"
#include "WVariant.h"
#include "wiring_digital.h"

Uart::Uart(SERCOM *_s, uint8_t _pinRX, uint8_t _pinTX)
{
	sercom = _s;
	uc_pinRX = _pinRX;
	uc_pinTX = _pinTX;
}

void Uart::begin(unsigned long baudrate)
{
	begin(baudrate, (uint8_t)SERIAL_8N1);
}

void Uart::begin(unsigned long baudrate, uint8_t config)
{
  pinPeripheral(uc_pinRX, g_APinDescription[uc_pinRX].ulPinType);
  pinPeripheral(uc_pinTX, g_APinDescription[uc_pinTX].ulPinType);

	sercom->initUART(UART_INT_CLOCK, SAMPLE_RATE_x16, baudrate);
	sercom->initFrame(extractCharSize(config), LSB_FIRST, extractParity(config), extractNbStopBit(config));
	sercom->initPads(UART_TX_PAD_2, SERCOM_RX_PAD_3);


	sercom->enableUART();
}

void Uart::end()
{
	sercom->resetUART();
	rxBuffer.clear();
}

void Uart::flush()
{
	sercom->flushUART();
}

void Uart::IrqHandler()
{
	if(sercom->availableDataUART())
	{
		rxBuffer.store_char(sercom->readDataUART());
	}

	if(	sercom->isBufferOverflowErrorUART() ||
		sercom->isFrameErrorUART() ||
		sercom->isParityErrorUART())
	{
		sercom->clearStatusUART();
	}
}

int Uart::available()
{
	return rxBuffer.available();
}

int Uart::peek()
{
	return rxBuffer.peek();
}

int Uart::read()
{
	return rxBuffer.read_char();
}

size_t Uart::write(const uint8_t data)
{
	sercom->writeDataUART(data);
	return 1;
}

size_t Uart::write(const char * data)
{
	size_t writed = 0;

	while(*data != '\0')
	{
		writed += write(*data);
		++data;
	}

	return writed;
}

SercomNumberStopBit Uart::extractNbStopBit(uint8_t config)
{
	switch(config & HARDSER_STOP_BIT_MASK)
	{
		case HARDSER_STOP_BIT_1:
		default:
			return SERCOM_STOP_BIT_1;

		case HARDSER_STOP_BIT_2:
			return SERCOM_STOP_BITS_2;
	}
}

SercomUartCharSize Uart::extractCharSize(uint8_t config)
{
	switch(config & HARDSER_DATA_MASK)
	{
		case HARDSER_DATA_5:
			return UART_CHAR_SIZE_5_BITS;

		case HARDSER_DATA_6:
			return UART_CHAR_SIZE_6_BITS;

		case HARDSER_DATA_7:
			return UART_CHAR_SIZE_7_BITS;

		case HARDSER_DATA_8:
		default:
			return UART_CHAR_SIZE_8_BITS;

	}
}

SercomParityMode Uart::extractParity(uint8_t config)
{
	switch(config & HARDSER_PARITY_MASK)
	{
		case HARDSER_PARITY_NONE:
		default:
			return SERCOM_NO_PARITY;

		case HARDSER_PARITY_EVEN:
			return SERCOM_EVEN_PARITY;

		case HARDSER_PARITY_ODD:
			return SERCOM_ODD_PARITY;
	}
}

void SERCOM0_Handler()
{
	Serial.IrqHandler();
}
void SERCOM5_Handler()
{
	Serial5.IrqHandler();
}
