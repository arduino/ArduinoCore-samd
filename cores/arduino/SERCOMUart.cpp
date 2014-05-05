#include "SERCOMUart.h"


SERCOMUart::SERCOMUart(SERCOM *sercom)
{
	this->sercom = sercom;
}

void SERCOMUart::begin(uint16_t baudrate)
{
	begin(baudrate, SERIAL_8N1);
}

void SERCOMUart::begin(uint16_t baudrate, uint8_t config)
{
	sercom->init(UART_INT_CLOCK, SAMPLE_RATE_x16, baudrate);
	sercom->initFrame(extractCharSize(config), LSB_FIRST, extractParity(config), extractNbStopBit(config));
	sercom->initPads(PAD_0, PAD_2);
	
	sercom->enableUART();
}

void SERCOMUart::end()
{
	sercom->resetUART();
	rxBuffer.clear();
}

void SERCOMUart::flush()
{
	sercom->flushUART();
}

void SERCOMUart::IrqHandler()
{
	if(sercom->availableDataUART())
	{
		rxBuffer.store_char(sercom->readDataUART())
	}
	
	if(sercom->isDataRegisterEmptyUART())
	{
		writeDataUART(txBuffer.read_char());
	}
	
	if(	sercom->isBufferOverflowErrorUART() ||
		sercom->isFrameErrorUART() ||
		sercom->isParityErrorUART())
	{
		sercom->clearStatusUART();
	}
}

int SERCOMUart::available()
{
	return rxBuffer.available();
}

int SERCOMUart::peek()
{
	return rxBuffer.peek();
}

int SERCOMUart::read()
{
	return rxBuffer.read_char();
}

size_t SERCOMUart::write(uint8_t data)
{
	if(txBuffer.isFull())
		return 0;
		
	txBuffer.store_char(data);
	return 1;
}

SercomNumberStopBit extractNbStopBit(uint8_t config)
{
	switch(config & STOP_BITE_MASK)
	{
		case STOP_BITE_1:	
		default:
			return STOP_BIT_1;

		case STOP_BITE_2:	
			return STOP_BITS_2
	}
}

SercomCharSize extractCharSize(uint8_t config)
{
	switch(config & DATA_MASK)
	{
		case DATA_5:
			return 5_BITS;

		case DATA_6:
			return 6_BITS;

		case DATA_7:
			return 7_BITS;

		case DATA_8:
		default:
			return 8_BITS;

	}
}

SercomParityMode extractParity(uint8_t config)
{
	switch(config & PARITY_MASK)
	{
		case PARITY_NONE:
		default:
			return NO_PARITY;

		case PARITY_EVEN:
			return EVEN_PARITY;

		case PARITY_ODD:
			return ODD_PARITY;
	}
}

