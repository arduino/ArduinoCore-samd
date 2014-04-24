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

void SERCOMUart::end(){
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

	if(	sercom->isBufferOverflowErrorUART() ||
		sercom->isFrameErrorUART() ||
		sercom->isParityErrorUART())
	{
		sercom->clearStatusUART();
	}
}

bool SERCOMUart::available()
{
	return rxBuffer.available_char();
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
	return sercom->writeDataUART(data);
}

SercomNumberStopBit extractNbStopBit(uint8_t config)
{
	switch(config)
	{
		case SERIAL_5N1:
		case SERIAL_6N1:
		case SERIAL_7N1:
		case SERIAL_8N1:
		case SERIAL_5E1:
		case SERIAL_6E1:
		case SERIAL_7E1:
		case SERIAL_8E1:	
		case SERIAL_5O1:
		case SERIAL_6O1:
		case SERIAL_7O1:
		case SERIAL_8O1:	
		default:
			return 1_STOP_BIT;

		case SERIAL_5N2:
		case SERIAL_6N2:
		case SERIAL_7N2:
		case SERIAL_8N2:
		case SERIAL_5E2:
		case SERIAL_6E2:
		case SERIAL_7E2:
		case SERIAL_8E2:	
		case SERIAL_5O2:
		case SERIAL_6O2:
		case SERIAL_7O2:
		case SERIAL_8O2:	
			return 2_STOP_BITS;
	}
}

SercomCharSize extractCharSize(uint8_t config)
{
	switch(config)
	{
		case SERIAL_5N1:
		case SERIAL_5N2:
		case SERIAL_5E1:
		case SERIAL_5E2:
		case SERIAL_5O1:
		case SERIAL_5O2:
			return 5_BITS;

		case SERIAL_6N1:
		case SERIAL_6N2:
		case SERIAL_6E1:
		case SERIAL_6E2:
		case SERIAL_6O1:
		case SERIAL_6O2:
			return 6_BITS;

		case SERIAL_7N1:
		case SERIAL_7N2:
		case SERIAL_7E1:
		case SERIAL_7E2:
		case SERIAL_7O1:
		case SERIAL_7O2:
			return 7_BITS;

		case SERIAL_8N1:
		case SERIAL_8N2:
		case SERIAL_8E1:
		case SERIAL_8E2:
		case SERIAL_8O1:
		case SERIAL_8O2:
		default:
			return 8_BITS;

	}
}

SercomParityMode extractParity(uint8_t config)
{
	switch(config)
	{
		case SERIAL_5N1:
		case SERIAL_6N1:
		case SERIAL_7N1:
		case SERIAL_8N1:
		case SERIAL_5N2:
		case SERIAL_6N2:
		case SERIAL_7N2:
		case SERIAL_8N2:
		default:
			return NO_PARITY;

		case SERIAL_5E1:
		case SERIAL_6E1:
		case SERIAL_7E1:
		case SERIAL_8E1:
		case SERIAL_5E2:
		case SERIAL_6E2:
		case SERIAL_7E2:
		case SERIAL_8E2:
			return EVEN_PARITY;

		case SERIAL_5O1:
		case SERIAL_6O1:
		case SERIAL_7O1:
		case SERIAL_8O1:
		case SERIAL_5O2:
		case SERIAL_6O2:
		case SERIAL_7O2:
		case SERIAL_8O2:
			return ODD_PARITY;
	}
}

