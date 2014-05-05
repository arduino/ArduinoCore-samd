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
	sercom->initUART(UART_INT_CLOCK, SAMPLE_RATE_x16, baudrate);
	sercom->initFrame(extractCharSize(config), LSB_FIRST, extractParity(config), extractNbStopBit(config));
	sercom->initPads(UART_TX_PAD_0, SERCOM_RX_PAD_2);
	
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
		rxBuffer.store_char(sercom->readDataUART());
	}
	
	if(sercom->isDataRegisterEmptyUART())
	{
		sercom->writeDataUART(txBuffer.read_char());
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

SercomNumberStopBit SERCOMUart::extractNbStopBit(uint8_t config)
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

SercomUartCharSize SERCOMUart::extractCharSize(uint8_t config)
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

SercomParityMode SERCOMUart::extractParity(uint8_t config)
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

