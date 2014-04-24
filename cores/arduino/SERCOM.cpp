#include "SERCOM.h"


SERCOM::SERCOM(Sercom* sercom)
{
	this->sercom = sercom;
	sercomUart = sercom->UART;
	sercomSpi = sercom->SPI;
}

void SERCOM::initUART(SercomUartMode mode, SercomUartSampleRate sampleRate, uint32_t baudrate)
{
	resetUART();
	
	sercomUart->CTRLA.reg =		SERCOM_UART_CTRLA_MODE(mode) |
								SERCOM_USART_CTRLA_SAMPR(sampleRate);

	if(mode == UART_INT_CLOCK)
	{
		uint32_t sampleRateValue;	
		if(sampleRate == SAMPLE_RATE_x16)
		{
			sampleRateValue = 16;
		}
		else if(sampleRate == SAMPLE_RATE_x8)
		{
			sampleRateValue = 8;
		}
		else
		{
			sampleRateValue = 3;
		}

		//Asynchronous arithmetic mode
		sercomUart->BAUD.reg = 65535 * ( 1 - sampleRateValue * (baudrate / SERCOM_FREQ_REF));
	}
}
void initFrame(SercomCharSize charSize, SercomDataOrder dataOrder, SercomParityMode parityMode, SercomNumberStopBit nbStopBits)
{
	sercomUart->CTRLA.reg |=	SERCOM_UART_CTRLA_FORM( (parityMode == NO_PARITY ? 0 : 1) ) |
								dataOrder << SERCOM_UART_CTRLA_DORD_Pos;

	sercomUart->CTRLB.reg |=	SERCOM_UART_CTRLB_CHSIZE(charSize) |
								nbStopBits << SERCOM_UART_CTRLB_SBMODE_Pos |
								(parityMode == NO_PARITY ? 0 : parityMode) << SERCOM_UART_CTRLB_PMODE_Pos |;
}

void initPads(SercomUartTXPad txPad, SercomUartRXPad rxPad)
{
	sercomUart->CTRLA.reg |=	SERCOM_UART_CTRLA_TXPO(txPad) |
								SERCOM_UART_CTRLA_RXPO(rxPad);

	sercomUart->CTRLB.reg |=	SERCOM_UART_CTRLB_TXEN |
								SERCOM_UART_CTRLB_RXEN;
}

void SERCOM::resetUART()
{
	sercomUart->CTRLA.bit.SWRST = 0x1u;
	while(sercomUart->CTRLA.bit.SWRST || sercomUart->SYNCBUSY.SWRST);
}

void SERCOM::enableUART()
{
	sercomUart->CTRLA.bit.ENABLE = 0x1u;
}

void SERCOM::flushUART()
{
	while(sercomUart->INTFLAG.bit.DRE != SERCOM_USART_INTFLAG_DRE);
}

void SERCOM::clearStatusUART()
{
	sercomUart->STATUS.reg = SERCOM_USART_STATUS_RESETVALUE;
}

bool SERCOM::availableDataUART()
{
	return sercomUart->INTFLAG.bit.RXC;
}

bool SERCOM::isBufferOverflowErrorUART()
{
	return sercomUart->STATUS.bit.BUFOVF;
}

bool SERCOM::isFrameErrorUART()
{
	return sercomUart->STATUS.bit.FERR;
}

bool SERCOM::isParityErrorUART()
{
	return sercomUart->STATUS.bit.PERR;
}

uint8_t SERCOM::readDataUART()
{
	return sercomUart->DATA.bit.DATA;
}

int SERCOM::writeDataUART(uint8_t data)
{
	flushUART();

	sercomUart->DATA.bit.DATA = data;
	return 1;
}

