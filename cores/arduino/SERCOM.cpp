#include "SERCOM.h"


SERCOM::SERCOM(Sercom* sercom)
{
	this->sercom = sercom;
	sercomUart = sercom->UART;
}

void SERCOM::initUART(SercomUartMode mode, SercomUartTXPad txPad, SercomUartRXPad rxPad, SercomCharSize charSize, 
						SercomDataOrder dataOrder, SercomParityMode parityMode, SercomNumberStopBit nbStopBits,
						SercomUartSampleRate sampleRate, uint32_t baudrate)
{
	resetUART();
	
	sercomUart->CTRLA.reg =		SERCOM_UART_CTRLA_MODE(mode) |
								SERCOM_USART_CTRLA_SAMPR(sampleRate) |
								SERCOM_UART_CTRLA_TXPO(txPad) |
								SERCOM_UART_CTRLA_RXPO(rxPad) |
								SERCOM_UART_CTRLA_FORM( (parityMode == NO_PARITY ? 0 : 1) ) |
								dataOrder << SERCOM_UART_CTRLA_DORD_Pos;

	sercomUart->CTRLB.reg =		SERCOM_UART_CTRLB_CHSIZE(charSize) |
								nbStopBits << SERCOM_UART_CTRLB_SBMODE_Pos |
								(parityMode == NO_PARITY ? 0 : parityMode) << SERCOM_UART_CTRLB_PMODE_Pos |
								SERCOM_UART_CTRLB_TXEN |
								SERCOM_UART_CTRLB_RXEN;

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
		sercomUart->BAUD.reg = 65535 * ( 1 - sampleRateValue * (baudrate / UART_FREQ_REF));
	}
}

void SERCOM::resetUART()
{
	sercomUart->CTRLA.SWRST = 0x1u;
	while(sercomUart->CTRLA.SWRST || sercomUart->SYNCBUSY.SWRST);
}

void SERCOM::enableUART()
{
	sercomUart->CTRLA.ENABLE = 0x1u;
}

bool SERCOM::availableDataUART()
{
	return sercomUart->INTFLAG.RXC;
}

