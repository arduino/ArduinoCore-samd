#ifndef _SERCOM_UART_CLASS
#define _SERCOM_UART_CLASS

#include "SERCOM.h"
#include "RingBuffer.h"


class SERCOMUart
{
	public:
		SERCOMUart(SERCOM *sercom);
		void begin(uint16_t baudRate);
		void begin(uint16_t baudrate, uint8_t config);
		void end();
		int available();
		int peek();
		int read();
		void flush();
		size_t write(const uint8_t c);

		void IrqHandler();

		operator bool() { return true; }

	private:
		SERCOM *sercom;
		RingBuffer rxBuffer;
		RingBuffer txBuffer;

		SercomNumberStopBit extractNbStopBit(uint8_t config);
		SercomCharSize extractCharSize(uint8_t config);
		SercomParityMode extractParity(uint8_t config)
};


#endif
