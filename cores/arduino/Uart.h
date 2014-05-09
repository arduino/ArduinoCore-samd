#ifndef _SERCOM_UART_CLASS
#define _SERCOM_UART_CLASS

#include "HardwareSerial.h"
#include "SERCOM.h"
#include "RingBuffer.h"

#include <cstddef>


class Uart : public HardwareSerial
{
	public:
		Uart(SERCOM *sercom);
		void begin(unsigned long baudRate);
		void begin(unsigned long baudrate, uint8_t config);
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
		SercomUartCharSize extractCharSize(uint8_t config);
		SercomParityMode extractParity(uint8_t config);
};
extern Uart Serial;


#endif
