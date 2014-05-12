#ifndef _SERCOM_UART_CLASS
#define _SERCOM_UART_CLASS

#include "HardwareSerial.h"
#include "SERCOM.h"
#include "RingBuffer.h"

#include <cstddef>


class Uart : public HardwareSerial
{
	public:
		Uart(SERCOM *s, uint8_t pinRX, uint8_t pinTX);
		void begin(unsigned long baudRate);
		void begin(unsigned long baudrate, uint8_t config);
		void end();
		int available();
		int peek();
		int read();
		void flush();
		size_t write(const uint8_t data);
		size_t write(const char * data);

		void IrqHandler();

		operator bool() { return true; }

	private:
		SERCOM *sercom;
		RingBuffer rxBuffer;

		SercomNumberStopBit extractNbStopBit(uint8_t config);
		SercomUartCharSize extractCharSize(uint8_t config);
		SercomParityMode extractParity(uint8_t config);
};

extern Uart Serial;
extern Uart Serial5;


#endif
