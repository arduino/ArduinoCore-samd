#ifndef _SERCOM_CLASS_
#define _SERCOM_CLASS_

#include "sam.h"

#define SERCOM_FREQ_REF 48000000

typedef enum
{
	UART_EXT_CLOCK = 0,
	UART_INT_CLOCK = 0x1u
} SercomUartMode;

typedef enum
{
	SPI_SLAVE_OPERATION = 0x2u,
	SPI_MASTER_OPERATION = 0x3u
} SercomSpiMode;

typedef enum
{
	I2C_SLAVE_OPERATION = 0x4u,
	I2C_MASTER_OPERATION = 0x5u
} SercomI2CMode;

typedef enum
{
	EVEN_PARITY = 0,
	ODD_PARITY,
	NO_PARITY
} SercomParityMode;

typedef enum
{
	1_STOP_BIT = 0,
	2_STOP_BITS
} SercomNumberStopBit;

typedef enum
{
	MSB_FIRST = 0,
	LSB_FIRST
} SercomDataOrder;

typedef enum
{
	8_BITS = 0,
	9_BITS,
	5_BITS = 0x5u,
	6_BITS,
	7_BITS
} SercomUartCharSize;

typedef enum
{
	PAD_0 = 0,
	PAD_1,
	PAD_2,
	PAD_3
} SercomRXPad;

Vtypedef enum
{
	PAD_0 = 0,	//Only for Intern Clock
	PAD_1 = 0,	//Only for Extern Clock
	PAD_2 = 1,  //Only for Intern Clock
	PAD_3 = 1	//Only for Extern Clock
} SercomUartTXPad;

typedef enum
{
	SAMPLE_RATE_x16 = 0,	//Arithmetic
	SAMPLE_RATE_x8 = 0x2,	//Arithmetic
	SAMPLE_RATE_x3 = 0x3	//Arithmetic
} SercomUartSampleRate;

typedef enum
{
	SPI_MODE_0 = 0,	// CPOL : 0  | CPHA : 0
	SPI_MODE_1,		// CPOL : 0  | CPHA : 1
	SPI_MODE_2,		// CPOL : 1  | CPHA : 0
	SPI_MODE_3		// CPOL : 1  | CPHA : 1
} SercomSpiClockMode;

typedef enum
{
	PAD_0_SCK_1 = 0,
	PAD_2_SCK_3,
	PAD_3_SCK_1,
	PAD_0_SCK_3
} SercomSpiTXPad;

typedef enum
{
	8_BITS = 0,
	9_BITS = 1
} SercomSpiCharSize;

class SERCOM
{
	public:
		SERCOM(Sercom* sercom);
	
		/* ========== UART ========== */
		void initUART(SercomUartMode mode, SercomUartSampleRate sampleRate, uint32_t baudrate=0);
		void initFrame(SercomUartCharSize charSize, SercomDataOrder dataOrder, SercomParityMode parityMode, SercomNumberStopBit nbStopBits);
		void initPads(SercomUartTXPad txPad, SercomRXPad rxPad);
		
		void resetUART();
		void enableUART();
		void flushUART();
		void clearStatusUART();
		bool availableDataUART();
		bool isBufferOverflowErrorUART();
		bool isFrameErrorUART();
		bool isParityErrorUART();
		bool isDataRegisterEmptyUART()
		uint8_t readDataUART();
		int writeDataUART(uint8_t data);

		/* ========== SPI ========== */
		void initSPI(SercomSpiTXPad mosi, SercomRXPad miso, SercomSpiCharSize charSize, SercomDataOrder dataOrder);
		void initClock(SercomSpiClockMode clockMode, uint32_t baudrate);
		
		void resetSPI();
		void enableSPI();
		void disableSPI();
		void setDataOrderSPI(SercomDataOrder dataOrder);
		void setBaudrateSPI(uint8_t divider);
		void setClockModeSPI(SercomSpiClockMode clockMode);
		void writeDataSPI(uint8_t data);
		uint8_t readDataSPI();
		bool isBufferOverflowErrorSPI();
		bool isDataRegisterEmptySPI();
		bool isTransmitCompleteSPI();
		bool isReceiveCompleteSPI();
		

	private:
		Sercom* sercom;
		uint8_t calculateBaudrateSynchronous(uint32_t baudrate);
};

#endif
