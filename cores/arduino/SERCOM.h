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
} SercomCharSize;

typedef enum
{
	PAD_0 = 0,
	PAD_1,
	PAD_2,
	PAD_3
} SercomUartRXPad;

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
{//					CPOL	CPHA
	MODE_0 = 0,	//	  0		  0			
	MODE_1,		//	  0		  1
	MODE_2,		//	  1		  0	
	MODE_3		//	  1		  1
} SercomSpiMode;

class SERCOM
{
	public:
		SERCOM(Sercom* sercom);
	
		/* ========== UART ========== */
		void initUART(SercomUartMode mode, SercomUartTXPad txPad, SercomUartRXPad rxPad, SercomCharSize charSize, 
						SercomDataOrder dataOrder, SercomParityMode parityMode, SercomNumberStopBit nbStopBits,
						SercomUartSampleRate sampleRate, uint32_t baudrate=0);
		void resetUART();
		void enableUART();
		void flushUART();
		void clearStatusUART();
		bool availableDataUART();
		bool isBufferOverflowErrorUART();
		bool isFrameErrorUART();
		bool isParityErrorUART();
		uint8_t readDataUART();
		int writeDataUART(uint8_t data);

		/* ========== SPI ========== */
		void initSPI();
		

	private:
		Sercom* sercom;
		SercomUart* sercomUart;
		SercomSpi* sercomSpi;
};

#endif
