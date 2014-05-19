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
	SERCOM_EVEN_PARITY = 0,
	SERCOM_ODD_PARITY,
	SERCOM_NO_PARITY
} SercomParityMode;

typedef enum
{
	SERCOM_STOP_BIT_1 = 0,
	SERCOM_STOP_BITS_2
} SercomNumberStopBit;

typedef enum
{
	MSB_FIRST = 0,
	LSB_FIRST
} SercomDataOrder;

typedef enum
{
	UART_CHAR_SIZE_8_BITS = 0,
	UART_CHAR_SIZE_9_BITS,
	UART_CHAR_SIZE_5_BITS = 0x5u,
	UART_CHAR_SIZE_6_BITS,
	UART_CHAR_SIZE_7_BITS
} SercomUartCharSize;

typedef enum
{
	SERCOM_RX_PAD_0 = 0,
	SERCOM_RX_PAD_1,
	SERCOM_RX_PAD_2,
	SERCOM_RX_PAD_3
} SercomRXPad;

typedef enum
{
	UART_TX_PAD_0 = 0x0ul,	//Only for UART
	UART_TX_PAD_2 = 0x1ul,  //Only for UART
	//UART_TX_PAD_1 = 0x0ul,	//DON'T USE
	//UART_TX_PAD_3 = 0x1ul	//DON'T USE
} SercomUartTXPad;

typedef enum
{
	SAMPLE_RATE_x16 = 0,	//Arithmetic
	SAMPLE_RATE_x8 = 0x2,	//Arithmetic
	SAMPLE_RATE_x3 = 0x3	//Arithmetic
} SercomUartSampleRate;

typedef enum
{
	SERCOM_SPI_MODE_0 = 0,	// CPOL : 0  | CPHA : 0
	SERCOM_SPI_MODE_1,		// CPOL : 0  | CPHA : 1
	SERCOM_SPI_MODE_2,		// CPOL : 1  | CPHA : 0
	SERCOM_SPI_MODE_3		// CPOL : 1  | CPHA : 1
} SercomSpiClockMode;

typedef enum
{
	SPI_PAD_0_SCK_1 = 0,
	SPI_PAD_2_SCK_3,
	SPI_PAD_3_SCK_1,
	SPI_PAD_0_SCK_3
} SercomSpiTXPad;

typedef enum
{
	SPI_CHAR_SIZE_8_BITS = 0x0ul,
	SPI_CHAR_SIZE_9_BITS
} SercomSpiCharSize;

typedef enum
{
	WIRE_UNKNOWN_STATE = 0x0ul,
	WIRE_IDLE_STATE,
	WIRE_OWNER_STATE,
	WIRE_BUSY_STATE
} SercomWireBusState;

typedef enum
{
	WIRE_WRITE_FLAG = 0x0ul,
	WIRE_READ_FLAG
} SercomWireReadWriteFlag;

typedef enum
{
	WIRE_MASTER_ACT_NO_ACTION = 0,
	WIRE_MASTER_ACT_REPEAT_START,
	WIRE_MASTER_ACT_READ,
	WIRE_MASTER_ACT_STOP
} SercomMasterCommandWire;

typedef enum
{
	WIRE_MASTER_ACK_ACTION = 0,
	WIRE_MASTER_NACK_ACTION
} SercomMasterAckActionWire;

class SERCOM
{
	public:
		SERCOM(Sercom* s) ;

		/* ========== UART ========== */
		void initUART(SercomUartMode mode, SercomUartSampleRate sampleRate, uint32_t baudrate=0) ;
		void initFrame(SercomUartCharSize charSize, SercomDataOrder dataOrder, SercomParityMode parityMode, SercomNumberStopBit nbStopBits) ;
		void initPads(SercomUartTXPad txPad, SercomRXPad rxPad) ;

		void resetUART( void ) ;
		void enableUART( void ) ;
		void flushUART( void ) ;
		void clearStatusUART( void ) ;
		bool availableDataUART( void ) ;
		bool isBufferOverflowErrorUART( void ) ;
		bool isFrameErrorUART( void ) ;
		bool isParityErrorUART( void ) ;
		bool isDataRegisterEmptyUART( void ) ;
		uint8_t readDataUART( void ) ;
		int writeDataUART(uint8_t data) ;

		/* ========== SPI ========== */
		void initSPI(SercomSpiTXPad mosi, SercomRXPad miso, SercomSpiCharSize charSize, SercomDataOrder dataOrder) ;
		void initSPIClock(SercomSpiClockMode clockMode, uint32_t baudrate) ;

		void resetSPI( void ) ;
		void enableSPI( void ) ;
		void disableSPI( void ) ;
		void setDataOrderSPI(SercomDataOrder dataOrder) ;
		void setBaudrateSPI(uint8_t divider) ;
		void setClockModeSPI(SercomSpiClockMode clockMode) ;
		void writeDataSPI(uint8_t data) ;
		uint16_t readDataSPI( void ) ;
		bool isBufferOverflowErrorSPI( void ) ;
		bool isDataRegisterEmptySPI( void ) ;
		bool isTransmitCompleteSPI( void ) ;
		bool isReceiveCompleteSPI( void ) ;

		/* ========== WIRE ========== */
		void initSlaveWIRE(uint8_t address) ;
		void initMasterWIRE(uint32_t baudrate) ;

		void resetWIRE( void ) ;
		void enableWIRE( void ) ;
    void disableWIRE( void );
    void sendNackBitWIRE( void ) ;
    void sendAckBitWIRE( void ) ;
    void sendStopBitWIRE( void ) ;
		bool startTransmissionWIRE(uint8_t address, SercomWireReadWriteFlag flag) ;
		bool sendDataMasterWIRE(uint8_t data) ;
		bool sendDataSlaveWIRE(uint8_t data) ;
		bool isMasterWIRE( void ) ;
		bool isSlaveWIRE( void ) ;
		bool isBusIdleWIRE( void ) ;
		bool isDataReadyWIRE( void ) ;
		bool isStopDetectedWIRE( void ) ;
		bool isRestartDetectedWIRE( void ) ;
		bool isAddressMatch( void ) ;
		bool isMasterReadOperationWIRE( void ) ;
    bool isRXNackReceivedWIRE( void ) ;
		int availableWIRE( void ) ;
		uint8_t readDataWIRE( void ) ;

	private:
		Sercom* sercom;
		uint8_t calculateBaudrateSynchronous(uint32_t baudrate) ;
		uint32_t division(uint32_t dividend, uint32_t divisor) ;
		void initClockNVIC( void ) ;
};

#endif
