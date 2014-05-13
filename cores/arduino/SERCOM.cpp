#include "SERCOM.h"

// Constants for Clock multiplexers
#define GENERIC_CLOCK_SERCOM0	(0x14ul)
#define GENERIC_CLOCK_SERCOM1	(0x15ul)
#define GENERIC_CLOCK_SERCOM2	(0x16ul)
#define GENERIC_CLOCK_SERCOM3	(0x17ul)
#define GENERIC_CLOCK_SERCOM4	(0x18ul)
#define GENERIC_CLOCK_SERCOM5	(0x19ul)

SERCOM::SERCOM(Sercom* s)
{
	sercom = s;
}

/* 	=========================
 *	===== Sercom UART
 *	=========================
*/
void SERCOM::initUART(SercomUartMode mode, SercomUartSampleRate sampleRate, uint32_t baudrate)
{		
	resetUART();
	initClock();
	initNVIC();
	
	//Setting the CTRLA register
	sercom->USART.CTRLA.reg =	SERCOM_USART_CTRLA_MODE(mode) |
								SERCOM_USART_CTRLA_SAMPR(sampleRate);

	//Setting the Interrupt register
	sercom->USART.INTENSET.reg =	SERCOM_USART_INTENSET_RXC |  //Received complete
									SERCOM_USART_INTENSET_ERROR; //All others errors
									
	
	if(mode == UART_INT_CLOCK)
	{
		uint16_t sampleRateValue;	
		
		if(sampleRate == SAMPLE_RATE_x16)
			sampleRateValue = 16;
		else if(sampleRate == SAMPLE_RATE_x8)
			sampleRateValue = 8;
		else
			sampleRateValue = 3;
		
		//Asynchronous arithmetic mode
		//65535 * ( 1 - sampleRateValue * baudrate / SERCOM_FREQ_REF);
		sercom->USART.BAUD.reg = 65535.0 * ( 1.0 - (float)(sampleRateValue) * (float)(baudrate) / (float)(SERCOM_FREQ_REF));
		
	}
}
void SERCOM::initFrame(SercomUartCharSize charSize, SercomDataOrder dataOrder, SercomParityMode parityMode, SercomNumberStopBit nbStopBits)
{
	//Setting the CTRLA register
	sercom->USART.CTRLA.reg |=	SERCOM_USART_CTRLA_FORM( (parityMode == SERCOM_NO_PARITY ? 0 : 1) ) |
								dataOrder << SERCOM_USART_CTRLA_DORD_Pos;

	//Setting the CTRLB register
	sercom->USART.CTRLB.reg |=	SERCOM_USART_CTRLB_CHSIZE(charSize) |
								nbStopBits << SERCOM_USART_CTRLB_SBMODE_Pos |
								(parityMode == SERCOM_NO_PARITY ? 0 : parityMode) << SERCOM_USART_CTRLB_PMODE_Pos; //If no parity use default value
}

void SERCOM::initPads(SercomUartTXPad txPad, SercomRXPad rxPad)
{
	//Setting the CTRLA register
	sercom->USART.CTRLA.reg |=	SERCOM_USART_CTRLA_TXPO(txPad) |
								SERCOM_USART_CTRLA_RXPO(rxPad);

	//Setting the CTRLB register (Enabling Transceiver and Receiver)
	sercom->USART.CTRLB.reg |=	SERCOM_USART_CTRLB_TXEN |
								SERCOM_USART_CTRLB_RXEN;
}

void SERCOM::resetUART()
{
	//Setting  the Software bit to 1
	sercom->USART.CTRLA.bit.SWRST = 0x1u;
	
	//Wait for both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
	while(sercom->USART.CTRLA.bit.SWRST || sercom->USART.SYNCBUSY.bit.SWRST);
}

void SERCOM::enableUART()
{
	//Setting  the enable bit to 1
	sercom->USART.CTRLA.bit.ENABLE = 0x1u;
	
	//Wait for then enable bit from SYNCBUSY is equal to 0;
	while(sercom->USART.SYNCBUSY.bit.ENABLE);
}

void SERCOM::flushUART()
{
	// Wait for transmission to complete
	while(sercom->USART.INTFLAG.bit.DRE != SERCOM_USART_INTFLAG_DRE);
}

void SERCOM::clearStatusUART()
{
	//Reset (with 0) the STATUS register
	sercom->USART.STATUS.reg = SERCOM_USART_STATUS_RESETVALUE;
}

bool SERCOM::availableDataUART()
{
	//RXC : Receive Complete
	return sercom->USART.INTFLAG.bit.RXC;
}

bool SERCOM::isBufferOverflowErrorUART()
{
	//BUFOVF : Buffer Overflow
	return sercom->USART.STATUS.bit.BUFOVF;
}

bool SERCOM::isFrameErrorUART()
{
	//FERR : Frame Error
	return sercom->USART.STATUS.bit.FERR;
}

bool SERCOM::isParityErrorUART()
{
	//PERR : Parity Error
	return sercom->USART.STATUS.bit.PERR;
}

bool SERCOM::isDataRegisterEmptyUART()
{
	//DRE : Data Register Empty
	return sercom->USART.INTFLAG.bit.DRE;
}

uint8_t SERCOM::readDataUART()
{
	return sercom->USART.DATA.bit.DATA;
}

int SERCOM::writeDataUART(uint8_t data)
{
	//Flush UART buffer
	flushUART();

	//Put data into DATA register
	sercom->USART.DATA.reg = (uint16_t)data;
	return 1;
}

/*	=========================
 *	===== Sercom SPI
 *	=========================
*/
void SERCOM::initSPI(SercomSpiTXPad mosi, SercomRXPad miso, SercomSpiCharSize charSize, SercomDataOrder dataOrder)
{
	resetSPI();
	
	//Setting the CTRLA register
	sercom->SPI.CTRLA.reg =	SERCOM_SPI_CTRLA_MODE(SPI_MASTER_OPERATION) |
							SERCOM_SPI_CTRLA_DOPO(mosi) |
							SERCOM_SPI_CTRLA_DIPO(miso) |
							dataOrder << SERCOM_SPI_CTRLA_DORD_Pos;
	
	//Setting the CTRLB register
	sercom->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE(charSize) |
							(0x1ul) << SERCOM_SPI_CTRLB_RXEN_Pos;	//Active the SPI receiver.
}

void SERCOM::initSPIClock(SercomSpiClockMode clockMode, uint32_t baudrate)
{
	//Extract data from clockMode
	int cpha, cpol;
	
	if((clockMode & (0x1ul)) == 0 )	
		cpha = 0;
	else
		cpha = 1;
		
	if((clockMode & (0x2ul)) == 0)
		cpol = 0;
	else
		cpol = 1;
		
	//Setting the CTRLA register
	sercom->SPI.CTRLA.reg |=	cpha << SERCOM_SPI_CTRLA_CPHA_Pos |
								cpol << SERCOM_SPI_CTRLA_CPOL_Pos;
	
	//Synchronous arithmetic
	sercom->SPI.BAUD.reg = calculateBaudrateSynchronous(baudrate);
}

void SERCOM::resetSPI()
{
	//Setting the Software bit to 1
	sercom->SPI.CTRLA.bit.SWRST = 0x1u;

	//Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
	while(sercom->SPI.CTRLA.bit.SWRST || sercom->SPI.SYNCBUSY.bit.SWRST);
}
	
void SERCOM::enableSPI()
{
	//Setting the enable bit to 1
	sercom->SPI.CTRLA.bit.ENABLE = 0x1ul;
	
	//Waiting then enable bit from SYNCBUSY is equal to 0;
	while(sercom->SPI.SYNCBUSY.bit.ENABLE);
}
	
void SERCOM::disableSPI()
{
	//Setting the enable bit to 0
	sercom->SPI.CTRLA.bit.ENABLE = 0x0ul;
	
	//Waiting then enable bit from SYNCBUSY is equal to 0;
	while(sercom->SPI.SYNCBUSY.bit.ENABLE);
}

void SERCOM::setDataOrderSPI(SercomDataOrder dataOrder)
{
	//Register enable-protected
	disableSPI();
	
	sercom->SPI.CTRLA.bit.DORD = dataOrder;
	
	enableSPI();
}

void SERCOM::setBaudrateSPI(uint8_t divider)
{
	//Can't divide by 0
	if(divider == 0)
		return;
		
	//Register enable-protected
	disableSPI();
	
	sercom->SPI.BAUD.reg = calculateBaudrateSynchronous(SERCOM_FREQ_REF / divider);
	
	enableSPI();
}

void SERCOM::setClockModeSPI(SercomSpiClockMode clockMode)
{
	int cpha, cpol;
	if((clockMode & (0x1ul)) == 0)
		cpha = 0;
	else
		cpha = 1;

	if((clockMode & (0x2ul)) == 0)
		cpol = 0;
	else
		cpol = 1;

	//Register enable-protected
	disableSPI();
	
	sercom->SPI.CTRLA.bit.CPOL = cpol;
	sercom->SPI.CTRLA.bit.CPHA = cpha;
	
	enableSPI();
}
void SERCOM::writeDataSPI(uint8_t data)
{
	sercom->SPI.DATA.bit.DATA = data;
}

uint8_t SERCOM::readDataSPI()
{
	return sercom->SPI.DATA.bit.DATA;
}

bool SERCOM::isBufferOverflowErrorSPI()
{
	return sercom->SPI.STATUS.bit.BUFOVF;
}

bool SERCOM::isDataRegisterEmptySPI()
{
	//DRE : Data Register Empty
	return sercom->SPI.INTFLAG.bit.DRE;
}

bool SERCOM::isTransmitCompleteSPI()
{
	//TXC : Transmit complete
	return sercom->SPI.INTFLAG.bit.TXC;
}

bool SERCOM::isReceiveCompleteSPI()
{
	//RXC : Receive complete
	return sercom->SPI.INTFLAG.bit.RXC;
}

uint8_t SERCOM::calculateBaudrateSynchronous(uint32_t baudrate)
{
	return SERCOM_FREQ_REF / (2 * baudrate) - 1;
}


/*	=========================
 *	===== Sercom WIRE
 *	=========================
*/

void SERCOM::resetWIRE()
{
	//I2CM OR I2CS, no matter SWRST is the same bit.

	//Setting the Software bit to 1
	sercom->I2CM.CTRLA.bit.SWRST = 0x1ul;

	//Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
	while(sercom->I2CM.CTRLA.bit.SWRST || sercom->I2CM.SYNCBUSY.bit.SWRST);
}

void SERCOM::enableWIRE()
{
	//I2CM OR I2CS, no matter ENABLE is the same bit.
	
	//Setting the enable bit to 1
	sercom->I2CM.CTRLA.bit.ENABLE = 0x1ul;
	
	//Waiting the enable bit from SYNCBUSY is equal to 0;
	while(sercom->I2CM.SYNCBUSY.bit.ENABLE);
}

void SERCOM::initSlaveWIRE(uint8_t address)
{
	resetWIRE();
	
	//Setting slave mode
	sercom->I2CS.CTRLA.bit.MODE = I2C_SLAVE_OPERATION;
	
	//Enable Quick Command
	sercom->I2CM.CTRLB.bit.QCEN = 0x1ul;
	
	sercom->I2CS.ADDR.reg = SERCOM_I2CS_ADDR_ADDR(address & 0x7Ful) | // 0x7F, select only 7 bits
							SERCOM_I2CS_ADDR_ADDRMASK(0x3FFul);		// 0x3FF all bits set
	
	//Setting the interrupt register
	sercom->I2CS.INTENSET.reg = SERCOM_I2CS_INTENSET_AMATCH | //Address Match
								SERCOM_I2CS_INTENSET_DRDY; //Data Ready
							
	//Waiting the SYSOP bit from SYNCBUSY is equal to 0;
	while(sercom->I2CM.SYNCBUSY.bit.SYSOP);
}

void SERCOM::initMasterWIRE(uint32_t baudrate)
{
	resetWIRE();
	
	//Setting master mode and SCL Clock Stretch mode (stretch after ACK bit)
	sercom->I2CM.CTRLA.reg = 	SERCOM_I2CM_CTRLA_MODE(I2C_MASTER_OPERATION) |
								SERCOM_I2CM_CTRLA_SCLSM;
								
	//Enable Quick Command
	sercom->I2CM.CTRLB.bit.QCEN = 0x1ul;
	
	//Setting bus idle mode
	sercom->I2CM.STATUS.bit.BUSSTATE = WIRE_IDLE_STATE;
	
	//Setting all interrupts
	sercom->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_MB |
								SERCOM_I2CM_INTENSET_SB |
								SERCOM_I2CM_INTENSET_ERROR;
	
	//Synchronous
	sercom->I2CM.BAUD.bit.BAUD = SERCOM_FREQ_REF / ( 2 * baudrate) - 1;
	
	//Waiting the SYSOP bit from SYNCBUSY is equal to 0;
	while(sercom->I2CM.SYNCBUSY.bit.SYSOP);
}

void SERCOM::prepareStopBitWIRE()
{
	sercom->I2CM.CTRLB.bit.CMD = WIRE_MASTER_ACT_STOP;
}

void SERCOM::prepareAckBitWIRE()
{
	sercom->I2CM.CTRLB.bit.CMD = WIRE_MASTER_ACT_READ;
}

bool SERCOM::startTransmissionWIRE(uint8_t address, SercomWireReadWriteFlag flag)
{
	//7-bits address + 1-bits R/W
	address = (address << 1) | flag;

	//Wait idle bus mode
	while(!isBusIdleWIRE());
	
	//Send start and address
	sercom->I2CM.ADDR.bit.ADDR = address;
	
	
	//Address Transmitted
	while(!sercom->I2CM.INTFLAG.bit.MB);
	
	//ACK received (0: ACK, 1: NACK)
	if(sercom->I2CM.STATUS.bit.RXNACK)
		return false;
	else
		return true;
}

bool SERCOM::sendDataMasterWIRE(uint8_t data)
{
	//Send data
	sercom->I2CM.DATA.bit.DATA = data;
	
	//Wait transmission successful
	while(!sercom->I2CM.INTFLAG.bit.MB);
	
	//Problems on line? nack received?
	if(sercom->I2CM.STATUS.bit.RXNACK)
		return false;
	else
		return true;
}

bool SERCOM::sendDataSlaveWIRE(uint8_t data)
{
	//Send data
	sercom->I2CS.DATA.bit.DATA = data;
	
	//Wait data transmission successful
	while(!sercom->I2CS.INTFLAG.bit.DRDY);
	
	//Problems on line? nack received?
	if(sercom->I2CS.STATUS.bit.RXNACK)
		return false;
	else
		return true;
}

bool SERCOM::isMasterWIRE()
{
	return sercom->I2CS.CTRLA.bit.MODE == I2C_MASTER_OPERATION;
}

bool SERCOM::isSlaveWIRE()
{
	return sercom->I2CS.CTRLA.bit.MODE == I2C_SLAVE_OPERATION;
}

bool SERCOM::isBusIdleWIRE()
{
	return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_IDLE_STATE;
}

bool SERCOM::isDataReadyWIRE()
{
	return sercom->I2CS.INTFLAG.bit.DRDY;
}

bool SERCOM::isStopDetectedWIRE()
{
	return sercom->I2CS.INTFLAG.bit.PREC;
}

bool SERCOM::isRestartDetectedWIRE()
{
	return sercom->I2CS.STATUS.bit.SR;
}

bool SERCOM::isAddressMatch()
{
	return sercom->I2CS.INTFLAG.bit.AMATCH;
}

bool SERCOM::isMasterReadOperationWIRE()
{
	return sercom->I2CS.STATUS.bit.DIR;
}

int SERCOM::availableWIRE()
{
	if(isMasterWIRE())
		return sercom->I2CM.INTFLAG.bit.SB;
	else
		return sercom->I2CS.INTFLAG.bit.DRDY;
}

uint8_t SERCOM::readDataWIRE()
{
	if(isMasterWIRE())
		return sercom->I2CM.DATA.reg;
	else
		return sercom->I2CS.DATA.reg;
}


void SERCOM::initClock()
{
	uint8_t clockId = 0;
	
	if(sercom == SERCOM0)
	{
		clockId = GENERIC_CLOCK_SERCOM0;
	}
	else if(sercom == SERCOM1)
	{
		clockId = GENERIC_CLOCK_SERCOM1;
	}
	else if(sercom == SERCOM2)
	{
		clockId = GENERIC_CLOCK_SERCOM2;
	}
	else if(sercom == SERCOM3)
	{
		clockId = GENERIC_CLOCK_SERCOM3;
	}
	else if(sercom == SERCOM4)
	{
		clockId = GENERIC_CLOCK_SERCOM4;
	}
	else if(sercom == SERCOM5)
	{
		clockId = GENERIC_CLOCK_SERCOM5;
	}
	
	//Setting clock
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( clockId ) | // Generic Clock 0 (SERCOMx)
	GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is source
	GCLK_CLKCTRL_CLKEN ;
	

	while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
	{
		/* Wait for synchronization */
	}
}

void SERCOM::initNVIC()
{
		IRQn_Type Id;
		
		if(sercom == SERCOM0)
		{
			Id = SERCOM0_IRQn;
		}
		else if(sercom == SERCOM1)
		{
			Id = SERCOM1_IRQn;
		}
		else if(sercom == SERCOM2)
		{
			Id = SERCOM2_IRQn;
		}
		else if(sercom == SERCOM3)
		{
			Id = SERCOM3_IRQn;
		}
		else if(sercom == SERCOM4)
		{
			Id = SERCOM4_IRQn;
		}
		else if(sercom == SERCOM5)
		{
			Id = SERCOM5_IRQn;
		}
		
	NVIC_EnableIRQ(Id);
	NVIC_SetPriority (Id, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority */
}

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
*/
SERCOM * SERCOM::sercom0 = new SERCOM(SERCOM0);
SERCOM * SERCOM::sercom1 = new SERCOM(SERCOM1);
SERCOM * SERCOM::sercom2 = new SERCOM(SERCOM2);
SERCOM * SERCOM::sercom3 = new SERCOM(SERCOM3);
SERCOM * SERCOM::sercom4 = new SERCOM(SERCOM4);
SERCOM * SERCOM::sercom5 = new SERCOM(SERCOM5);
