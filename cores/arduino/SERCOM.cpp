/*
  Copyright (c) 2014 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "SERCOM.h"
#include "variant.h"

uint32_t SercomClock = 1000000ul; // this default is changed in initClockNVIC()

#ifndef WIRE_RISE_TIME_NANOSECONDS
// Default rise time in nanoseconds, based on 4.7K ohm pull up resistors
// you can override this value in your variant if needed
#define WIRE_RISE_TIME_NANOSECONDS 125
#endif

SERCOM::SERCOM(Sercom* s)
{
  sercom = s;
}

#if 0
// From https://en.wikipedia.org/wiki/Division_algorithm
static uint64_t divide64(uint64_t n, uint64_t d)
{
  uint64_t q = 0, r = 0;

  for (int8_t i = 63; i >= 0; i--) {
    r = r << 1;

    // set the LSB of r equal to bit i of n
    // because r was just shifted right by one, we don't need to explicitly set r to 0 if bit i of n is 0.
    if (n & ((uint64_t)1 << i)) {
      r |= 0x01;
    }

    if (r >= d) {
      r = r - d;
      q |= ((uint64_t)1 << i);
    }
  }

  return q;
}
#endif

/* 	=========================
 *	===== Sercom UART
 *	=========================
*/
void SERCOM::initUART(SercomUartMode mode, SercomUartSampleRate sampleRate, uint32_t baudrate)
{
#if (SAML21)
  // On the SAML21, SERCOM5 is on PD0, which is a low power domain on a different bridge than the other SERCOMs.
  // SERCOM5 does not support SAMPLE_RATE_x8 or SAMPLE_RATE_x3.
  if (sercom == SERCOM5) {
    sampleRate = SAMPLE_RATE_x16;
  }
#endif

  initClockNVIC();
  resetUART();

  //Setting the CTRLA register
  sercom->USART.CTRLA.reg =	SERCOM_USART_CTRLA_MODE(mode) |
                SERCOM_USART_CTRLA_SAMPR(sampleRate);

  //Setting the Interrupt register
  sercom->USART.INTENSET.reg =	SERCOM_USART_INTENSET_RXC |  //Received complete
                                SERCOM_USART_INTENSET_ERROR; //All others errors

  if ( mode == UART_INT_CLOCK )
  {
    uint16_t sampleRateValue;

    if (sampleRate == SAMPLE_RATE_x16) {
      sampleRateValue = 16;
    } else {
      sampleRateValue = 8;
    }

#if 0
    // Asynchronous arithmetic mode
    // 65535 * ( 1 - sampleRateValue * baudrate / SercomClock);
    // 65535 - 65535 * (sampleRateValue * baudrate / SercomClock));
    // sercom->USART.BAUD.reg = 65535.0f * ( 1.0f - (float)(sampleRateValue) * (float)(baudrate) / (float)(SercomCoreClock));  // this pulls in 3KB of floating point math code
    // make numerator much larger than denominator so result is integer (avoid floating point).
    uint64_t numerator = ((sampleRateValue * (uint64_t)baudrate) << 32); // 32 bits of shifting ensures no loss of precision.
    uint64_t ratio = divide64(numerator, SercomClock);
    uint64_t scale = ((uint64_t)1 << 32) - ratio;
    uint64_t baudValue = (65536 * scale) >> 32;
    sercom->USART.BAUD.reg = baudValue;
#endif
    // Asynchronous fractional mode (Table 24-2 in datasheet)
    //   BAUD = fref / (sampleRateValue * fbaud)
    // (multiply by 8, to calculate fractional piece)
    uint32_t baudTimes8 = (SercomClock * 8) / (sampleRateValue * baudrate);

    sercom->USART.BAUD.FRAC.FP   = (baudTimes8 % 8);
    sercom->USART.BAUD.FRAC.BAUD = (baudTimes8 / 8);
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

  // Enable Transceiver and Receiver
  sercom->USART.CTRLB.reg |= SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN ;
}

void SERCOM::resetUART()
{
  // Start the Software Reset
  sercom->USART.CTRLA.bit.SWRST = 1 ;

  while ( sercom->USART.CTRLA.bit.SWRST || sercom->USART.SYNCBUSY.bit.SWRST )
  {
    // Wait for both bits Software Reset from CTRLA and SYNCBUSY coming back to 0
  }
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
  // Skip checking transmission completion if data register is empty
  if(isDataRegisterEmptyUART())
    return;

  // Wait for transmission to complete
  while(!sercom->USART.INTFLAG.bit.TXC);
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

bool SERCOM::isUARTError()
{
  return sercom->USART.INTFLAG.bit.ERROR;
}

void SERCOM::acknowledgeUARTError()
{
  sercom->USART.INTFLAG.bit.ERROR = 1;
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
  // Wait for data register to be empty
  while(!isDataRegisterEmptyUART());

  //Put data into DATA register
  sercom->USART.DATA.reg = (uint16_t)data;
  return 1;
}

void SERCOM::enableDataRegisterEmptyInterruptUART()
{
  sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
}

void SERCOM::disableDataRegisterEmptyInterruptUART()
{
  sercom->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
}

/*	=========================
 *	===== Sercom SPI
 *	=========================
*/
void SERCOM::initSPI(SercomSpiTXPad mosi, SercomRXPad miso, SercomSpiCharSize charSize, SercomDataOrder dataOrder)
{
  resetSPI();
  initClockNVIC();

  //Setting the CTRLA register
  sercom->SPI.CTRLA.reg =	SERCOM_SPI_CTRLA_MODE( SPI_MASTER_OPERATION ) |
                          SERCOM_SPI_CTRLA_DOPO(mosi) |
                          SERCOM_SPI_CTRLA_DIPO(miso) |
                          dataOrder << SERCOM_SPI_CTRLA_DORD_Pos;

  //Setting the CTRLB register
  sercom->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE(charSize) |
                          SERCOM_SPI_CTRLB_RXEN;	//Active the SPI receiver.


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
  sercom->SPI.CTRLA.reg |=	( cpha << SERCOM_SPI_CTRLA_CPHA_Pos ) |
                            ( cpol << SERCOM_SPI_CTRLA_CPOL_Pos );

  //Synchronous arithmetic
  sercom->SPI.BAUD.reg = calculateBaudrateSynchronous(baudrate);
}

void SERCOM::resetSPI()
{
  //Setting the Software Reset bit to 1
  sercom->SPI.CTRLA.bit.SWRST = 1;

  //Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
  while(sercom->SPI.CTRLA.bit.SWRST || sercom->SPI.SYNCBUSY.bit.SWRST);
}

void SERCOM::enableSPI()
{
  //Setting the enable bit to 1
  sercom->SPI.CTRLA.bit.ENABLE = 1;

  while(sercom->SPI.SYNCBUSY.bit.ENABLE)
  {
    //Waiting then enable bit from SYNCBUSY is equal to 0;
  }
}

void SERCOM::disableSPI()
{
  while(sercom->SPI.SYNCBUSY.bit.ENABLE)
  {
    //Waiting then enable bit from SYNCBUSY is equal to 0;
  }

  //Setting the enable bit to 0
  sercom->SPI.CTRLA.bit.ENABLE = 0;
}

void SERCOM::setDataOrderSPI(SercomDataOrder dataOrder)
{
  //Register enable-protected
  disableSPI();

  sercom->SPI.CTRLA.bit.DORD = dataOrder;

  enableSPI();
}

SercomDataOrder SERCOM::getDataOrderSPI()
{
  return (sercom->SPI.CTRLA.bit.DORD ? LSB_FIRST : MSB_FIRST);
}

void SERCOM::setBaudrateSPI(uint8_t divider)
{
  //Can't divide by 0
  if(divider == 0)
    return;

  //Register enable-protected
  disableSPI();

  sercom->SPI.BAUD.reg = calculateBaudrateSynchronous( SercomClock / divider );

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

uint8_t SERCOM::transferDataSPI(uint8_t data)
{
  sercom->SPI.DATA.bit.DATA = data; // Writing data into Data register

  while( sercom->SPI.INTFLAG.bit.RXC == 0 )
  {
    // Waiting Complete Reception
  }

  return sercom->SPI.DATA.bit.DATA;  // Reading data
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

//bool SERCOM::isTransmitCompleteSPI()
//{
//	//TXC : Transmit complete
//	return sercom->SPI.INTFLAG.bit.TXC;
//}
//
//bool SERCOM::isReceiveCompleteSPI()
//{
//	//RXC : Receive complete
//	return sercom->SPI.INTFLAG.bit.RXC;
//}

uint32_t SERCOM::calculateBaudrateSynchronous(uint32_t baudrate)
{
  return ((SercomClock / (2 * baudrate)) - 1);
}


/*	=========================
 *	===== Sercom WIRE
 *	=========================
 */
void SERCOM::resetWIRE()
{
  //I2CM OR I2CS, no matter SWRST is the same bit.

  //Setting the Software bit to 1
  sercom->I2CM.CTRLA.bit.SWRST = 1;

  //Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
  while(sercom->I2CM.CTRLA.bit.SWRST || sercom->I2CM.SYNCBUSY.bit.SWRST);
}

void SERCOM::enableWIRE()
{
  // I2C Master and Slave modes share the ENABLE bit function.

  // Enable the I2C master mode
  sercom->I2CM.CTRLA.bit.ENABLE = 1 ;

  while ( sercom->I2CM.SYNCBUSY.bit.ENABLE != 0 )
  {
    // Waiting the enable bit from SYNCBUSY is equal to 0;
  }

  // Setting bus idle mode
  sercom->I2CM.STATUS.bit.BUSSTATE = 1 ;

  while ( sercom->I2CM.SYNCBUSY.bit.SYSOP != 0 )
  {
    // Wait the SYSOP bit from SYNCBUSY coming back to 0
  }
}

void SERCOM::disableWIRE()
{
  // I2C Master and Slave modes share the ENABLE bit function.

  // Enable the I2C master mode
  sercom->I2CM.CTRLA.bit.ENABLE = 0 ;

  while ( sercom->I2CM.SYNCBUSY.bit.ENABLE != 0 )
  {
    // Waiting the enable bit from SYNCBUSY is equal to 0;
  }
}

void SERCOM::initSlaveWIRE( uint8_t ucAddress, bool enableGeneralCall )
{
  // Initialize the peripheral clock and interruption
  initClockNVIC() ;
  resetWIRE() ;

  // Set slave mode
  sercom->I2CS.CTRLA.bit.MODE = I2C_SLAVE_OPERATION;

  sercom->I2CS.ADDR.reg = SERCOM_I2CS_ADDR_ADDR( ucAddress & 0x7Ful ) | // 0x7F, select only 7 bits
                          SERCOM_I2CS_ADDR_ADDRMASK( 0x00ul );          // 0x00, only match exact address
  if (enableGeneralCall) {
    sercom->I2CS.ADDR.reg |= SERCOM_I2CS_ADDR_GENCEN;                   // enable general call (address 0x00)
  }

  // Set the interrupt register
  sercom->I2CS.INTENSET.reg = SERCOM_I2CS_INTENSET_PREC |   // Stop
                              SERCOM_I2CS_INTENSET_AMATCH | // Address Match
                              SERCOM_I2CS_INTENSET_DRDY ;   // Data Ready

  while ( sercom->I2CM.SYNCBUSY.bit.SYSOP != 0 )
  {
    // Wait the SYSOP bit from SYNCBUSY to come back to 0
  }
}

void SERCOM::initMasterWIRE( uint32_t baudrate )
{
  // Initialize the peripheral clock and interruption
  initClockNVIC() ;

  resetWIRE() ;

  // Set master mode and enable SCL Clock Stretch mode (stretch after ACK bit)
  sercom->I2CM.CTRLA.reg =  SERCOM_I2CM_CTRLA_MODE( I2C_MASTER_OPERATION )/* |
                            SERCOM_I2CM_CTRLA_SCLSM*/ ;

  // Enable Smart mode and Quick Command
  //sercom->I2CM.CTRLB.reg =  SERCOM_I2CM_CTRLB_SMEN /*| SERCOM_I2CM_CTRLB_QCEN*/ ;


  // Enable all interrupts
//  sercom->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_MB | SERCOM_I2CM_INTENSET_SB | SERCOM_I2CM_INTENSET_ERROR ;

  // Synchronous arithmetic baudrate
  sercom->I2CM.BAUD.bit.BAUD = SercomClock / ( 2 * baudrate) - 5 - (((SercomClock / 1000000) * WIRE_RISE_TIME_NANOSECONDS) / (2 * 1000));
}

void SERCOM::prepareNackBitWIRE( void )
{
  if(isMasterWIRE()) {
    // Send a NACK
    sercom->I2CM.CTRLB.bit.ACKACT = 1;
  } else {
    sercom->I2CS.CTRLB.bit.ACKACT = 1;
  }
}

void SERCOM::prepareAckBitWIRE( void )
{
  if(isMasterWIRE()) {
    // Send an ACK
    sercom->I2CM.CTRLB.bit.ACKACT = 0;
  } else {
    sercom->I2CS.CTRLB.bit.ACKACT = 0;
  }
}

void SERCOM::prepareCommandBitsWire(uint8_t cmd)
{
  if(isMasterWIRE()) {
    sercom->I2CM.CTRLB.bit.CMD = cmd;

    while(sercom->I2CM.SYNCBUSY.bit.SYSOP)
    {
      // Waiting for synchronization
    }
  } else {
    sercom->I2CS.CTRLB.bit.CMD = cmd;
  }
}

bool SERCOM::startTransmissionWIRE(uint8_t address, SercomWireReadWriteFlag flag)
{
  // 7-bits address + 1-bits R/W
  address = (address << 0x1ul) | flag;

  // Wait idle or owner bus mode
  while ( !isBusIdleWIRE() && !isBusOwnerWIRE() );

  // Send start and address
  sercom->I2CM.ADDR.bit.ADDR = address;

  // Address Transmitted
  if ( flag == WIRE_WRITE_FLAG ) // Write mode
  {
    while( !sercom->I2CM.INTFLAG.bit.MB )
    {
      // Wait transmission complete
    }
  }
  else  // Read mode
  {
    while( !sercom->I2CM.INTFLAG.bit.SB )
    {
        // If the slave NACKS the address, the MB bit will be set.
        // In that case, send a stop condition and return false.
        if (sercom->I2CM.INTFLAG.bit.MB) {
            sercom->I2CM.CTRLB.bit.CMD = 3; // Stop condition
            return false;
        }
      // Wait transmission complete
    }

    // Clean the 'Slave on Bus' flag, for further usage.
    //sercom->I2CM.INTFLAG.bit.SB = 0x1ul;
  }


  //ACK received (0: ACK, 1: NACK)
  if(sercom->I2CM.STATUS.bit.RXNACK)
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool SERCOM::sendDataMasterWIRE(uint8_t data)
{
  //Send data
  sercom->I2CM.DATA.bit.DATA = data;

  //Wait transmission successful
  while(!sercom->I2CM.INTFLAG.bit.MB) {

    // If a bus error occurs, the MB bit may never be set.
    // Check the bus error bit and bail if it's set.
    if (sercom->I2CM.STATUS.bit.BUSERR) {
      return false;
    }
  }

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

  //Problems on line? nack received?
  if(!sercom->I2CS.INTFLAG.bit.DRDY || sercom->I2CS.STATUS.bit.RXNACK)
    return false;
  else
    return true;
}

bool SERCOM::isMasterWIRE( void )
{
  return sercom->I2CS.CTRLA.bit.MODE == I2C_MASTER_OPERATION;
}

bool SERCOM::isSlaveWIRE( void )
{
  return sercom->I2CS.CTRLA.bit.MODE == I2C_SLAVE_OPERATION;
}

bool SERCOM::isBusIdleWIRE( void )
{
  return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_IDLE_STATE;
}

bool SERCOM::isBusOwnerWIRE( void )
{
  return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_OWNER_STATE;
}

bool SERCOM::isDataReadyWIRE( void )
{
  return sercom->I2CS.INTFLAG.bit.DRDY;
}

bool SERCOM::isStopDetectedWIRE( void )
{
  return sercom->I2CS.INTFLAG.bit.PREC;
}

bool SERCOM::isRestartDetectedWIRE( void )
{
  return sercom->I2CS.STATUS.bit.SR;
}

bool SERCOM::isAddressMatch( void )
{
  return sercom->I2CS.INTFLAG.bit.AMATCH;
}

bool SERCOM::isMasterReadOperationWIRE( void )
{
  return sercom->I2CS.STATUS.bit.DIR;
}

bool SERCOM::isRXNackReceivedWIRE( void )
{
  return sercom->I2CM.STATUS.bit.RXNACK;
}

int SERCOM::availableWIRE( void )
{
  if(isMasterWIRE())
    return sercom->I2CM.INTFLAG.bit.SB;
  else
    return sercom->I2CS.INTFLAG.bit.DRDY;
}

uint8_t SERCOM::readDataWIRE( void )
{
  if(isMasterWIRE())
  {
    while( sercom->I2CM.INTFLAG.bit.SB == 0 )
    {
      // Waiting complete receive
    }

    return sercom->I2CM.DATA.bit.DATA ;
  }
  else
  {
    return sercom->I2CS.DATA.reg ;
  }
}


void SERCOM::initClockNVIC( void )
{
  uint8_t clockId = 0;
  #if (SAMD51)
    IRQn_Type IdNvic0=PendSV_IRQn ; // Dummy init to intercept potential error later
    IRQn_Type IdNvic1=PendSV_IRQn ;
    IRQn_Type IdNvic2=PendSV_IRQn ;
    IRQn_Type IdNvic3=PendSV_IRQn ;
  #else
    IRQn_Type IdNvic=PendSV_IRQn ; // Dummy init to intercept potential error later
  #endif

  if(sercom == SERCOM0)
  {
    clockId = GCM_SERCOM0_CORE;
    #if (SAMD51)
      IdNvic0 = SERCOM0_0_IRQn;
      IdNvic1 = SERCOM0_1_IRQn;
      IdNvic2 = SERCOM0_2_IRQn;
      IdNvic3 = SERCOM0_3_IRQn;
    #else
      IdNvic = SERCOM0_IRQn;
    #endif
  }
  else if(sercom == SERCOM1)
  {
    clockId = GCM_SERCOM1_CORE;
    #if (SAMD51)
      IdNvic0 = SERCOM1_0_IRQn;
      IdNvic1 = SERCOM1_1_IRQn;
      IdNvic2 = SERCOM1_2_IRQn;
      IdNvic3 = SERCOM1_3_IRQn;
    #else
      IdNvic = SERCOM1_IRQn;
    #endif
  }
#if !(SAMD11C14)
  else if(sercom == SERCOM2)
  {
    clockId = GCM_SERCOM2_CORE;
    #if (SAMD51)
      IdNvic0 = SERCOM2_0_IRQn;
      IdNvic1 = SERCOM2_1_IRQn;
      IdNvic2 = SERCOM2_2_IRQn;
      IdNvic3 = SERCOM2_3_IRQn;
    #else
      IdNvic = SERCOM2_IRQn;
    #endif
  }
#endif
#if !(SAMD11_SERIES)
  else if(sercom == SERCOM3)
  {
    clockId = GCM_SERCOM3_CORE;
    #if (SAMD51)
      IdNvic0 = SERCOM3_0_IRQn;
      IdNvic1 = SERCOM3_1_IRQn;
      IdNvic2 = SERCOM3_2_IRQn;
      IdNvic3 = SERCOM3_3_IRQn;
    #else
      IdNvic = SERCOM3_IRQn;
    #endif
  }
#endif
#if !(SAMD11_SERIES) && !(SAMD21E) && !(SAMC21E)
  else if(sercom == SERCOM4)
  {
    clockId = GCM_SERCOM4_CORE;
    #if (SAMD51)
      IdNvic0 = SERCOM4_0_IRQn;
      IdNvic1 = SERCOM4_1_IRQn;
      IdNvic2 = SERCOM4_2_IRQn;
      IdNvic3 = SERCOM4_3_IRQn;
    #else
      IdNvic = SERCOM4_IRQn;
    #endif
  }
  else if(sercom == SERCOM5)
  {
    clockId = GCM_SERCOM5_CORE;
    #if (SAMD51)
      IdNvic0 = SERCOM5_0_IRQn;
      IdNvic1 = SERCOM5_1_IRQn;
      IdNvic2 = SERCOM5_2_IRQn;
      IdNvic3 = SERCOM5_3_IRQn;
    #else
      IdNvic = SERCOM5_IRQn;
    #endif
  }
#endif
#if (SAMD51N || SAMD51P)
  else if(sercom == SERCOM6)
  {
    clockId = GCM_SERCOM6_CORE;
    IdNvic0 = SERCOM6_0_IRQn;
    IdNvic1 = SERCOM6_1_IRQn;
    IdNvic2 = SERCOM6_2_IRQn;
    IdNvic3 = SERCOM6_3_IRQn;
  }
  else if(sercom == SERCOM7)
  {
    clockId = GCM_SERCOM7_CORE;
    IdNvic0 = SERCOM7_0_IRQn;
    IdNvic1 = SERCOM7_1_IRQn;
    IdNvic2 = SERCOM7_2_IRQn;
    IdNvic3 = SERCOM7_3_IRQn;
  }
#endif

#if (SAMD51)
  if ( IdNvic0 == PendSV_IRQn || IdNvic1 == PendSV_IRQn || IdNvic2 == PendSV_IRQn || IdNvic3 == PendSV_IRQn )
#else
  if ( IdNvic == PendSV_IRQn )
#endif
  {
    // We got a problem here
    return ;
  }

  // Setting NVIC
#if (SAMD51)
  NVIC_EnableIRQ(IdNvic0);
  NVIC_SetPriority (IdNvic0, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority */
  NVIC_EnableIRQ(IdNvic1);
  NVIC_SetPriority (IdNvic1, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority */
  NVIC_EnableIRQ(IdNvic2);
  NVIC_SetPriority (IdNvic2, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority */
  NVIC_EnableIRQ(IdNvic3);
  NVIC_SetPriority (IdNvic3, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority */
#else
  NVIC_EnableIRQ(IdNvic);
  NVIC_SetPriority (IdNvic, SERCOM_NVIC_PRIORITY);  /* set Priority */
#endif

  //Setting clock
#if (SAMD21 || SAMD11)
  GCLK->CLKCTRL.reg = ( GCLK_CLKCTRL_ID( clockId ) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN );
  SercomClock = SystemCoreClock;
  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );
#elif (SAML21 || SAMC21)
  GCLK->PCHCTRL[clockId].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0 );
  SercomClock = SystemCoreClock;
  while ( (GCLK->PCHCTRL[clockId].reg & GCLK_PCHCTRL_CHEN) != GCLK_PCHCTRL_CHEN );      // wait for sync
#elif (SAMD51)
  GCLK->PCHCTRL[clockId].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK10 );  // use 96MHz clock (100MHz max for SERCOM) from GCLK10, which was setup in startup.c
  SercomClock = 96000000ul;
  while ( (GCLK->PCHCTRL[clockId].reg & GCLK_PCHCTRL_CHEN) != GCLK_PCHCTRL_CHEN );      // wait for sync
#else
  #error "SERCOM.cpp: Unsupported chip"
#endif
}
