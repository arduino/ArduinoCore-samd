/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.

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

#include "board_driver_i2c.h"

#ifdef CONFIGURE_PMIC

/*- Definitions -------------------------------------------------------------*/
#define I2C_SERCOM            SERCOM0
#define I2C_SERCOM_GCLK_ID    GCLK_CLKCTRL_ID_SERCOM0_CORE_Val
#define I2C_SERCOM_CLK_GEN    0
#define I2C_SERCOM_APBCMASK   PM_APBCMASK_SERCOM0

static uint8_t txBuffer[2];
uint8_t rxBuffer[1];
static uint8_t txBufferLen = 0;
static uint8_t rxBufferLen = 0;
static uint8_t txAddress;

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

typedef enum
{
  I2C_SLAVE_OPERATION = 0x4u,
  I2C_MASTER_OPERATION = 0x5u
} SercomI2CMode;

static inline void pin_set_peripheral_function(uint32_t pinmux)
{
    /* the variable pinmux consist of two components:
        31:16 is a pad, wich includes:
            31:21 : port information 0->PORTA, 1->PORTB
            20:16 : pin 0-31
        15:00 pin multiplex information
        there are defines for pinmux like: PINMUX_PA09D_SERCOM2_PAD1 
    */
    uint16_t pad = pinmux >> 16;    // get pad (port+pin)
    uint8_t port = pad >> 5;        // get port
    uint8_t pin  = pad & 0x1F;      // get number of pin - no port information anymore
    
    PORT->Group[port].PINCFG[pin].bit.PMUXEN =1;
    
    /* each pinmux register is for two pins! with pin/2 you can get the index of the needed pinmux register
       the p mux resiter is 8Bit   (7:4 odd pin; 3:0 evan bit)  */
    // reset pinmux values.                             VV shift if pin is odd (if evan:  (4*(pin & 1))==0  )
    PORT->Group[port].PMUX[pin/2].reg &= ~( 0xF << (4*(pin & 1)) );
                    //          
    // set new values
    PORT->Group[port].PMUX[pin/2].reg |=  ( (uint8_t)( (pinmux&0xFFFF) <<(4*(pin&1)) ) ); 
}

static inline void initClockNVIC( void )
{
  //Setting clock
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( I2C_SERCOM_GCLK_ID ) | // Generic Clock 0 (SERCOMx)
                      GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is source
                      GCLK_CLKCTRL_CLKEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;
}

static inline bool isBusIdleWIRE( void )
{
  return I2C_SERCOM->I2CM.STATUS.bit.BUSSTATE == WIRE_IDLE_STATE;
}

static inline bool isBusOwnerWIRE( void )
{
  return I2C_SERCOM->I2CM.STATUS.bit.BUSSTATE == WIRE_OWNER_STATE;
}

static inline bool isDataReadyWIRE( void )
{
  return I2C_SERCOM->I2CS.INTFLAG.bit.DRDY;
}

static inline bool isStopDetectedWIRE( void )
{
  return I2C_SERCOM->I2CS.INTFLAG.bit.PREC;
}

static inline bool isRestartDetectedWIRE( void )
{
  return I2C_SERCOM->I2CS.STATUS.bit.SR;
}

static inline bool isAddressMatch( void )
{
  return I2C_SERCOM->I2CS.INTFLAG.bit.AMATCH;
}

static inline bool isMasterReadOperationWIRE( void )
{
  return I2C_SERCOM->I2CS.STATUS.bit.DIR;
}

static inline bool isRXNackReceivedWIRE( void )
{
  return I2C_SERCOM->I2CM.STATUS.bit.RXNACK;
}

static inline int availableWIRE( void )
{
  return I2C_SERCOM->I2CM.INTFLAG.bit.SB;
}

static inline uint8_t readDataWIRE( void )
{
  while( I2C_SERCOM->I2CM.INTFLAG.bit.SB == 0 )
  {
    // Waiting complete receive
  }

  return I2C_SERCOM->I2CM.DATA.bit.DATA ;
}

/*  =========================
 *  ===== Sercom WIRE
 *  =========================
 */
static inline void resetWIRE()
{
  //I2CM OR I2CS, no matter SWRST is the same bit.

  //Setting the Software bit to 1
  I2C_SERCOM->I2CM.CTRLA.bit.SWRST = 1;

  //Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
  while(I2C_SERCOM->I2CM.CTRLA.bit.SWRST || I2C_SERCOM->I2CM.SYNCBUSY.bit.SWRST);
}

static inline void enableWIRE()
{
  // I2C Master and Slave modes share the ENABLE bit function.

  // Enable the I²C master mode
  I2C_SERCOM->I2CM.CTRLA.bit.ENABLE = 1 ;

  while ( I2C_SERCOM->I2CM.SYNCBUSY.bit.ENABLE != 0 )
  {
    // Waiting the enable bit from SYNCBUSY is equal to 0;
  }

  // Setting bus idle mode
  I2C_SERCOM->I2CM.STATUS.bit.BUSSTATE = 1 ;

  while ( I2C_SERCOM->I2CM.SYNCBUSY.bit.SYSOP != 0 )
  {
    // Wait the SYSOP bit from SYNCBUSY coming back to 0
  }
}

static inline void disableWIRE()
{
  // I2C Master and Slave modes share the ENABLE bit function.

  // Enable the I²C master mode
  I2C_SERCOM->I2CM.CTRLA.bit.ENABLE = 0 ;

  while ( I2C_SERCOM->I2CM.SYNCBUSY.bit.ENABLE != 0 )
  {
    // Waiting the enable bit from SYNCBUSY is equal to 0;
  }
}

static inline void initMasterWIRE( uint32_t baudrate )
{
  // Initialize the peripheral clock and interruption
  initClockNVIC() ;

  //NVIC_EnableIRQ(SERCOM0_IRQn);
  //NVIC_SetPriority (SERCOM0_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority */

  resetWIRE() ;

  // Set master mode and enable SCL Clock Stretch mode (stretch after ACK bit)
  I2C_SERCOM->I2CM.CTRLA.reg =  SERCOM_I2CM_CTRLA_MODE( I2C_MASTER_OPERATION )/* |
                            SERCOM_I2CM_CTRLA_SCLSM*/ ;

  // Enable Smart mode and Quick Command
  //I2C_SERCOM->I2CM.I2CM.CTRLB.reg =  SERCOM_I2CM_CTRLB_SMEN /*| SERCOM_I2CM_CTRLB_QCEN*/ ;


  // Enable all interrupts
  //I2C_SERCOM->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_MB | SERCOM_I2CM_INTENSET_SB | SERCOM_I2CM_INTENSET_ERROR ;

  // Synchronous arithmetic baudrate
  I2C_SERCOM->I2CM.BAUD.bit.BAUD = 48000000 / ( 2 * baudrate) - 1 ;
}

static inline void prepareNackBitWIRE( void )
{
  // Send a NACK
  I2C_SERCOM->I2CM.CTRLB.bit.ACKACT = 1;
}

static inline void prepareAckBitWIRE( void )
{
  // Send an ACK
  I2C_SERCOM->I2CM.CTRLB.bit.ACKACT = 0;
}

static inline void prepareCommandBitsWire(uint8_t cmd)
{
  I2C_SERCOM->I2CM.CTRLB.bit.CMD = cmd;

  while(I2C_SERCOM->I2CM.SYNCBUSY.bit.SYSOP)
  {
    // Waiting for synchronization
  }
}

static inline bool startTransmissionWIRE(uint8_t address, SercomWireReadWriteFlag flag)
{
  // 7-bits address + 1-bits R/W
  address = (address << 0x1ul) | flag;


  // Wait idle or owner bus mode
  while ( !isBusIdleWIRE() && !isBusOwnerWIRE() );

  // Send start and address
  I2C_SERCOM->I2CM.ADDR.bit.ADDR = address;

  // Address Transmitted
  if ( flag == WIRE_WRITE_FLAG ) // Write mode
  {
    while( !I2C_SERCOM->I2CM.INTFLAG.bit.MB )
    {
      // Wait transmission complete
    }
  }
  else  // Read mode
  {
    while( !I2C_SERCOM->I2CM.INTFLAG.bit.SB )
    {
        // If the slave NACKS the address, the MB bit will be set.
        // In that case, send a stop condition and return false.
        if (I2C_SERCOM->I2CM.INTFLAG.bit.MB) {
            I2C_SERCOM->I2CM.CTRLB.bit.CMD = 3; // Stop condition
            return false;
        }
      // Wait transmission complete
    }

    // Clean the 'Slave on Bus' flag, for further usage.
    //I2C_SERCOM->I2CM.I2CM.INTFLAG.bit.SB = 0x1ul;
  }


  //ACK received (0: ACK, 1: NACK)
  if(I2C_SERCOM->I2CM.STATUS.bit.RXNACK)
  {
    return false;
  }
  else
  {
    return true;
  }
}

static inline bool sendDataMasterWIRE(uint8_t data)
{
  //Send data
  I2C_SERCOM->I2CM.DATA.bit.DATA = data;

  //Wait transmission successful
  while(!I2C_SERCOM->I2CM.INTFLAG.bit.MB) {

    // If a bus error occurs, the MB bit may never be set.
    // Check the bus error bit and bail if it's set.
    if (I2C_SERCOM->I2CM.STATUS.bit.BUSERR) {
      return false;
    }
  }

  //Problems on line? nack received?
  if(I2C_SERCOM->I2CM.STATUS.bit.RXNACK)
    return false;
  else
    return true;
}

void i2c_init(uint32_t baud) {
  //Master Mode
  initMasterWIRE(baud);
  enableWIRE();

  pin_set_peripheral_function(PINMUX_PA08C_SERCOM0_PAD0);
  pin_set_peripheral_function(PINMUX_PA09C_SERCOM0_PAD1);
}

void i2c_end() {
  disableWIRE();
}

uint8_t i2c_requestFrom(uint8_t address, uint8_t quantity, bool stopBit)
{
  if(quantity == 0)
  {
    return 0;
  }

  uint8_t byteRead = 0;

  rxBufferLen = 0;;

  if(startTransmissionWIRE(address, WIRE_READ_FLAG))
  {
    // Read first data
    rxBuffer[rxBufferLen++] = readDataWIRE();

    // Connected to slave
    for (byteRead = 1; byteRead < quantity; ++byteRead)
    {
      prepareAckBitWIRE();                          // Prepare Acknowledge
      prepareCommandBitsWire(WIRE_MASTER_ACT_READ); // Prepare the ACK command for the slave
      rxBuffer[rxBufferLen++] = readDataWIRE();          // Read data and send the ACK
    }
    prepareNackBitWIRE();                           // Prepare NACK to stop slave transmission
    //I2C_SERCOM->I2CM.readDataWIRE();                               // Clear data register to send NACK

    if (stopBit)
    {
      prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);   // Send Stop
    }
  }

  return byteRead;
}

void i2c_beginTransmission(uint8_t address) {
  // save address of target and clear buffer
  txAddress = address;
  txBufferLen = 0;
}

// Errors:
//  0 : Success
//  1 : Data too long
//  2 : NACK on transmit of address
//  3 : NACK on transmit of data
//  4 : Other error
uint8_t i2c_endTransmission(bool stopBit)
{

  // Start I2C transmission
  if ( !startTransmissionWIRE( txAddress, WIRE_WRITE_FLAG ) )
  {
    prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
    return 2 ;  // Address error
  }

  // Send all buffer
  int tempLen = txBufferLen;
  while( txBufferLen > 0 )
  {
    // Trying to send data
    if ( !sendDataMasterWIRE( txBuffer[tempLen-txBufferLen] ) )
    {
      prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
      return 3 ;  // Nack or error
    } else {
      txBufferLen--;
    }
  }
  
  if (stopBit)
  {
    prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
  }   

  return 0;
}

uint8_t i2c_write(uint8_t ucData)
{
  txBuffer[txBufferLen++] = ucData ;
  return 1 ;
}

#endif
