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
#define I2C_SERCOM            SERCOM2
#define I2C_SERCOM_PMUX       PORT_PMUX_PMUXE_D_Val
#define I2C_SERCOM_GCLK_ID    SERCOM0_GCLK_ID_CORE
#define I2C_SERCOM_CLK_GEN    0
#define I2C_SERCOM_APBCMASK   PM_APBCMASK_SERCOM0

#define T_RISE                215e-9 // Depends on the board, actually

enum
{
  I2C_TRANSFER_WRITE = 0,
  I2C_TRANSFER_READ  = 1,
};

enum
{
  I2C_PINS_SDA = (1 << 0),
  I2C_PINS_SCL = (1 << 1),
};

#define F_CPU   48000000

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

//-----------------------------------------------------------------------------
void i2c_init(int freq)
{

  PM->APBCMASK.reg |= I2C_SERCOM_APBCMASK;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(I2C_SERCOM_GCLK_ID) |
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(I2C_SERCOM_CLK_GEN);

  I2C_SERCOM->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_SWRST;
  while (I2C_SERCOM->I2CM.CTRLA.reg & SERCOM_I2CM_CTRLA_SWRST);

  I2C_SERCOM->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN;
  while (I2C_SERCOM->I2CM.SYNCBUSY.reg);

  I2C_SERCOM->I2CM.BAUD.reg = SERCOM_I2CM_BAUD_BAUD(48);
  while (I2C_SERCOM->I2CM.SYNCBUSY.reg);

  I2C_SERCOM->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_ENABLE |
      SERCOM_I2CM_CTRLA_MODE_I2C_MASTER |
      SERCOM_I2CM_CTRLA_SDAHOLD(3);
  while (I2C_SERCOM->I2CM.SYNCBUSY.reg);

  I2C_SERCOM->I2CM.STATUS.reg |= SERCOM_I2CM_STATUS_BUSSTATE(1);

  pin_set_peripheral_function(PINMUX_PA08C_SERCOM0_PAD0);
  pin_set_peripheral_function(PINMUX_PA09C_SERCOM0_PAD1);
}

//-----------------------------------------------------------------------------
bool i2c_start(int addr)
{
  I2C_SERCOM->I2CM.ADDR.reg = addr;

  while (0 == (I2C_SERCOM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB) &&
         0 == (I2C_SERCOM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_SB));

  if (I2C_SERCOM->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK)
  {
    I2C_SERCOM->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(3);
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool i2c_stop(void)
{
  if ((I2C_SERCOM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB) ||
      (I2C_SERCOM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_SB))
  {
    I2C_SERCOM->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(3);
  }

  return true;
}

//-----------------------------------------------------------------------------
bool i2c_read_byte(uint8_t *byte, bool last)
{
  while (0 == (I2C_SERCOM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_SB));

  if (last)
    I2C_SERCOM->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_ACKACT | SERCOM_I2CM_CTRLB_CMD(3);
  else
    I2C_SERCOM->I2CM.CTRLB.reg &= ~SERCOM_I2CM_CTRLB_ACKACT;

  *byte = I2C_SERCOM->I2CM.DATA.reg;

  return true;
}

//-----------------------------------------------------------------------------
bool i2c_write_byte(uint8_t byte)
{
  I2C_SERCOM->I2CM.DATA.reg = byte;

  while (0 == (I2C_SERCOM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB));

  if (I2C_SERCOM->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK)
  {
    I2C_SERCOM->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(3);
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool i2c_busy(int addr)
{
  bool busy;

  I2C_SERCOM->I2CM.ADDR.reg = addr | I2C_TRANSFER_WRITE;

  while (0 == (I2C_SERCOM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB));

  busy = (0 != (I2C_SERCOM->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK));

  I2C_SERCOM->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(3);

  return busy;
}

uint8_t readRegister(uint8_t startAddress) {

    uint8_t DATA = 0;
    i2c_start(PMIC_ADDRESS);
    i2c_write_byte(startAddress);
    i2c_stop();

    i2c_start(PMIC_ADDRESS);
    i2c_read_byte(&DATA, true);
    return DATA;
}


/*******************************************************************************
 * Function Name  :
 * Description    :
 * Input          :
 * Return         :
 *******************************************************************************/
void writeRegister(uint8_t address, uint8_t DATA) {

    i2c_start(PMIC_ADDRESS);
    i2c_write_byte(address);
    i2c_write_byte(DATA);
    i2c_stop();
}

bool disableWatchdog(void) {

    uint8_t DATA = readRegister(CHARGE_TIMER_CONTROL_REGISTER);
    writeRegister(CHARGE_TIMER_CONTROL_REGISTER, (DATA & 0b11001110));
    return 1;
}

bool setInputVoltageLimit(uint16_t voltage) {

    uint8_t DATA = readRegister(INPUT_SOURCE_REGISTER);
    uint8_t mask = DATA & 0b10000111;

    switch(voltage) {

        case 3880:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b00000000));
        break;

        case 3960:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b00001000));
        break;

        case 4040:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b00010000));
        break;

        case 4120:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b00011000));
        break;

        case 4200:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b00100000));
        break;

        case 4280:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b00101000));
        break;

        case 4360:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b00110000));
        break;

        case 4440:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b00111000));
        break;

        case 4520:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b01000000));
        break;

        case 4600:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b01001000));
        break;

        case 4680:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b01010000));
        break;

        case 4760:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b01011000));
        break;

        case 4840:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b01100000));
        break;

        case 4920:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b01101000));
        break;

        case 5000:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b01110000));
        break;

        case 5080:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b01111000));
        break;

        default:
        return 0; // return error since the value passed didn't match
    }

    return 1; // value was written successfully
}

bool setInputCurrentLimit(uint16_t current) {


    uint8_t DATA = readRegister(INPUT_SOURCE_REGISTER);
    uint8_t mask = DATA & 0b11111000;

    switch (current) {

        case 100:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b00000000));
        break;

        case 150:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b00000001));
        break;

        case 500:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b00000010));
        break;

        case 900:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b00000011));
        break;

        case 1200:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b00000100));
        break;

        case 1500:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b00000101));
        break;

        case 2000:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b00000110));
        break;

        case 3000:
        writeRegister(INPUT_SOURCE_REGISTER, (mask | 0b00000111));
        break;

        default:
        return 0; // return error since the value passed didn't match
    }

    return 1; // value was written successfully
}

bool setChargeCurrent(bool bit7, bool bit6, bool bit5, bool bit4, bool bit3, bool bit2) {

    uint8_t current = 0;
    if (bit7) current = current | 0b10000000;
    if (bit6) current = current | 0b01000000;
    if (bit5) current = current | 0b00100000;
    if (bit4) current = current | 0b00010000;
    if (bit3) current = current | 0b00001000;
    if (bit2) current = current | 0b00000100;

    uint8_t DATA = readRegister(CHARGE_CURRENT_CONTROL_REGISTER);
    uint8_t mask = DATA & 0b00000001;
    writeRegister(CHARGE_CURRENT_CONTROL_REGISTER, current | mask);
    return 1;
}

bool setChargeVoltage(uint16_t voltage) {

    uint8_t DATA = readRegister(CHARGE_VOLTAGE_CONTROL_REGISTER);
    uint8_t mask = DATA & 0b000000011;

    switch (voltage) {

        case 4112:
        writeRegister(CHARGE_VOLTAGE_CONTROL_REGISTER, (mask | 0b10011000));
        break;

        case 4208:
        writeRegister(CHARGE_VOLTAGE_CONTROL_REGISTER, (mask | 0b10110000));
        break;

        default:
        return 0; // return error since the value passed didn't match
    }

    return 1; // value was written successfully
}

void apply_pmic_newdefaults()
{
  disableWatchdog();
  //disableDPDM();
  setInputVoltageLimit(4360); // default
  setInputCurrentLimit(900);     // 900mA
  setChargeCurrent(0,0,0,0,0,0); // 512mA
  setChargeVoltage(4112);        // 4.112V termination voltage
}

void configure_pmic() {
  i2c_init(100000);
  apply_pmic_newdefaults();
}

#endif