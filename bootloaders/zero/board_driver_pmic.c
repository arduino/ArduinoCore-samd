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
#include "board_driver_pmic.h"

#ifdef CONFIGURE_PMIC

extern uint8_t rxBuffer[1];

uint8_t readRegister(uint8_t reg) {
  i2c_beginTransmission(PMIC_ADDRESS);
  i2c_write(reg);
  i2c_endTransmission(true);

  i2c_requestFrom(PMIC_ADDRESS, 1, true);
  return rxBuffer[0];
}

uint8_t writeRegister(uint8_t reg, uint8_t data) {
  i2c_beginTransmission(PMIC_ADDRESS);
  i2c_write(reg);
  i2c_write(data);
  i2c_endTransmission(true);

  return 2;
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

bool disableCharge()
{
  uint8_t DATA = readRegister(POWERON_CONFIG_REGISTER);
  uint8_t mask = DATA & 0b11001111;

  writeRegister(POWERON_CONFIG_REGISTER, mask);

  return 1;
}

void apply_pmic_newdefaults()
{
  disableWatchdog();

  //disableDPDM();
  disableCharge();
  setInputVoltageLimit(4360); // default
  setInputCurrentLimit(2000);     // 2A
  setChargeCurrent(0,0,0,0,0,0); // 512mA
  setChargeVoltage(4112);        // 4.112V termination voltage
}

void configure_pmic()
{
  i2c_init(100000);
  apply_pmic_newdefaults();
}

#endif
