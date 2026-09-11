/*
  PMIC.cpp - initialization of Power Management ICs
  Copyright (c) 2020 Kevin P. Fleming.  All right reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "PMICClass.h"

PMICClass::PMICClass(void) {
}

PMICClass::~PMICClass(void) {
}


void PMICClass::init(SERCOM * sercom) {
  _sercom = sercom;
}

void PMICClass::setupBQ24195(bool batteryPresent, bool USBDetect) {
  BQ24195_REG00 reg00;
  reg00.IINLIM = _ilim; // input current limit 500 mA
  reg00.VINDPM = 0x00; // input voltage limit 3.88V
  reg00.EN_HIZ = 0x00; // disable

  writereg(BQ24195_ADDRESS, BQ24195_REG00_ADDRESS, (reg00.val));

  BQ24195_REG01 reg01;
  reg01.RSVD = 0x01;
  reg01.SYS_MIN = 0x05; // minimum system voltage 3.5V
  reg01.CHG_CONFIG_ENABLE = (batteryPresent ? 0x01 : 0x00); // battery charge enable/disable
  reg01.CHG_CONFIG_OTG = 0x00;
  reg01.WATCHDOG_TIMER_RESET = 0x01; // reset watchdog timer
  reg01.REGISTER_RESET = 0x00; // keep current register setting

  writereg(BQ24195_ADDRESS, BQ24195_REG01_ADDRESS, (reg01.val));

  BQ24195_REG05 reg05;
  reg05.RSVD = 0x00;
  reg05.CHG_TIMER = 0x01; // fast charge timer 8 hours
  reg05.EN_TIMER = (batteryPresent ? 0x01 : 0x00); // enable/disable charge safety timer
  reg05.WATCHDOG = 0x00; // disable watchdog timer to stay in host mode
  reg05.TERM_STAT = 0x00; // charge termination indicator match ITERM
  reg05.EN_TERM = 0x01; // enable charge termination

  writereg(BQ24195_ADDRESS, BQ24195_REG05_ADDRESS, (reg05.val));

  BQ24195_REG07 reg07;
  reg07.INT_MASK_BAT = 0x01; // INT on battery fault
  reg07.INT_MASK_CHG = 0x01; // INT on charge fault
  reg07.RSVD = 0x02;
  reg07.BATFET_DISABLE = (batteryPresent ? 0x00: 0x01); // battery FET enable/disable
  reg07.TMR2X_EN = 0x00; // safety timer not slowed by 2X
  reg07.DPDM_EN = 0x00; // D+/D- detection enable/disable

  writereg(BQ24195_ADDRESS, BQ24195_REG07_ADDRESS, (reg07.val));

  BQ24195_REG04 reg04;
  reg04.VRECHG = 0x2C;
  reg04.BATLOWV = 0x01;
  reg04.VREG = 0x01;
  writereg(BQ24195_ADDRESS, BQ24195_REG04_ADDRESS, (reg04.val));
}

void PMICClass::setupMP2629(bool batteryPresent) {
  // disable or enable battery bat feet
  MP2629_REG0A reg0A;
  uint8_t byte = readFrom(MP2629_ADDRESS, MP2629_REG0A_ADDRESS);
  reg0A.TDISC_L = byte & 0x03;
  reg0A.TDISC_H = (byte & 0x0C) >> 2;
  reg0A.SYSRST_SEL = (byte & 0x10) >> 5;
  reg0A.BATFET_DIS = (batteryPresent ? 0b0 : 0b1);
  reg0A.TMR2X_EN = (byte & 0x40) >> 6;
  reg0A.SW_FREQ = byte >> 7;
  writereg(MP2629_ADDRESS, MP2629_REG0A_ADDRESS, reg0A.val);
  delay(300);

  // disable watchdog timer reset
  byte = readFrom(MP2629_ADDRESS, MP2629_REG08_ADDRESS);
  MP2629_REG08 reg08;
  reg08.EN_TIMER = byte & 0x01;
  reg08.CHG_TMR = (byte & 0x06) >> 1;
  reg08.WATCHDOG_TIM_RST = 0;
  reg08.WATCHDOG = (byte & 0x30) >> 4;
  reg08.RSVD = (byte & 0x40) >> 6;
  reg08.EN_TIMER = byte >> 7;
  writereg(MP2629_ADDRESS, MP2629_REG08_ADDRESS, reg08.val);

  MP2629_REG0B reg0B;
  reg0B.RSVD = byte & 0x3F;
  reg0B.USB_DET_EN = 1;
  reg0B.INT_MASK = 1;
  writereg(MP2629_ADDRESS, MP2629_REG0B_ADDRESS, reg0B.val);

  // set ICC
  byte = readFrom(MP2629_ADDRESS, MP2629_REG05_ADDRESS);
  MP2629_REG05 reg05;
  reg05.ICC = 0x05;
  reg05.VBATT_PRE = byte >> 7;
  writereg(MP2629_ADDRESS, MP2629_REG05_ADDRESS, reg05.val);
  
  // set the vbat voltage to 4.1 V
  byte = readFrom(MP2629_ADDRESS, MP2629_REG07_ADDRESS);
  MP2629_REG07 reg07;
  reg07.VRECH = byte & 0x01;
  reg07.VBATT_REG = 0x46;//set vbat charge threshold to 4.1 V
  writereg(MP2629_ADDRESS, MP2629_REG07_ADDRESS, reg07.val);
}

bool PMICClass::isBatteryConnected() {
  // reset bat feet to default value
  MP2629_REG0A reg0A;
  uint8_t byte = readFrom(MP2629_ADDRESS, MP2629_REG0A_ADDRESS);
  reg0A.TDISC_L = byte & 0x03;
  reg0A.TDISC_H = (byte & 0x0C) >> 2;
  reg0A.SYSRST_SEL = (byte & 0x10) >> 5;
  reg0A.BATFET_DIS = 0;
  reg0A.TMR2X_EN = (byte & 0x40) >> 6;
  reg0A.SW_FREQ = byte >> 7;
  writereg(MP2629_ADDRESS, MP2629_REG0A_ADDRESS, reg0A.val);


  MP2629_REG03 reg03;
  // set MP2'S ADC in CONTINUOS acquision mode
  byte = readFrom(MP2629_ADDRESS, MP2629_REG03_ADDRESS);
  reg03.IIN_DSCHG = (byte & 0x07);
  reg03.VIN_DSCHG = (byte & 0x56) >> 3;
  reg03.ADC_RATE = 1;
  reg03.ADC_START = byte >> 7;
  writereg(MP2629_ADDRESS, MP2629_REG03_ADDRESS, reg03.val);
  delay(10);

  return readFrom(MP2629_ADDRESS, MP2629_REG12_ADDRESS) > 0;
}

bool PMICClass::isBQ24195() {
  return (readFrom(BQ24195_ADDRESS, BQ24195_REG0A_ADDRESS) == 0x23);
}

void PMICClass::setIlim(uint8_t ilim){
  _ilim = ilim;
}

uint8_t PMICClass::readFrom(uint8_t pmicAddress, uint8_t address) {
  uint8_t readedValue = 0;

  _sercom->startTransmissionWIRE(pmicAddress, WIRE_WRITE_FLAG);
  _sercom->sendDataMasterWIRE(address);
  _sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);

  if (_sercom->startTransmissionWIRE(pmicAddress, WIRE_READ_FLAG))
  {
    readedValue = _sercom->readDataWIRE();
    _sercom->prepareNackBitWIRE();
    _sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
  }
  return readedValue;
}

void PMICClass::writereg(uint8_t pmicAddress, uint8_t address, uint8_t value) {
  _sercom->startTransmissionWIRE(pmicAddress, WIRE_WRITE_FLAG );
  _sercom->sendDataMasterWIRE(address);
  _sercom->sendDataMasterWIRE(value);
  _sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
}

PMICClass PMICArduino;
