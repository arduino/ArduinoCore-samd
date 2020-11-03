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

#include "Arduino.h"

#ifdef USE_BQ24195L_PMIC

#include "PMIC/BQ24195.h"

#include "wiring_private.h"

void setupPMIC(SERCOM& sercom, bool batteryPresent, bool USBDetect) {
  BQ24195_REG00 reg00;
  reg00.IINLIM = 0b110; // input current limit 2A
  reg00.VINDPM = 0b0000; // input voltage limit 3.88V
  reg00.EN_HIZ = 0b0; // disable

  sercom.startTransmissionWIRE(BQ24195_ADDRESS, WIRE_WRITE_FLAG);
  sercom.sendDataMasterWIRE(BQ24195_REG00_ADDRESS);
  sercom.sendDataMasterWIRE(reg00.val);
  sercom.prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);

  BQ24195_REG01 reg01;
  reg01.RSVD = 0b1;
  reg01.SYS_MIN = 0b101; // minimum system voltage 3.5V
  reg01.CHG_CONFIG_ENABLE = (batteryPresent ? 0b1 : 0b0); // battery charge enable/disable
  reg01.CHG_CONFIG_OTG = 0b0;
  reg01.WATCHDOG_TIMER_RESET = 0b1; // reset watchdog timer
  reg01.REGISTER_RESET = 0b0; // keep current register setting

  sercom.startTransmissionWIRE(BQ24195_ADDRESS, WIRE_WRITE_FLAG);
  sercom.sendDataMasterWIRE(BQ24195_REG01_ADDRESS);
  sercom.sendDataMasterWIRE(reg01.val);
  sercom.prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);

  BQ24195_REG05 reg05;
  reg05.RSVD = 0b0;
  reg05.CHG_TIMER = 0b01; // fast charge timer 8 hours
  reg05.EN_TIMER = (batteryPresent ? 0b1 : 0b0); // enable/disable charge safety timer
  reg05.WATCHDOG = 0b00; // disable watchdog timer to stay in host mode
  reg05.TERM_STAT = 0b0; // charge termination indicator match ITERM
  reg05.EN_TERM = 0b1; // enable charge termination

  sercom.startTransmissionWIRE(BQ24195_ADDRESS, WIRE_WRITE_FLAG);
  sercom.sendDataMasterWIRE(BQ24195_REG05_ADDRESS);
  sercom.sendDataMasterWIRE(reg05.val);
  sercom.prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);

  BQ24195_REG07 reg07;
  reg07.INT_MASK_BAT = 0b1; // INT on battery fault
  reg07.INT_MASK_CHG = 0b1; // INT on charge fault
  reg07.RSVD = 0b010;
  reg07.BATFET_DISABLE = (batteryPresent ? 0b0 : 0b1); // battery FET enable/disable
  reg07.TMR2X_EN = 0b0; // safety timer not slowed by 2X
  reg07.DPDM_EN = (USBDetect ? 0b1 : 0b0); // D+/D- detection enable/disable

  sercom.startTransmissionWIRE(BQ24195_ADDRESS, WIRE_WRITE_FLAG);
  sercom.sendDataMasterWIRE(BQ24195_REG07_ADDRESS);
  sercom.sendDataMasterWIRE(reg07.val);
  sercom.prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
}

#endif
