/*
  PMIC.h - initialization of Power Management ICs
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
#ifndef _PMIC_ARDUINO_H_
#define _PMIC_ARDUINO_H_

#include "Arduino.h"
#include "PMIC/BQ24195.h"
#include "PMIC/MP2629.h"
#include "wiring_private.h"

#define ILIM_100MA 0x00
#define ILIM_150MA 0x01
#define ILIM_500MA 0x02
#define ILIM_900MA 0x03
#define ILIM_1_2A  0x04
#define ILIM_1_5A  0x05
#define ILIM_2_0A  0x06
#define ILIM_3_0A  0x07




class PMICClass
{
public:
  PMICClass(void);
  ~PMICClass(void);
  void init(SERCOM * sercom);
  void setupBQ24195(bool batteryPresent, bool USBDetect);
  void setupMP2629(bool batteryPresent);
  bool isBatteryConnected();
  bool isBQ24195();
  void setIlim(uint8_t ilim = ILIM_500MA);
private:
  uint8_t readFrom(uint8_t pmicAdd, uint8_t address);
  void writereg(uint8_t pmicAdd, uint8_t address, uint8_t value);
private:
  SERCOM * _sercom;
  uint8_t _ilim = ILIM_500MA;
};

extern PMICClass PMICArduino;
#endif
