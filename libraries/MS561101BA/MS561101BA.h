/*
MS5611-01BA.h - Interfaces a Measurement Specialities MS5611-01BA with Arduino
See http://www.meas-spec.com/downloads/MS5611-01BA01.pdf for the device datasheet

Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>

Development of this code has been supported by the Department of Computer Science, Universita' degli Studi di Torino, Italy within the Piemonte Project http://www.piemonte.di.unito.it/


This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/



#ifndef MS561101BA_h
#define MS561101BA_h

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#include "Arduino.h"
#include <Wire.h>

//#define DEBUG_V
//#define DEBUG
//#include <DebugUtils.h>

// addresses of the device
#define MS561101BA_ADDR_CSB_HIGH  0x76   //CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
#define MS561101BA_ADDR_CSB_LOW   0x77   //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)

// registers of the device
#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
#define MS561101BA_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS561101BA_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define MS561101BA_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values. 
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry.



class MS561101BA {
  public:
    MS561101BA();
    void init(uint8_t addr);
    float getPressure(uint8_t OSR);
    float getTemperature(uint8_t OSR);
    int32_t getDeltaTemp(uint8_t OSR);
    uint32_t rawPressure(uint8_t OSR);
    uint32_t rawTemperature(uint8_t OSR);
    int readPROM();
    void reset();
    uint32_t lastPresConv, lastTempConv;
  private:
    void startConversion(uint8_t command);
    uint32_t getConversion(uint8_t command);
    uint8_t _addr;
    uint16_t _Cal[MS561101BA_PROM_REG_COUNT];
    uint32_t pressCache, tempCache;
};

#endif // MS561101BA_h