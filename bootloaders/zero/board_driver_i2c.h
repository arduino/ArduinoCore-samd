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

#ifndef _BOARD_DRIVER_I2C_
#define _BOARD_DRIVER_I2C_

#include <sam.h>
#include <stdbool.h>
#include "board_definitions.h"

void i2c_init(uint32_t baud);
void i2c_end();
uint8_t i2c_requestFrom(uint8_t address, uint8_t quantity, bool stopBit);
void i2c_beginTransmission(uint8_t address);
uint8_t i2c_endTransmission(bool stopBit);
uint8_t i2c_write(uint8_t ucData);


#endif // _BOARD_DRIVER_I2C_
