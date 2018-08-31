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

#ifndef _BOARD_DRIVER_PMIC_
#define _BOARD_DRIVER_PMIC_

#include <sam.h>
#include <stdbool.h>
#include "board_definitions.h"

//Default PMIC (BQ24195) I2C address
#define PMIC_ADDRESS                        0x6B

// Register address definitions
#define INPUT_SOURCE_REGISTER               0x00
#define POWERON_CONFIG_REGISTER             0x01
#define CHARGE_CURRENT_CONTROL_REGISTER     0x02
#define PRECHARGE_CURRENT_CONTROL_REGISTER  0x03
#define CHARGE_VOLTAGE_CONTROL_REGISTER     0x04
#define CHARGE_TIMER_CONTROL_REGISTER       0x05
#define THERMAL_REG_CONTROL_REGISTER        0x06
#define MISC_CONTROL_REGISTER               0x07
#define SYSTEM_STATUS_REGISTER              0x08
#define FAULT_REGISTER                      0x09
#define PMIC_VERSION_REGISTER               0x0A

void configure_pmic();

#endif // _BOARD_DRIVER_PMIC_
