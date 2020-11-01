/*
  BQ24195.h - Register definitions for BQ24195/BQ24195L PMICs.
  Copyright (c) 2020 Kevin P. Fleming.  All right reserved.

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

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef union {
  struct {
    uint8_t IINLIM:3;
    uint8_t VINDPM:4;
    uint8_t EN_HIZ:1;
  };
  uint8_t val;
} BQ24195_REG00;

static_assert(sizeof(BQ24195_REG00) == 1, "BQ24195_REG00 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t RSVD:1;
    uint8_t SYS_MIN:3;
    uint8_t CHG_CONFIG_ENABLE:1;
    uint8_t CHG_CONFIG_OTG:1;
    uint8_t WATCHDOG_TIMER_RESET:1;
    uint8_t REGISTER_RESET:1;
  };
  uint8_t val;
} BQ24195_REG01;

static_assert(sizeof(BQ24195_REG01) == 1, "BQ24195_REG01 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t FORCE_20PCT:1;
    uint8_t RSVD:1;
    uint8_t ICHG:6;
  };
  uint8_t val;
} BQ24195_REG02;

static_assert(sizeof(BQ24195_REG02) == 1, "BQ24195_REG02 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t ITERM:4;
    uint8_t IPRECHG:4;
  };
  uint8_t val;
} BQ24195_REG03;

static_assert(sizeof(BQ24195_REG03) == 1, "BQ24195_REG03 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t VRECHG:1;
    uint8_t BATLOWV:1;
    uint8_t VREG:6;
  };
  uint8_t val;
} BQ24195_REG04;

static_assert(sizeof(BQ24195_REG04) == 1, "BQ24195_REG04 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t RSVD:1;
    uint8_t CHG_TIMER:2;
    uint8_t EN_TIMER:1;
    uint8_t WATCHDOG:2;
    uint8_t TERM_STAT:1;
    uint8_t EN_TERM:1;
  };
  uint8_t val;
} BQ24195_REG05;

static_assert(sizeof(BQ24195_REG05) == 1, "BQ24195_REG05 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t VRECHG:1;
    uint8_t BATLOWV:1;
    uint8_t VREG:6;
  };
  uint8_t val;
} BQ24195_REG06;

static_assert(sizeof(BQ24195_REG06) == 1, "BQ24195_REG06 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t INT_MASK_BAT:1;
    uint8_t INT_MASK_CHG:1;
    uint8_t RSVD:3;
    uint8_t BATFET_DISABLE:1;
    uint8_t TMR2X_EN:1;
    uint8_t DPDM_EN:1;
  };
  uint8_t val;
} BQ24195_REG07;

static_assert(sizeof(BQ24195_REG07) == 1, "BQ24195_REG07 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t VSYS_STAT:1;
    uint8_t THERM_STAT:1;
    uint8_t PG_STAT:1;
    uint8_t DPM_STAT:1;
    uint8_t CHRG_STAT:2;
    uint8_t VBUS_STAT:2;
  };
  uint8_t val;
} BQ24195_REG08;

static_assert(sizeof(BQ24195_REG08) == 1, "BQ24195_REG08 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t NTC_FAULT:3;
    uint8_t BAT_FAULT:1;
    uint8_t CHRG_FAULT:2;
    uint8_t RSVD:1;
    uint8_t WATCHDOG_FAULT:1;
  };
  uint8_t val;
} BQ24195_REG09;

static_assert(sizeof(BQ24195_REG09) == 1, "BQ24195_REG09 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t DEV_REG:2;
    uint8_t TS_PROFILE:1;
    uint8_t PN:3;
    uint8_t RSVD:2;
  };
  uint8_t val;
} BQ24195_REG0A;

static_assert(sizeof(BQ24195_REG0A) == 1, "BQ24195_REG0A union size is incorrect, should be 1 byte.");

#define BQ24195_ADDRESS		0x6B
#define BQ24195_REG00_ADDRESS	0x00
#define BQ24195_REG01_ADDRESS	0x01
#define BQ24195_REG02_ADDRESS	0x02
#define BQ24195_REG03_ADDRESS	0x03
#define BQ24195_REG04_ADDRESS	0x04
#define BQ24195_REG05_ADDRESS	0x05
#define BQ24195_REG06_ADDRESS	0x06
#define BQ24195_REG07_ADDRESS	0x07
#define BQ24195_REG08_ADDRESS	0x08
#define BQ24195_REG09_ADDRESS	0x09
#define BQ24195_REG0A_ADDRESS	0x0A

#ifdef __cplusplus
} // extern "C"
#endif
