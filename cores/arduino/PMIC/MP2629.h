/*
  MP2629.h - Register definitions for MP2629/MP2629L PMICs.
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
    uint8_t IIN_LIM:6;
    uint8_t EN_LIM:1;
    uint8_t EN_HIZ:1;
  };
  uint8_t val;
} MP2629_REG00;

static_assert(sizeof(MP2629_REG00) == 1, "MP2629_REG00 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t VIN_MIN:4;
    uint8_t RSVD:3;
    uint8_t REGISTER_RESET:1;
  };
  uint8_t val;
} MP2629_REG01;

static_assert(sizeof(MP2629_REG01) == 1, "MP2629_REG01 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t AICO_EN:1;
    uint8_t NTC_OPTION:1;
    uint8_t TJ_REG:2;
    uint8_t EN_CHG_NTC:1;
    uint8_t EN_OTG_NTC:1;
    uint8_t NTC_TYPE:1;
    uint8_t TSM_DLY:1;
  };
  uint8_t val;
} MP2629_REG02;

static_assert(sizeof(MP2629_REG02) == 1, "MP2629_REG02 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t IIN_DSCHG:3;
    uint8_t VIN_DSCHG:3;
    uint8_t ADC_RATE:1;
    uint8_t ADC_START:1;
  };
  uint8_t val;
} MP2629_REG03;

static_assert(sizeof(MP2629_REG03) == 1, "MP2629_REG03 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t VTRACK:1;
    uint8_t VSYS_MIN:3;
    uint8_t CHG_CONFIG:2;
    uint8_t STAT_EN:1;
    uint8_t BAT_LOADEN:1;
  };
  uint8_t val;
} MP2629_REG04;

static_assert(sizeof(MP2629_REG04) == 1, "MP2629_REG04 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t ICC:7;
    uint8_t VBATT_PRE:1;
  };
  uint8_t val;
} MP2629_REG05;

static_assert(sizeof(MP2629_REG05) == 1, "MP2629_REG05 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t ITERM:4;
    uint8_t IPRE:4;
  };
  uint8_t val;
} MP2629_REG06;

static_assert(sizeof(MP2629_REG06) == 1, "MP2629_REG06 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t VRECH:1;
    uint8_t VBATT_REG:7;
  };
  uint8_t val;
} MP2629_REG07;

static_assert(sizeof(MP2629_REG07) == 1, "MP2629_REG07 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t EN_TIMER:1;
    uint8_t CHG_TMR:2;
    uint8_t WATCHDOG_TIM_RST:1;
    uint8_t WATCHDOG:2;
    uint8_t RSVD:1;
    uint8_t EN_TERM:1;
  };
  uint8_t val;
} MP2629_REG08;

static_assert(sizeof(MP2629_REG08) == 1, "MP2629_REG08 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t VCLAMP:3;
    uint8_t RSVD:1;
    uint8_t RBAT_CMP:4;
  };
  uint8_t val;
} MP2629_REG09;

static_assert(sizeof(MP2629_REG09) == 1, "MP2629_REG09 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t TDISC_L:2;
    uint8_t TDISC_H:2;
    uint8_t SYSRST_SEL:1;
    uint8_t BATFET_DIS:1;
    uint8_t TMR2X_EN:1;
    uint8_t SW_FREQ:1;
  };
  uint8_t val;
} MP2629_REG0A;

static_assert(sizeof(MP2629_REG0A) == 1, "MP2629_REG0A union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t RSVD:6;
    uint8_t USB_DET_EN:1;
    uint8_t INT_MASK:1;
  };
  uint8_t val;
} MP2629_REG0B;

static_assert(sizeof(MP2629_REG0B) == 1, "MP2629_REG0B union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t VSYS_STAT:1;
    uint8_t THERM_STAT:1;
    uint8_t RNTC_FLOAT_STAT:1;
    uint8_t CHG_STAT:2;
    uint8_t VIN_STAT:3;
  };
  uint8_t val;
} MP2629_REG0C;

static_assert(sizeof(MP2629_REG0C) == 1, "MP2629_REG0C union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t NTC_FAULT:3;
    uint8_t BAT_FAULT:1;
    uint8_t THERMAL_SHUTDOWN:1;
    uint8_t INPUT_FAULT:1;
    uint8_t OTG_FAULT:1;
    uint8_t WATCHDOG_FAULT:1;
  };
  uint8_t val;
} MP2629_REG0D;

static_assert(sizeof(MP2629_REG0D) == 1, "MP2629_REG0D union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t VBATT:8;
  };
  uint8_t val;
} MP2629_REG0E;

static_assert(sizeof(MP2629_REG0E) == 1, "MP2629_REG0E union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t VSYS:8;
  };
  uint8_t val;
} MP2629_REG0F;

static_assert(sizeof(MP2629_REG0F) == 1, "MP2629_REG0F union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t NTC:8;
  };
  uint8_t val;
} MP2629_REG10;

static_assert(sizeof(MP2629_REG10) == 1, "MP2629_REG10 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t VIN:7;
    uint8_t RSVD:1;
  };
  uint8_t val;
} MP2629_REG11;

static_assert(sizeof(MP2629_REG11) == 1, "MP2629_REG11 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t ICHG:8;
  };
  uint8_t val;
} MP2629_REG12;

static_assert(sizeof(MP2629_REG12) == 1, "MP2629_REG12 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t IIN:8;
  };
  uint8_t val;
} MP2629_REG13;

static_assert(sizeof(MP2629_REG13) == 1, "MP2629_REG13 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t IIN_DPM:6;
    uint8_t IINPPM_STAT:1;
    uint8_t VINPPM_STAT:1;
  };
  uint8_t val;
} MP2629_REG14;

static_assert(sizeof(MP2629_REG14) == 1, "MP2629_REG14 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t RSVD:5;
    uint8_t VINPPM_INT_MASK:2;
    uint8_t AICO_STAT:1;
  };
  uint8_t val;
} MP2629_REG15;

static_assert(sizeof(MP2629_REG15) == 1, "MP2629_REG15 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t VCOLD:1;
    uint8_t VCOOL:2;
    uint8_t VWARM:2;
    uint8_t VHOT:1;
    uint8_t JEITA_ISET:1;
    uint8_t JEITA_VSET:1;
  };
  uint8_t val;
} MP2629_REG16;

static_assert(sizeof(MP2629_REG16) == 1, "MP2629_REG16 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t RSVDL3_0:3;
    uint8_t PN:3;
    uint8_t RSVD7:1;
    uint8_t SAFETY_TIMER:1;
  };
  uint8_t val;
} MP2629_REG17;

static_assert(sizeof(MP2629_REG17) == 1, "MP2629_REG17 union size is incorrect, should be 1 byte.");

typedef union {
  struct {
    uint8_t RSVD4_0:5;
    uint8_t ADDRESS:1;
    uint8_t RSVD7:1;
    uint8_t IINLIM_VINMIN_RESET_EN:1;
  };
  uint8_t val;
} MP2629_REG18;

static_assert(sizeof(MP2629_REG18) == 1, "MP2629_REG18 union size is incorrect, should be 1 byte.");


#define MP2629_ADDRESS          0x4B
#define MP2629_REG00_ADDRESS    0x00
#define MP2629_REG01_ADDRESS    0x01
#define MP2629_REG02_ADDRESS    0x02
#define MP2629_REG03_ADDRESS    0x03
#define MP2629_REG04_ADDRESS    0x04
#define MP2629_REG05_ADDRESS    0x05
#define MP2629_REG06_ADDRESS    0x06
#define MP2629_REG07_ADDRESS    0x07
#define MP2629_REG08_ADDRESS    0x08
#define MP2629_REG09_ADDRESS    0x09
#define MP2629_REG0A_ADDRESS    0x0A
#define MP2629_REG0B_ADDRESS    0x0B
#define MP2629_REG0C_ADDRESS    0x0C
#define MP2629_REG0D_ADDRESS    0x0D
#define MP2629_REG0E_ADDRESS    0x0E
#define MP2629_REG0F_ADDRESS    0x0F
#define MP2629_REG0G_ADDRESS    0x0G
#define MP2629_REG10_ADDRESS    0x10
#define MP2629_REG11_ADDRESS    0x11
#define MP2629_REG12_ADDRESS    0x12
#define MP2629_REG13_ADDRESS    0x13
#define MP2629_REG14_ADDRESS    0x14
#define MP2629_REG15_ADDRESS    0x15
#define MP2629_REG16_ADDRESS    0x16
#define MP2629_REG17_ADDRESS    0x17
#define MP2629_REG18_ADDRESS    0x18


#ifdef __cplusplus
} // extern "C"
#endif
