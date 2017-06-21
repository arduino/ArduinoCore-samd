/*
 * Copyright (c) 2015 Arduino LLC.  All right reserved.
 * Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.
 * Copyright (c) 2017 MattairTech LLC. All right reserved.
 * Copyright (C) 2014, ChaN, all right reserved.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef _UTIL_H_
#define _UTIL_H_

#include "sam.h"
#include <stdbool.h>
#include "board_definitions.h"

// TODO: Variable bootloader sizes
#if (SAMD11)
  #define APP_START 0x00001000
#else
  #define APP_START 0x00002000
#endif

#define SCB_AIRCR_VECTKEY_Val   0x05FA

void flashErase (uint32_t startAddress);
void flashWrite (uint32_t startAddress, uint32_t * buffer, uint32_t * ptr_data);
void pinMux (uint32_t pinmux);
void systemReset (void);
void pinConfig (uint8_t port, uint8_t pin, uint8_t config);
bool isPinActive (uint8_t port, uint8_t pin, uint8_t config);
void delayUs (uint32_t delay);
void waitForSync (void);

#endif
