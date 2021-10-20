/*
  Copyright (c) 2021 Arduino LLC.  All right reserved.

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

#ifndef _BOARD_H_INCLUDED
#define _BOARD_H_INCLUDED


/*SDU_SIZE is not specifically for nano 33 iot but it's core version dependent.
  From 1.8.11 the core footprint is too big to fit 0x4000.
  If you need to recompile SDUBoot.ino for */


#ifdef ARDUINO_SAM_ZERO
#define SDU_START    0x4000
#else
#define SDU_START    0x2000
#endif
#if defined(ARDUINO_SAMD_NANO_33_IOT)
#define SDU_SIZE  0x6000
#else
#define SDU_SIZE  0x4000
#endif

#define SKETCH_START (uint32_t*)(SDU_START + SDU_SIZE)

#define UPDATE_FILE "UPDATE.BIN"

#if defined(USE_ARDUINO_MKR_PIN_LAYOUT)
#ifndef SDCARD_SS_PIN
#define SDCARD_SS_PIN 4
#endif
#else
#define SDCARD_SS_PIN 10
#endif

#endif
