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

#if defined(BOARD_ID_arduino_zero)
  #include "board_definitions_arduino_zero.h"
#elif defined(BOARD_ID_arduino_mkr1000)
  #include "board_definitions_arduino_mkr1000.h"
#elif defined(BOARD_ID_arduino_mkrzero)
  #include "board_definitions_arduino_mkrzero.h"
#elif defined(BOARD_ID_arduino_mkrfox1200)
  #include "board_definitions_arduino_mkrfox1200.h"
#elif defined(BOARD_ID_arduino_mkrgsm1400)
  #include "board_definitions_arduino_mkrgsm1400.h"
#elif defined(BOARD_ID_arduino_mkrwan1300)
  #include "board_definitions_arduino_mkrwan1300.h"
#elif defined(BOARD_ID_arduino_mkrwan1310)
  #include "board_definitions_arduino_mkrwan1310.h"
#elif defined(BOARD_ID_arduino_mkrwifi1010)
  #include "board_definitions_arduino_mkrwifi1010.h"
#elif defined(BOARD_ID_arduino_mkrvidor4000)
  #include "board_definitions_arduino_mkrvidor4000.h"
#elif defined(BOARD_ID_arduino_mkrnb1500)
  #include "board_definitions_arduino_mkrnb1500.h"
#elif defined(BOARD_ID_arduino_nano_33_iot)
  #include "board_definitions_arduino_nano_33_iot.h"
#else
  #error You must define a BOARD_ID and add the corresponding definitions in board_definitions.h
#endif

// Common definitions
// ------------------

#define BOOT_PIN_MASK (1U << (BOOT_LOAD_PIN & 0x1f))

