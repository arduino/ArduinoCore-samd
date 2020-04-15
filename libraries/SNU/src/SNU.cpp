/*
  Copyright (c) 2017 Arduino LLC.  All right reserved.

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

#include <Arduino.h>

#include "SNU.h"

__attribute__ ((section(".sketch_boot")))
unsigned char sduBoot[0x4000] = {
#if defined(ARDUINO_SAMD_MKRWIFI1010)
  #include "boot/mkrwifi1010.h"
#elif defined(ARDUINO_SAMD_MKRVIDOR4000)
  #include "boot/mkrvidor4000.h"
#elif defined(ARDUINO_SAMD_NANO_33_IOT)
  #include "boot/nano33iot.h"
#else
  #error "Unsupported board!"
#endif
};
