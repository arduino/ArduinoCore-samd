/*
  Copyright (c) 2011 Arduino.  All right reserved.

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

#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <inttypes.h>

#include "Stream.h"

#define PARITY_EVEN (0x1ul) 
#define PARITY_ODD	(0x2ul) 
#define PARITY_NONE (0x3ul) 
#define PARITY_MASK	(0xFul)

#define STOP_BIT_1	 	(0x10ul)
#define STOP_BIT_1_5	(0x20ul
#define STOP_BIT_2	 	(0x30ul)
#define STOP_BIT_MASK	(0xF0ul)

#define DATA_5	 	(0x100ul)
#define DATA_6	 	(0x200ul)
#define DATA_7	 	(0x300ul)
#define DATA_8	 	(0x400ul)
#define DATA_MASK	(0xF00ul)

#define SERIAL_5N1	(STOP_BIT_1 | PARITY_NONE | DATA_5)
#define SERIAL_6N1	(STOP_BIT_1 | PARITY_NONE | DATA_6)
#define SERIAL_7N1	(STOP_BIT_1 | PARITY_NONE | DATA_7)
#define SERIAL_8N1	(STOP_BIT_1 | PARITY_NONE | DATA_8) 
#define SERIAL_5N2	(STOP_BIT_2 | PARITY_NONE | DATA_5) 
#define SERIAL_6N2	(STOP_BIT_2 | PARITY_NONE | DATA_6) 
#define SERIAL_7N2	(STOP_BIT_2 | PARITY_NONE | DATA_7)
#define SERIAL_8N2	(STOP_BIT_2 | PARITY_NONE | DATA_8)
#define SERIAL_5E1	(STOP_BIT_1 | PARITY_EVEN | DATA_5)
#define SERIAL_6E1	(STOP_BIT_1 | PARITY_EVEN | DATA_6)
#define SERIAL_7E1	(STOP_BIT_1 | PARITY_EVEN | DATA_7)
#define SERIAL_8E1	(STOP_BIT_1 | PARITY_EVEN | DATA_8)
#define SERIAL_5E2	(STOP_BIT_2 | PARITY_EVEN | DATA_5)
#define SERIAL_6E2	(STOP_BIT_2 | PARITY_EVEN | DATA_6)
#define SERIAL_7E2	(STOP_BIT_2 | PARITY_EVEN | DATA_7)
#define SERIAL_8E2	(STOP_BIT_2 | PARITY_EVEN | DATA_8)
#define SERIAL_5O1	(STOP_BIT_1 | PARITY_ODD  | DATA_5)
#define SERIAL_6O1	(STOP_BIT_1 | PARITY_ODD  | DATA_6)
#define SERIAL_7O1	(STOP_BIT_1 | PARITY_ODD  | DATA_7)
#define SERIAL_8O1	(STOP_BIT_1 | PARITY_ODD  | DATA_8)
#define SERIAL_5O2	(STOP_BIT_2 | PARITY_ODD  | DATA_5)
#define SERIAL_6O2	(STOP_BIT_2 | PARITY_ODD  | DATA_6)
#define SERIAL_7O2	(STOP_BIT_2 | PARITY_ODD  | DATA_7)
#define SERIAL_8O2	(STOP_BIT_2 | PARITY_ODD  | DATA_8)

class HardwareSerial : public Stream
{
  public:
    virtual void begin(unsigned long);
    virtual void end();
    virtual int available(void) = 0;
    virtual int peek(void) = 0;
    virtual int read(void) = 0;
    virtual void flush(void) = 0;
    virtual size_t write(uint8_t) = 0;
    using Print::write; // pull in write(str) and write(buf, size) from Print
    virtual operator bool() = 0;
};

extern void serialEventRun(void) __attribute__((weak));

#endif
