/*
  Copyright (c) 2014 Arduino.  All right reserved.

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

#ifndef _RING_BUFFER_
#define _RING_BUFFER_

#include <stdint.h>

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which head is the index of the location
// to which to write the next incoming character and tail is the index of the
// location from which to read.
#define SERIAL_BUFFER_SIZE 64

#define RINGBUFFER_HAS_ADDITIONAL_STORAGE_API

class RxBuffer {
public:
  RxBuffer(uint8_t* _buf, int _size) : buf(_buf), size(_size) {}
  uint8_t* buf;
  int size;
};

class TxBuffer {
public:
  TxBuffer(uint8_t* _buf, int _size) : buf(_buf), size(_size) {}
  uint8_t* buf;
  int size;
};

#define TXBUFFER(x) TxBuffer(x, sizeof(x))
#define RXBUFFER(x) RxBuffer(x, sizeof(x))

class RingBuffer
{
  public:
    uint8_t _aucBuffer[SERIAL_BUFFER_SIZE] ;
    int _iHead ;
    int _iTail ;

  public:
	RingBuffer( void ) ;
	void store_char( uint8_t c ) ;
	void clear();
	int read_char();
	int available();
	int peek();
	bool isFull();
	void addStorage(uint8_t* _buffer, int _size) {
		additionalSize = _size;
		additionalBuffer = _buffer;
	};

  private:
	int nextIndex(int index);
	uint8_t* additionalBuffer;
	int additionalSize = 0;
	int size;
} ;

#endif /* _RING_BUFFER_ */
