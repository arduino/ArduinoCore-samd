/*
  Copyright (c) 2020 Arduino.  All right reserved.

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

#ifdef __cplusplus

#ifndef _SAFE_RING_BUFFER_
#define _SAFE_RING_BUFFER_

#include <api/RingBuffer.h>
#include "sync.h"

namespace arduino {

template <int N>
class SafeRingBufferN : public RingBufferN<N>
{
  public:
    int read_char();
    void store_char( uint8_t c ) ;
};

typedef SafeRingBufferN<SERIAL_BUFFER_SIZE> SafeRingBuffer;

template <int N>
int SafeRingBufferN<N>::read_char() {
  synchronized {
    return RingBufferN<N>::read_char();
  }
}

template <int N>
void SafeRingBufferN<N>::store_char(uint8_t c) {
  synchronized {
    RingBufferN<N>::store_char(c);
  }
}

}

#endif /* _SAFE_RING_BUFFER_ */
#endif /* __cplusplus */