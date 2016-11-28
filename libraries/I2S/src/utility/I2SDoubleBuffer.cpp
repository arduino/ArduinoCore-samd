/*
  Copyright (c) 2016 Arduino LLC.  All right reserved.

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

#include <string.h>

#include "I2SDoubleBuffer.h"

I2SDoubleBuffer::I2SDoubleBuffer()
{
  reset();
}

I2SDoubleBuffer::~I2SDoubleBuffer()
{
}

void I2SDoubleBuffer::reset()
{
  _index = 0;
  _length[0] = 0;
  _length[1] = 0;
  _readOffset[0] = 0;
  _readOffset[1] = 0;
}

size_t I2SDoubleBuffer::availableForWrite()
{
  return (I2S_BUFFER_SIZE - (_length[_index] - _readOffset[_index]));
}

size_t I2SDoubleBuffer::write(const void *buffer, size_t size)
{
  size_t space = availableForWrite();

  if (size > space) {
    size = space;
  }

  if (size == 0) {
    return 0;
  }

  memcpy(&_buffer[_index][_length[_index]], buffer, size);

  _length[_index] += size;

  return size;
}

size_t I2SDoubleBuffer::read(void *buffer, size_t size)
{
  size_t avail = available();

  if (size > avail) {
    size = avail;
  }

  if (size == 0) {
    return 0;
  }

  memcpy(buffer, &_buffer[_index][_readOffset[_index]], size);
  _readOffset[_index] += size;

  return size;
}

size_t I2SDoubleBuffer::peek(void *buffer, size_t size)
{
  size_t avail = available();

  if (size > avail) {
    size = avail;
  }

  if (size == 0) {
    return 0;
  }

  memcpy(buffer, &_buffer[_index][_readOffset[_index]], size);

  return size;
}

void* I2SDoubleBuffer::data()
{
  return (void*)_buffer[_index];
}

size_t I2SDoubleBuffer::available()
{
  return _length[_index] - _readOffset[_index];
}

void I2SDoubleBuffer::swap(int length)
{
  if (_index == 0) {
    _index = 1;
  } else {
    _index = 0;
  }

  _length[_index] = length;
  _readOffset[_index] = 0;
}
