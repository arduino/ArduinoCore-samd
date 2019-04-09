/*
  This file is part of the ArduinoECCX08 library.
  Copyright (c) 2019 Arduino SA. All rights reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <Arduino.h>
#include "MixedSerial.h"
#include <variant.h>
#include <WInterrupts.h>

// Constructor
MixedSerial::MixedSerial(Uart& _s, int _pinTX) :
  _rx_delay_centering(0),
  _rx_delay_intrabit(0),
  _rx_delay_stopbit(0),
  _tx_delay(0),
  uart(_s)
{
  _transmitPin = _pinTX;
}

// Destructor
MixedSerial::~MixedSerial() {
  end();
}

void MixedSerial::setTX(uint8_t tx) {
  // First write, then set output. If we do this the other way around,
  // the pin would be output low for a short while before switching to
  // output hihg. Now, it is input with pullup for a short while, which
  // is fine. With inverse logic, either order is fine.
  pinMode(tx, OUTPUT);
  _transmitBitMask = digitalPinToBitMask(tx);
  PortGroup * port = digitalPinToPort(tx);
  _transmitPortRegister = (PORT_OUT_Type*) portOutputRegister(port);
}

void MixedSerial::begin(long speed) {

  uart.begin(speed);
  setTX(_transmitPin);

  // Precalculate the various delays
  //Calculate the distance between bit in micro seconds
  uint32_t bit_delay = (uint32_t)(1000000) / (speed);

  _tx_delay = bit_delay;
}

void MixedSerial::end() {
}

int MixedSerial::read() {
  return uart.read();
}


int MixedSerial::available() {
  return uart.available();
}


size_t MixedSerial::write(uint8_t b) {
  if (_tx_delay == 0) {
    setWriteError();
    return 0;
  }

  // By declaring these as local variables, the compiler will put them
  // in registers _before_ disabling interrupts and entering the
  // critical timing sections below, which makes it a lot easier to
  // verify the cycle timings
  volatile PORT_OUT_Type *reg = _transmitPortRegister;
  uint32_t reg_mask = _transmitBitMask;
  uint32_t inv_mask = ~_transmitBitMask;
  uint16_t delay = _tx_delay;

  noInterrupts();

  reg->reg &= inv_mask;

  delayMicroseconds(delay);

  // Write each of the 8 bits
  for (uint8_t i = 0; i < 8; i++) {
    if (bitRead(b, i)) {
      reg->reg |= reg_mask; // send 1
    }
    else {
      reg->reg &= inv_mask; // send 0
    }
    delayMicroseconds(delay);
  }

  // restore pin to natural state
  reg->reg |= reg_mask;

  delayMicroseconds(delay);
  interrupts();

  return 1;
}

void MixedSerial::flush() {
  uart.flush();
}

int MixedSerial::peek() {
  uart.peek();
}
