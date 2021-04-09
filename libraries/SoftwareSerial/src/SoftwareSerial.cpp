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
#include "SoftwareSerial.h"
#include <variant.h>
#include <WInterrupts.h>

SoftwareSerial *SoftwareSerial::active_object = 0;
char SoftwareSerial::_receive_buffer[_SS_MAX_RX_BUFF];
volatile uint8_t SoftwareSerial::_receive_buffer_tail = 0;
volatile uint8_t SoftwareSerial::_receive_buffer_head = 0;

bool SoftwareSerial::listen()
{
  if (!_rx_delay_stopbit)
    return false;

  if (active_object != this) {
    if (active_object) {
      active_object->stopListening();
    }

    _buffer_overflow = false;
    _receive_buffer_head = _receive_buffer_tail = 0;
    active_object = this;

    if (_inverse_logic)
      //Start bit high
      attachInterrupt(_receivePin, handle_interrupt, RISING);
    else
      //Start bit low
      attachInterrupt(_receivePin, handle_interrupt, FALLING);

    return true;
  }
  return false;
}

bool SoftwareSerial::stopListening() {
  if (active_object == this) {
    EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT( 1 << digitalPinToInterrupt( _receivePin )) ;
    active_object = NULL;
    return true;
  }
  return false;
}


void SoftwareSerial::recv() {
  uint8_t d = 0;

  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if (_inverse_logic ? rx_pin_read() : !rx_pin_read()) {
    EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT( 1 << digitalPinToInterrupt(_receivePin));

    // Wait approximately 1/2 of a bit width to "center" the sample
    delayMicroseconds(_rx_delay_centering);

    // Read each of the 8 bits
    for (uint8_t i = 0; i < 8; i++) {
      delayMicroseconds(_rx_delay_intrabit);
      d |= (rx_pin_read() ? 1 : 0) << i;
    }
    if (_inverse_logic) {
      d = ~d;
    }

    // if buffer full, set the overflow flag and return
    uint8_t next = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    if (next != _receive_buffer_head) {
      // save new data in buffer: tail points to where byte goes
      _receive_buffer[_receive_buffer_tail] = d; // save new byte
      _receive_buffer_tail = next;
    }
    else {
      _buffer_overflow = true;
    }
    // skip the stop bit
    delayMicroseconds(_rx_delay_stopbit);
    EIC->INTENSET.reg = EIC_INTENSET_EXTINT( 1 << digitalPinToInterrupt(_receivePin));
  }
}


uint32_t SoftwareSerial::rx_pin_read() {
  return _receivePortRegister->reg & digitalPinToBitMask(_receivePin);
}

/* static */
inline void SoftwareSerial::handle_interrupt() {
  if (active_object) {
    active_object->recv();
  }
}


// Constructor
SoftwareSerial::SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic /* = false */) :
  _rx_delay_centering(0),
  _rx_delay_intrabit(0),
  _rx_delay_stopbit(0),
  _tx_delay(0),
  _buffer_overflow(false),
  _inverse_logic(inverse_logic)
{
  _receivePin = receivePin;
  _transmitPin = transmitPin;
}

// Destructor
SoftwareSerial::~SoftwareSerial() {
  end();
}

void SoftwareSerial::setTX(uint8_t tx) {
  // First write, then set output. If we do this the other way around,
  // the pin would be output low for a short while before switching to
  // output hihg. Now, it is input with pullup for a short while, which
  // is fine. With inverse logic, either order is fine.
  pinMode(tx, OUTPUT);
  _transmitBitMask = digitalPinToBitMask(tx);
  PortGroup * port = digitalPinToPort(tx);
  _transmitPortRegister = (PORT_OUT_Type*) portOutputRegister(port);

}

void SoftwareSerial::setRX(uint8_t rx) {
  pinMode(rx, INPUT_PULLUP);
  if (!_inverse_logic)
    _receivePin = rx;
  _receiveBitMask = digitalPinToBitMask(rx);
  PortGroup * port = digitalPinToPort(rx);
  _receivePortRegister = (PORT_IN_Type*) portInputRegister(port);

}


void SoftwareSerial::begin(long speed) {
  setTX(_transmitPin);
  setRX(_receivePin);
  // Precalculate the various delays
  //Calculate the distance between bit in micro seconds
  uint32_t bit_delay = (uint32_t)(1000000) / (speed);

  _tx_delay = bit_delay;

  // Only setup rx when we have a valid PCINT for this pin
  if (digitalPinToInterrupt(_receivePin) != NOT_AN_INTERRUPT) {
    //Wait 1/2 bit - 2 micro seconds (time for interrupt to be served)
    _rx_delay_centering = (bit_delay / 2) - 2;
    //Wait 1 bit - 2 micro seconds (time in each loop iteration)
    _rx_delay_intrabit = bit_delay - 2;
    //Wait 1 bit (the stop one)
    _rx_delay_stopbit = bit_delay;
  }
  listen();
  write(0x55); //send a dummy byte to sync the start byte
}

void SoftwareSerial::end() {
  stopListening();
}

int SoftwareSerial::read() {
  if (!isListening()) {
    return -1;
  }


  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail) {
    return -1;
  }

  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}


int SoftwareSerial::available() {
  if (!isListening())
    return 0;

  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}


size_t SoftwareSerial::write(uint8_t b) {
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
  bool inv = _inverse_logic;
  uint16_t delay = _tx_delay;

  if (inv)
    b = ~b;
  // turn off interrupts for a clean txmit
  //noInterrupts();
  EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT( 1 << digitalPinToInterrupt(_transmitPin));
  // Write the start bit
  if (inv)
    reg->reg |= reg_mask;
  else
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
  if (inv)
    reg->reg &= inv_mask;
  else
    reg->reg |= reg_mask;


  //interrupts();
  EIC->INTENSET.reg = EIC_INTENSET_EXTINT( 1 << digitalPinToInterrupt(_transmitPin));
  delayMicroseconds(delay);

  return 1;
}

void SoftwareSerial::flush() {
  if (!isListening())
    return;

  EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT( 1 << digitalPinToInterrupt( _receivePin ) ) ;

  _receive_buffer_head = _receive_buffer_tail = 0;

  EIC->INTENSET.reg = EIC_INTENSET_EXTINT( 1 << digitalPinToInterrupt( _receivePin ) ) ;

}

int SoftwareSerial::peek() {
  if (!isListening())
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  return _receive_buffer[_receive_buffer_head];
}
