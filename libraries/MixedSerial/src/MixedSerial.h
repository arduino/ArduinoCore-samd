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

#ifndef SoftwareSerial_h
#define SoftwareSerial_h

#include <inttypes.h>
#include <Stream.h>
#include <Uart.h>
#include <variant.h>

/******************************************************************************
  Definitions
******************************************************************************/

class MixedSerial : public Stream {
  private:
    // per object data
    uint8_t _transmitPin;
    uint32_t _transmitBitMask;
    volatile PORT_OUT_Type *_transmitPortRegister;

    // Expressed as 4-cycle delays (must never be 0!)
    uint16_t _rx_delay_centering;
    uint16_t _rx_delay_intrabit;
    uint16_t _rx_delay_stopbit;
    uint16_t _tx_delay;

    uint16_t _buffer_overflow: 1;
    uint16_t _inverse_logic: 1;

    // static data
    Uart& uart;

    void tx_pin_write(uint8_t pin_state) __attribute__((__always_inline__));
    void setTX(uint8_t transmitPin);

  public:
    // public methods
    MixedSerial(Uart& _s, int _pinTX);
    ~MixedSerial();
    void begin(long speed);
    void end();
    int peek();

    virtual size_t write(uint8_t byte);
    virtual int read();
    virtual int available();
    virtual void flush();
    operator bool() {
      return true;
    }

    using Print::write;

    // public only for easy access by interrupt handlers
    static inline void handle_interrupt() __attribute__((__always_inline__));
};

#endif
