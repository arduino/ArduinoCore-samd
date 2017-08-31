/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

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

#include "Uart.h"
#include "Arduino.h"
#include "wiring_private.h"

#define NO_RTS_PIN 255
#define NO_CTS_PIN 255
#define RTS_RX_THRESHOLD 10

Uart::Uart(SERCOM *_s, uint8_t _pinRX, uint8_t _pinTX, SercomRXPad _padRX, SercomUartTXPad _padTX)
{
  sercom = _s;
  uc_pinRX = _pinRX;
  uc_pinTX = _pinTX;
  uc_padRX = _padRX ;
  uc_padTX = _padTX;
  uc_pinRTS = NO_RTS_PIN;
  uc_pinCTS = NO_CTS_PIN;
}

void Uart::begin(unsigned long baudrate)
{
  begin(baudrate, SERIAL_8N1);
}

void Uart::begin(unsigned long baudrate, uint16_t config)
{
  pinPeripheral(uc_pinRX, g_APinDescription[uc_pinRX].ulPinType);
  pinPeripheral(uc_pinTX, g_APinDescription[uc_pinTX].ulPinType);

  if (uc_pinRTS != NO_RTS_PIN) {
    pinPeripheral(uc_pinRTS, g_APinDescription[uc_pinRTS].ulPinType);
  }

  if (uc_pinCTS != NO_CTS_PIN) {
    pinPeripheral(uc_pinCTS, g_APinDescription[uc_pinCTS].ulPinType);
  }

  sercom->initUART(UART_INT_CLOCK, SAMPLE_RATE_x16, baudrate);
  sercom->initFrame(extractCharSize(config), LSB_FIRST, extractParity(config), extractNbStopBit(config));
  sercom->initPads(uc_padTX, uc_padRX);

  sercom->enableUART();
}

void Uart::end()
{
  if (uc_pinRTS != NO_RTS_PIN) {
    digitalWrite(uc_pinRTS, LOW);
    pinMode(uc_pinRTS, INPUT);
  }

  sercom->resetUART();
  rxBuffer.clear();
  txBuffer.clear();
}

void Uart::flush()
{
  while(txBuffer.available()); // wait until TX buffer is empty

  sercom->flushUART();
}

void Uart::IrqHandler()
{
  if (sercom->availableDataUART()) {
    rxBuffer.store_char(sercom->readDataUART());

    if (uc_pinRTS != NO_RTS_PIN) {
      // if there is NOT enough space in the RX buffer,
      // diable the receive complete interrupt
      if (rxBuffer.availableForStore() < RTS_RX_THRESHOLD) {
        sercom->disableReceiveCompleteInterruptUART();
      }
    }
  }

  if (sercom->isDataRegisterEmptyUART()) {
    if (txBuffer.available()) {
      uint8_t data = txBuffer.read_char();

      sercom->writeDataUART(data);
    } else {
      sercom->disableDataRegisterEmptyInterruptUART();
    }
  }

  if (sercom->isUARTError()) {
    sercom->acknowledgeUARTError();
    // TODO: if (sercom->isBufferOverflowErrorUART()) ....
    // TODO: if (sercom->isFrameErrorUART()) ....
    // TODO: if (sercom->isParityErrorUART()) ....
    sercom->clearStatusUART();
  }
}

int Uart::available()
{
  return rxBuffer.available();
}

int Uart::availableForWrite()
{
  return txBuffer.availableForStore();
}

int Uart::peek()
{
  return rxBuffer.peek();
}

int Uart::read()
{
  int c = rxBuffer.read_char();

  if (uc_pinRTS != NO_RTS_PIN) {
    // if there is enough space in the RX buffer, 
    // enable the receive completer interrupt
    if (rxBuffer.availableForStore() > RTS_RX_THRESHOLD) {
      sercom->enableReceiveCompleteInterruptUART();
    }
  }

  return c;
}

size_t Uart::write(const uint8_t data)
{
  if (sercom->isDataRegisterEmptyUART() && txBuffer.available() == 0) {
    sercom->writeDataUART(data);
  } else {
    while(txBuffer.isFull()); // spin lock until a spot opens up in the buffer

    txBuffer.store_char(data);

    sercom->enableDataRegisterEmptyInterruptUART();
  }

  return 1;
}

SercomNumberStopBit Uart::extractNbStopBit(uint16_t config)
{
  switch(config & HARDSER_STOP_BIT_MASK)
  {
    case HARDSER_STOP_BIT_1:
    default:
      return SERCOM_STOP_BIT_1;

    case HARDSER_STOP_BIT_2:
      return SERCOM_STOP_BITS_2;
  }
}

SercomUartCharSize Uart::extractCharSize(uint16_t config)
{
  switch(config & HARDSER_DATA_MASK)
  {
    case HARDSER_DATA_5:
      return UART_CHAR_SIZE_5_BITS;

    case HARDSER_DATA_6:
      return UART_CHAR_SIZE_6_BITS;

    case HARDSER_DATA_7:
      return UART_CHAR_SIZE_7_BITS;

    case HARDSER_DATA_8:
    default:
      return UART_CHAR_SIZE_8_BITS;

  }
}

SercomParityMode Uart::extractParity(uint16_t config)
{
  switch(config & HARDSER_PARITY_MASK)
  {
    case HARDSER_PARITY_NONE:
    default:
      return SERCOM_NO_PARITY;

    case HARDSER_PARITY_EVEN:
      return SERCOM_EVEN_PARITY;

    case HARDSER_PARITY_ODD:
      return SERCOM_ODD_PARITY;
  }
}

int Uart::attachRts(uint8_t pin)
{
  if (uc_padTX == UART_TX_RTS_CTS_PAD_0_2_3) {
    uc_pinRTS = pin;

    return 1;
  }

  return 0;
}

int Uart::attachCts(uint8_t pin)
{
  if (uc_padTX == UART_TX_RTS_CTS_PAD_0_2_3) {
    uc_pinCTS = pin;

    return 1;
  }

  return 0;
}
