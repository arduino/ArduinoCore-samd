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

Uart::Uart(SERCOM *_s, uint8_t _pinRX, uint8_t _pinTX, SercomRXPad _padRX, SercomUartTXPad _padTX)
{
  sercom = _s;
  uc_pinRX = _pinRX;
  uc_pinTX = _pinTX;
  uc_padRX=_padRX ;
  uc_padTX=_padTX;
}

void Uart::begin(unsigned long baudrate)
{
  begin(baudrate, (uint8_t)SERIAL_8N1);
}

void Uart::begin(unsigned long baudrate, uint16_t config)
{
  pinPeripheral(uc_pinRX, g_APinDescription[uc_pinRX].ulPinType);
  pinPeripheral(uc_pinTX, g_APinDescription[uc_pinTX].ulPinType);

  sercom->initUART(UART_INT_CLOCK, SAMPLE_RATE_x16, baudrate);
  sercom->initFrame(extractCharSize(config), LSB_FIRST, extractParity(config), extractNbStopBit(config));
  sercom->initPads(uc_padTX, uc_padRX);

  sercom->enableUART();
}

void Uart::end()
{
  sercom->resetUART();
  rxBuffer.clear();
}

void Uart::flush()
{
  sercom->flushUART();
}

void Uart::IrqHandler()
{
  if (sercom->availableDataUART()) {
    rxBuffer.store_char(sercom->readDataUART());
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
  return (sercom->isDataRegisterEmptyUART() ? 1 : 0);
}

int Uart::peek()
{
  return rxBuffer.peek();
}

int Uart::read()
{
  return rxBuffer.read_char();
}

size_t Uart::write(const uint8_t data)
{
  sercom->writeDataUART(data);
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
