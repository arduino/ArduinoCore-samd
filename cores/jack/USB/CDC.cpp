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

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

// Include Atmel headers
#include "Arduino.h"
#include "sam.h"
#include "wiring_constants.h"
#include "USBCore.h"
#include "USB/USB_device.h"
#include "USBDesc.h"
#include "USBAPI.h"

#include "Reset.h"


#ifdef CDC_ENABLED

#define CDC_SERIAL_BUFFER_SIZE	64

/* For information purpose only since RTS is not always handled by the terminal application */
#define CDC_LINESTATE_DTR		0x01 // Data Terminal Ready
#define CDC_LINESTATE_RTS		0x02 // Ready to Send

#define CDC_LINESTATE_READY		(CDC_LINESTATE_RTS | CDC_LINESTATE_DTR)

struct ring_buffer
{
	uint8_t buffer[CDC_SERIAL_BUFFER_SIZE];
	volatile uint32_t head;
	volatile uint32_t tail;
};

ring_buffer cdc_rx_buffer = { { 0 }, 0, 0};

typedef struct
{
	uint32_t	dwDTERate;
	uint8_t		bCharFormat;
	uint8_t 	bParityType;
	uint8_t 	bDataBits;
	uint8_t		lineState;
} LineInfo;

_Pragma("pack(1)")
static volatile LineInfo _usbLineInfo = {
   115200, // dWDTERate
    0x00,  // bCharFormat
    0x00,  // bParityType
    0x08,  // bDataBits
    0x00   // lineState
};

static const CDCDescriptor _cdcInterface =
{
#if (defined CDC_ENABLED) && defined(HID_ENABLED)
	D_IAD(0, 2, CDC_COMMUNICATION_INTERFACE_CLASS, CDC_ABSTRACT_CONTROL_MODEL, 0),
#endif
	//	CDC communication interface
	D_INTERFACE(CDC_ACM_INTERFACE,1,CDC_COMMUNICATION_INTERFACE_CLASS,CDC_ABSTRACT_CONTROL_MODEL,0),
	D_CDCCS( CDC_HEADER, CDC_V1_10 & 0xFF, (CDC_V1_10>>8) & 0x0FF ),	// Header (1.10 bcd)

	D_CDCCS4(CDC_ABSTRACT_CONTROL_MANAGEMENT,6),				// SET_LINE_CODING, GET_LINE_CODING, SET_CONTROL_LINE_STATE supported
	D_CDCCS(CDC_UNION,CDC_ACM_INTERFACE,CDC_DATA_INTERFACE),	// Communication interface is master, data interface is slave 0
	D_CDCCS(CDC_CALL_MANAGEMENT,1,1),							// Device handles call management (not)
	D_ENDPOINT(USB_ENDPOINT_IN (CDC_ENDPOINT_ACM),USB_ENDPOINT_TYPE_INTERRUPT,0x10, 0x10),

	//	CDC data interface
	D_INTERFACE(CDC_DATA_INTERFACE,2,CDC_DATA_INTERFACE_CLASS,0,0),
	D_ENDPOINT(USB_ENDPOINT_OUT(CDC_ENDPOINT_OUT),USB_ENDPOINT_TYPE_BULK,EPX_SIZE,0),
	D_ENDPOINT(USB_ENDPOINT_IN (CDC_ENDPOINT_IN ),USB_ENDPOINT_TYPE_BULK,EPX_SIZE,0)
};
_Pragma("pack()")

const void* WEAK CDC_GetInterface(void)
{
	return  &_cdcInterface;
}

uint32_t WEAK CDC_GetInterfaceLength(void)
{
    return sizeof( _cdcInterface );
}

bool WEAK CDC_Setup(Setup& setup)
{
	uint8_t r = setup.bRequest;
	uint8_t requestType = setup.bmRequestType;

	if (REQUEST_DEVICETOHOST_CLASS_INTERFACE == requestType)
	{
		if (CDC_GET_LINE_CODING == r)
		{
			USBD_SendControl(0,(void*)&_usbLineInfo,7);
			return true;
		}
	}

	if (REQUEST_HOSTTODEVICE_CLASS_INTERFACE == requestType)
	{
		if (CDC_SET_LINE_CODING == r)
		{
			USBD_RecvControl((void*)&_usbLineInfo,7);
		}

		if (CDC_SET_CONTROL_LINE_STATE == r)
		{
			_usbLineInfo.lineState = setup.wValueL;
		}

		if (CDC_SET_LINE_CODING == r || CDC_SET_CONTROL_LINE_STATE == r)
		{
			// auto-reset into the bootloader is triggered when the port, already
			// open at 1200 bps, is closed. We check DTR state to determine if host 
			// port is open (bit 0 of lineState).
			if (1200 == _usbLineInfo.dwDTERate && (_usbLineInfo.lineState & 0x01) == 0)
			{
				initiateReset(250);
			}
			else
			{
				cancelReset();
			}
			return false;
		}
	}
	return false;
}

uint32_t _serialPeek = -1;
void Serial_::begin(uint32_t baud_count)
{
}

void Serial_::begin(uint32_t baud_count, uint8_t config)
{
}

void Serial_::end(void)
{
}

void Serial_::accept(void)
{
	uint8_t buffer[CDC_SERIAL_BUFFER_SIZE];
	uint32_t len = USBD_Recv(CDC_ENDPOINT_OUT, (void*)&buffer, CDC_SERIAL_BUFFER_SIZE);

	noInterrupts();
	ring_buffer *ringBuffer = &cdc_rx_buffer;
	uint32_t i = ringBuffer->head;

	// if we should be storing the received character into the location
	// just before the tail (meaning that the head would advance to the
	// current location of the tail), we're about to overflow the buffer
	// and so we don't write the character or advance the head.
	uint32_t k = 0;
	i = (i + 1) % CDC_SERIAL_BUFFER_SIZE;
	while (i != ringBuffer->tail && len>0) {
		len--;
		ringBuffer->buffer[ringBuffer->head] = buffer[k++];
		ringBuffer->head = i;
		i = (i + 1) % CDC_SERIAL_BUFFER_SIZE;
	}
	interrupts();
}

int Serial_::available(void)
{
	ring_buffer *buffer = &cdc_rx_buffer;
	return (uint32_t)(CDC_SERIAL_BUFFER_SIZE + buffer->head - buffer->tail) % CDC_SERIAL_BUFFER_SIZE;
}

int Serial_::peek(void)
{
	ring_buffer *buffer = &cdc_rx_buffer;

	if (buffer->head == buffer->tail)
	{
		return -1;
	}
	else
	{
		return buffer->buffer[buffer->tail];
	}
}

int Serial_::read(void)
{
	ring_buffer *buffer = &cdc_rx_buffer;

	// if the head isn't ahead of the tail, we don't have any characters
	if (buffer->head == buffer->tail)
	{
		return -1;
	}
	else
	{
		unsigned char c = buffer->buffer[buffer->tail];
		buffer->tail = (uint32_t)(buffer->tail + 1) % CDC_SERIAL_BUFFER_SIZE;
		if (USBD_Available(CDC_ENDPOINT_OUT))
			accept();
		return c;
	}
}

void Serial_::flush(void)
{
	USBD_Flush(CDC_ENDPOINT_IN);
}

size_t Serial_::write(const uint8_t *buffer, size_t size)
{
	/* only try to send bytes if the high-level CDC connection itself
	 is open (not just the pipe) - the OS should set lineState when the port
	 is opened and clear lineState when the port is closed.
	 bytes sent before the user opens the connection or after
	 the connection is closed are lost - just like with a UART. */

	// TODO - ZE - check behavior on different OSes and test what happens if an
	// open connection isn't broken cleanly (cable is yanked out, host dies
	// or locks up, or host virtual serial port hangs)
//	if (_usbLineInfo.lineState > 0)  // Problem with Windows(R)
	{
		uint32_t r = USBD_Send(CDC_ENDPOINT_IN, buffer, size);

		if (r > 0)
		{
			return r;
		} else
		{
			setWriteError();
			return 0;
		}
	}
	setWriteError();
	return 0;
}

size_t Serial_::write(uint8_t c) {
	return write(&c, 1);
}

// This operator is a convenient way for a sketch to check whether the
// port has actually been configured and opened by the host (as opposed
// to just being connected to the host).  It can be used, for example, in
// setup() before printing to ensure that an application on the host is
// actually ready to receive and display the data.
// We add a short delay before returning to fix a bug observed by Federico
// where the port is configured (lineState != 0) but not quite opened.
Serial_::operator bool()
{
	// this is here to avoid spurious opening after upload
	if (millis() < 500)
		return false;

	bool result = false;

	if (_usbLineInfo.lineState > 0)
	{
		result = true;
	}

	delay(10);
	return result;
}

Serial_ SerialUSB;

#endif
