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

#pragma once

#define HSTPIPCFG_PTYPE_BLK 1
#define HSTPIPCFG_PTOKEN_IN 2
#define HSTPIPCFG_PTOKEN_OUT 3
#define HSTPIPCFG_PBK_1_BANK 4
#define HSTPIPCFG_PTYPE_INTRPT 5

#define EP0      0
#define EPX_SIZE 64 // 64 for Full Speed, EPT size max is 1024

#if defined __cplusplus

#include "Stream.h"
#include "RingBuffer.h"

//================================================================================
// USB

// Low level API
typedef struct {
	union {
		uint8_t bmRequestType;
		struct {
			uint8_t direction : 5;
			uint8_t type : 2;
			uint8_t transferDirection : 1;
		};
	};
	uint8_t bRequest;
	uint8_t wValueL;
	uint8_t wValueH;
	uint16_t wIndex;
	uint16_t wLength;
} Setup;

class USBDeviceClass {
public:
	USBDeviceClass() {};

	// USB Device API
	void init();
	bool attach();
	bool detach();
	void setAddress(uint32_t addr);

	bool configured();
	bool connected();

	// Setup API
	bool handleClassInterfaceSetup(Setup &setup);
	bool handleStandardSetup(Setup &setup);
	bool sendDescriptor(Setup &setup);

	// Control EndPoint API
	uint32_t sendControl(const void *data, uint32_t len);
	uint32_t recvControl(void *data, uint32_t len);
	bool sendConfiguration(uint32_t maxlen);
	bool sendStringDescriptor(const uint8_t *string, uint8_t maxlen);

	// Generic EndPoint API
	void initEP(uint32_t ep, uint32_t type);
	void handleEndpoint(uint8_t ep);

	uint32_t send(uint32_t ep, const void *data, uint32_t len);
	void sendZlp(uint32_t ep);
	uint32_t recv(uint32_t ep, void *data, uint32_t len);
	uint32_t recv(uint32_t ep);
	uint32_t available(uint32_t ep);
	void flush(uint32_t ep);
	void stall(uint32_t ep);

	// private?
	uint32_t armSend(uint32_t ep, const void *data, uint32_t len);
	uint8_t armRecv(uint32_t ep);
	uint8_t armRecvCtrlOUT(uint32_t ep);

	void ISRHandler();

private:
	bool initialized;
};

extern USBDeviceClass USBDevice;

//================================================================================
//	Serial over CDC (Serial1 is the physical port)

class Serial_ : public Stream
{
public:
	Serial_(USBDeviceClass &_usb) : usb(_usb) { }
	void begin(uint32_t baud_count);
	void begin(unsigned long, uint8_t);
	void end(void);

	virtual int available(void);
	virtual void accept(void);
	virtual int peek(void);
	virtual int read(void);
	virtual void flush(void);
	virtual size_t write(uint8_t);
	virtual size_t write(const uint8_t *buffer, size_t size);
	using Print::write; // pull in write(str) from Print
	operator bool();
private:
	USBDeviceClass &usb;
	RingBuffer *_cdc_rx_buffer;
};
extern Serial_ SerialUSB;

//================================================================================
//================================================================================
//	Mouse

#define MOUSE_LEFT 1
#define MOUSE_RIGHT 2
#define MOUSE_MIDDLE 4
#define MOUSE_ALL (MOUSE_LEFT | MOUSE_RIGHT | MOUSE_MIDDLE)

class Mouse_
{
private:
	uint8_t _buttons;
	void buttons(uint8_t b);
public:
	Mouse_(void);
	void begin(void);
	void end(void);
	void click(uint8_t b = MOUSE_LEFT);
	void move(signed char x, signed char y, signed char wheel = 0);
	void press(uint8_t b = MOUSE_LEFT);		// press LEFT by default
	void release(uint8_t b = MOUSE_LEFT);	// release LEFT by default
	bool isPressed(uint8_t b = MOUSE_ALL);	// check all buttons by default
};
extern Mouse_ Mouse;

//================================================================================
//================================================================================
//	Keyboard

#define KEY_LEFT_CTRL		0x80
#define KEY_LEFT_SHIFT		0x81
#define KEY_LEFT_ALT		0x82
#define KEY_LEFT_GUI		0x83
#define KEY_RIGHT_CTRL		0x84
#define KEY_RIGHT_SHIFT		0x85
#define KEY_RIGHT_ALT		0x86
#define KEY_RIGHT_GUI		0x87

#define KEY_UP_ARROW		0xDA
#define KEY_DOWN_ARROW		0xD9
#define KEY_LEFT_ARROW		0xD8
#define KEY_RIGHT_ARROW		0xD7
#define KEY_BACKSPACE		0xB2
#define KEY_TAB				0xB3
#define KEY_RETURN			0xB0
#define KEY_ESC				0xB1
#define KEY_INSERT			0xD1
#define KEY_DELETE			0xD4
#define KEY_PAGE_UP			0xD3
#define KEY_PAGE_DOWN		0xD6
#define KEY_HOME			0xD2
#define KEY_END				0xD5
#define KEY_CAPS_LOCK		0xC1
#define KEY_F1				0xC2
#define KEY_F2				0xC3
#define KEY_F3				0xC4
#define KEY_F4				0xC5
#define KEY_F5				0xC6
#define KEY_F6				0xC7
#define KEY_F7				0xC8
#define KEY_F8				0xC9
#define KEY_F9				0xCA
#define KEY_F10				0xCB
#define KEY_F11				0xCC
#define KEY_F12				0xCD

//	Low level key report: up to 6 keys and shift, ctrl etc at once
typedef struct
{
	uint8_t modifiers;
	uint8_t reserved;
	uint8_t keys[6];
} KeyReport;

class Keyboard_ : public Print
{
private:
	KeyReport _keyReport;
	void sendReport(KeyReport* keys);
public:
	Keyboard_(void);
	void begin(void);
	void end(void);
	virtual size_t write(uint8_t k);
	virtual size_t press(uint8_t k);
	virtual size_t release(uint8_t k);
	virtual void releaseAll(void);
};
extern Keyboard_ Keyboard;

//================================================================================
//================================================================================
//	HID 'Driver'

const void* HID_GetInterface(void);
uint32_t HID_GetInterfaceLength(void);
uint32_t HID_SizeReportDescriptor(void);

uint32_t		HID_GetDescriptor(void);
bool	HID_Setup(Setup& setup);
void	HID_SendReport(uint8_t id, const void* data, uint32_t len);

//================================================================================
//================================================================================
//	MSC 'Driver'

uint32_t		MSC_GetInterface(uint8_t* interfaceNum);
uint32_t		MSC_GetDescriptor(uint32_t i);
bool	MSC_Setup(Setup& setup);
bool	MSC_Data(uint8_t rx,uint8_t tx);

//================================================================================
//================================================================================
//	CDC 'Driver'

const void* CDC_GetInterface(/*uint8_t* interfaceNum*/);
uint32_t CDC_GetInterfaceLength(void);
uint32_t		CDC_GetOtherInterface(uint8_t* interfaceNum);
uint32_t		CDC_GetDescriptor(uint32_t i);
bool	CDC_Setup(Setup& setup);

#endif  // __cplusplus
