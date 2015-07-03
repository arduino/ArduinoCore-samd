/*
  Copyright (c) 2014 Arduino LLC.  All right reserved.

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

#ifndef __USBDESC_H__
#define __USBDESC_H__

// CDC or HID can be enabled together.
// These are now controlled by the boards.txt menu system
#if defined(ARDUINO_CDC_HID) || defined(ARDUINO_CDC_HID_UART)
  #define CDC_ENABLED
  #define HID_ENABLED
#elif defined(ARDUINO_CDC_ONLY) || defined(ARDUINO_CDC_UART)
  #define CDC_ENABLED
#elif defined(ARDUINO_HID_ONLY) || defined(ARDUINO_HID_UART)
  #define HID_ENABLED
#elif defined(ARDUINO_USB_UART_DISABLED) || defined(ARDUINO_UART_ONLY)
  // do nothing
#else
  #error "USBDesc.h: CDC_ENABLED and HID_ENABLED are now controlled by the boards.txt menu system"
#endif

// CDC
#define CDC_ACM_INTERFACE	0	// CDC ACM
#define CDC_DATA_INTERFACE	1	// CDC Data
#define CDC_ENDPOINT_ACM	1
#define CDC_ENDPOINT_OUT	2
#define CDC_ENDPOINT_IN		3

// HID
#define HID_INTERFACE		2   // HID
#define HID_ENDPOINT_INT	4

// Defined string description
#define IMANUFACTURER	1
#define IPRODUCT		2

#endif /* __USBDESC_H__ */
