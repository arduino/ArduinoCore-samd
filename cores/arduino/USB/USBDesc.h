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

#if !defined(ARDUINO_USB_UART_DISABLED) && !defined(ARDUINO_UART_ONLY)
#define PLUGGABLE_USB_ENABLED
#endif

// These are controlled by the boards.txt menu system.
// Now that there is Pluggable USB, these will be eliminated
// once an alternate method for USB PID allocation is found.
#if defined(ARDUINO_CDC_ONLY) || defined(ARDUINO_CDC_UART) || defined(ARDUINO_CDC_HID) || defined(ARDUINO_CDC_HID_UART) || defined(ARDUINO_CDC_MIDI_HID_UART) || defined(ARDUINO_CDC_MSD_HID_UART) || defined(ARDUINO_CDC_MSD_MIDI_HID_UART)
#define CDC_ENABLED
#if defined(ARDUINO_CDC_ONLY) || defined(ARDUINO_CDC_UART)
#define CDC_ONLY
#else
#define IAD_PRESENT
#endif
#endif

#if defined(ARDUINO_HID_ONLY) || defined(ARDUINO_HID_UART) || defined(ARDUINO_CDC_HID) || defined(ARDUINO_CDC_HID_UART) || defined(ARDUINO_CDC_MIDI_HID_UART) || defined(ARDUINO_CDC_MSD_HID_UART) || defined(ARDUINO_CDC_MSD_MIDI_HID_UART)
#define HID_ENABLED
#endif

#if defined(ARDUINO_MIDI_ONLY) || defined(ARDUINO_MIDI_UART) || defined(ARDUINO_CDC_MIDI_HID_UART) || defined(ARDUINO_CDC_MSD_MIDI_HID_UART)
#define MIDI_ENABLED
#define IAD_PRESENT
#endif

#if defined(ARDUINO_MSD_ONLY) || defined(ARDUINO_MSD_UART) || defined(ARDUINO_CDC_MSD_HID_UART) || defined(ARDUINO_CDC_MSD_MIDI_HID_UART)
#define MSD_ENABLED
#endif


#ifdef CDC_ENABLED
#define CDC_INTERFACE_COUNT 2
#define CDC_ENPOINT_COUNT 3
#else
#define CDC_INTERFACE_COUNT	0
#define CDC_ENPOINT_COUNT	0
#endif

// CDC
#define CDC_ACM_INTERFACE	0	// CDC ACM
#define CDC_DATA_INTERFACE	1	// CDC Data
#define CDC_FIRST_ENDPOINT	1
#define CDC_ENDPOINT_ACM	1
#define CDC_ENDPOINT_OUT	2
#define CDC_ENDPOINT_IN		3

#ifdef CDC_ENABLED
#define CDC_RX CDC_ENDPOINT_OUT
#define CDC_TX CDC_ENDPOINT_IN
#endif

#define ISERIAL_MAX_LEN        20

// Defined string description
#define IMANUFACTURER	1
#define IPRODUCT    2
#define ISERIAL    3

#endif /* __USBDESC_H__ */
