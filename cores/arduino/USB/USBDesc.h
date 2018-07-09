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

#ifdef USBCON

// CDC or HID can be enabled together.
#ifndef CDC_DISABLED
#define CDC_ENABLED
#endif

#ifndef PLUGGABLE_USB_DISABLED
#define PLUGGABLE_USB_ENABLED
#endif

#define ISERIAL_MAX_LEN        65

// Defined string description
#define IMANUFACTURER	1
#define IPRODUCT    2
#define ISERIAL    3

#endif /* USBCON */

#endif /* __USBDESC_H__ */
