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

#ifndef CDC_ENUMERATE_H
#define CDC_ENUMERATE_H

#include <sam.h>
#include <stdbool.h>

#define USB_EP_CTRL             (0)
#define USB_EP_OUT              (2)
#define USB_EP_OUT_SIZE         0x40
#define USB_EP_IN               (1)
#define USB_EP_IN_SIZE          0x40
#define USB_EP_COMM             (3)
#define MAX_EP                  (4)

/* USB standard request code */
#define STD_GET_STATUS_ZERO           (0x0080)
#define STD_GET_STATUS_INTERFACE      (0x0081)
#define STD_GET_STATUS_ENDPOINT       (0x0082)

#define STD_CLEAR_FEATURE_ZERO        (0x0100)
#define STD_CLEAR_FEATURE_INTERFACE   (0x0101)
#define STD_CLEAR_FEATURE_ENDPOINT    (0x0102)

#define STD_SET_FEATURE_ZERO          (0x0300)
#define STD_SET_FEATURE_INTERFACE     (0x0301)
#define STD_SET_FEATURE_ENDPOINT      (0x0302)

#define STD_SET_ADDRESS               (0x0500)
#define STD_GET_DESCRIPTOR            (0x0680)
#define STD_SET_DESCRIPTOR            (0x0700)
#define STD_GET_CONFIGURATION         (0x0880)
#define STD_SET_CONFIGURATION         (0x0900)
#define STD_GET_INTERFACE             (0x0A81)
#define STD_SET_INTERFACE             (0x0B01)
#define STD_SYNCH_FRAME               (0x0C82)

#define SAM_BA_MIN(a, b) (((a) < (b)) ? (a) : (b))


typedef struct _USB_CDC
{
	// Private members
	Usb *pUsb;
	uint8_t currentConfiguration;
	uint8_t currentConnection;
	// Public Methods:
	uint8_t (*IsConfigured)(struct _USB_CDC *pCdc);
//	uint32_t (*Write) (Usb *pUsb, const char *pData, uint32_t length, uint8_t ep_num);
//	uint32_t (*Read)  (Usb *pUsb, char *pData, uint32_t length);
} USB_CDC, *P_USB_CDC;

/**
 * \brief Initializes the USB module
 *
 * \return Pointer to the USB CDC structure
 */
P_USB_CDC usb_init(void);

void sam_ba_usb_CDC_Enumerate(P_USB_CDC pCdc);

extern USB_CDC sam_ba_cdc;



#endif // CDC_ENUMERATE_H
