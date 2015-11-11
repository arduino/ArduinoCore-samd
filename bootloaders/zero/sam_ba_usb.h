/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.

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

#define USB_EP_CTRL             (0u)
#define USB_EP_OUT              (2u)
#define USB_EP_OUT_SIZE         (0x40u)
#define USB_EP_IN               (1u)
#define USB_EP_IN_SIZE          (0x40u)
#define USB_EP_COMM             (3u)
#define MAX_EP                  (4u)

/* USB standard request code */
#define STD_GET_STATUS_ZERO            (0x0080u)
#define STD_GET_STATUS_INTERFACE       (0x0081u)
#define STD_GET_STATUS_ENDPOINT        (0x0082u)

#define STD_CLEAR_FEATURE_ZERO         (0x0100u)
#define STD_CLEAR_FEATURE_INTERFACE    (0x0101u)
#define STD_CLEAR_FEATURE_ENDPOINT     (0x0102u)

#define STD_SET_FEATURE_ZERO           (0x0300u)
#define STD_SET_FEATURE_INTERFACE      (0x0301u)
#define STD_SET_FEATURE_ENDPOINT       (0x0302u)

#define STD_SET_ADDRESS                (0x0500u)
#define STD_GET_DESCRIPTOR             (0x0680u)
#define STD_SET_DESCRIPTOR             (0x0700u)
#define STD_GET_CONFIGURATION          (0x0880u)
#define STD_SET_CONFIGURATION          (0x0900u)
#define STD_GET_INTERFACE              (0x0A81u)
#define STD_SET_INTERFACE              (0x0B01u)
#define STD_SYNCH_FRAME                (0x0C82u)

#define STD_GET_DESCRIPTOR_DEVICE                          (1u)
#define STD_GET_DESCRIPTOR_CONFIGURATION                   (2u)
#define STD_GET_DESCRIPTOR_STRING                          (3u)
#define STD_GET_DESCRIPTOR_INTERFACE                       (4u)
#define STD_GET_DESCRIPTOR_ENDPOINT                        (5u)
#define STD_GET_DESCRIPTOR_DEVICE_QUALIFIER                (6u)
#define STD_GET_DESCRIPTOR_OTHER_SPEED_CONFIGURATION       (7u)
#define STD_GET_DESCRIPTOR_INTERFACE_POWER1                (8u)

#define FEATURE_ENDPOINT_HALT          (0u)
#define FEATURE_DEVICE_REMOTE_WAKEUP   (1u)
#define FEATURE_TEST_MODE              (2u)

#if 0 // TODO: pending validation
#define STRING_INDEX_LANGUAGES         (0x00u)
#define STRING_INDEX_MANUFACTURER      (0x01u)
#define STRING_INDEX_PRODUCT           (0x02u)
#endif // 0

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

#if 0 // TODO: pending validation
uint32_t USB_SendString(Usb *pUsb, const char* ascii_string, uint8_t length, uint8_t maxLength);
#endif // 0

extern USB_CDC sam_ba_cdc;



#endif // CDC_ENUMERATE_H
