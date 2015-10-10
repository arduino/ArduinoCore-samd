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

#ifndef _BOARD_DRIVER_USB_H_
#define _BOARD_DRIVER_USB_H_

#include "sam_ba_cdc.h"

extern UsbDeviceDescriptor usb_endpoint_table[MAX_EP];
extern uint8_t udd_ep_out_cache_buffer[2][64]; //1 for CTRL, 1 for BULK
extern uint8_t udd_ep_in_cache_buffer[2][64]; //1 for CTRL, 1 for BULK

P_USB_CDC USB_Open(P_USB_CDC pCdc, Usb *pUsb);

void USB_Init(void);

uint32_t USB_Write(Usb *pUsb, const char *pData, uint32_t length, uint8_t ep_num);
uint32_t USB_Read(Usb *pUsb, char *pData, uint32_t length);
uint32_t USB_Read_blocking(Usb *pUsb, char *pData, uint32_t length);

uint8_t USB_IsConfigured(P_USB_CDC pCdc);

void USB_SendStall(Usb *pUsb, bool direction_in);
void USB_SendZlp(Usb *pUsb);

void USB_SetAddress(Usb *pUsb, uint16_t wValue);
void USB_Configure(Usb *pUsb);

#endif // _BOARD_DRIVER_USB_H_
