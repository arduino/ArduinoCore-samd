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

#ifndef USB_DEVICE_H_INCLUDED
#define USB_DEVICE_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "USB/samd21_device.h"


// bEndpointAddress in Endpoint Descriptor
#define USB_ENDPOINT_DIRECTION_MASK            0x80
#define USB_ENDPOINT_OUT(addr)                 ((addr) | 0x00)
#define USB_ENDPOINT_IN(addr)                  ((addr) | 0x80)

#define USB_ENDPOINT_TYPE_MASK                 0x03
#define USB_ENDPOINT_TYPE_CONTROL              0x00
#define USB_ENDPOINT_TYPE_ISOCHRONOUS          0x01
#define USB_ENDPOINT_TYPE_BULK                 0x02
#define USB_ENDPOINT_TYPE_INTERRUPT            0x03


extern void UDD_ClearIN(void);
extern uint32_t UDD_FifoByteCount(uint32_t ep);
extern void UDD_ReleaseRX(uint32_t ep);
extern void UDD_ReleaseTX(uint32_t ep);
extern uint32_t UDD_Send(uint32_t ep, const void* data, uint32_t len);
extern uint8_t UDD_Recv_data(uint32_t ep, uint32_t len);
extern void UDD_Recv(uint32_t ep, uint8_t** data);
extern void UDD_Init(void);
extern void UDD_InitEP( uint32_t ul_ep, uint32_t ul_ep_cfg );
extern void send_zlp (void);
extern void UDD_Attach(void);
extern void UDD_Detach(void);
extern void UDD_SetAddress(uint32_t addr);
extern void UDD_Stall(uint32_t ep);
extern uint32_t UDD_GetFrameNumber(void);
extern void UDD_SetStack(void (*pf_isr)(void)); 

#ifdef __cplusplus
}
#endif

#endif /* USB_DEVICE_H_INCLUDED */
