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

#include "sam_ba_cdc.h"
#include "board_driver_usb.h"

usb_cdc_line_coding_t line_coding=
{
  115200, // baudrate
  0,      // 1 Stop Bit
  0,      // None Parity
  8     // 8 Data bits
};

#define pCdc (&sam_ba_cdc)

int cdc_putc(/*P_USB_CDC pCdc,*/ int value)
{
  /* Send single byte on USB CDC */
  USB_Write(pCdc->pUsb, (const char *)&value, 1, USB_EP_IN);

  return 1;
}

int cdc_getc(/*P_USB_CDC pCdc*/void)
{
  uint8_t rx_char;

  /* Read singly byte on USB CDC */
  USB_Read(pCdc->pUsb, (char *)&rx_char, 1);

  return (int)rx_char;
}

bool cdc_is_rx_ready(/*P_USB_CDC pCdc*/void)
{
  /* Check whether the device is configured */
  if ( !USB_IsConfigured(pCdc) )
    return 0;

  /* Return transfer complete 0 flag status */
  return (pCdc->pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPINTFLAG.bit.TRCPT0);
}

uint32_t cdc_write_buf(/*P_USB_CDC pCdc,*/ void const* data, uint32_t length)
{
  /* Send the specified number of bytes on USB CDC */
  USB_Write(pCdc->pUsb, (const char *)data, length, USB_EP_IN);
  return length;
}

uint32_t cdc_read_buf(/*P_USB_CDC pCdc,*/ void* data, uint32_t length)
{
  /* Check whether the device is configured */
  if ( !USB_IsConfigured(pCdc) )
    return 0;

  /* Read from USB CDC */
  return USB_Read(pCdc->pUsb, (char *)data, length);
}

uint32_t cdc_read_buf_xmd(/*P_USB_CDC pCdc,*/ void* data, uint32_t length)
{
  /* Check whether the device is configured */
  if ( !USB_IsConfigured(pCdc) )
    return 0;

  /* Blocking read till specified number of bytes is received */
  // XXX: USB_Read_blocking is not reliable
  // return USB_Read_blocking(pCdc, (char *)data, length);

  char *dst = (char *)data;
  uint32_t remaining = length;
  while (remaining)
  {
    uint32_t readed = USB_Read(pCdc->pUsb, (char *)dst, remaining);
    remaining -= readed;
    dst += readed;
  }

  return length;
}
