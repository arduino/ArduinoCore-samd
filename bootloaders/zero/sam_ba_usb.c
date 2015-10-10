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

#include <stdint.h>
#include <string.h>
#include "sam_ba_usb.h"
#include "board_driver_usb.h"
#include "sam_ba_cdc.h"

/* This data array will be copied into SRAM as its length is inferior to 64 bytes,
 * and so can stay in flash.
 */
static __attribute__((__aligned__(4)))
const char devDescriptor[] =
{
  /* Device descriptor */
  0x12,   // bLength
  0x01,   // bDescriptorType
  0x00,   // bcdUSB L
  0x02,   // bcdUSB H
  0x02,   // bDeviceClass:    CDC class code
  0x00,   // bDeviceSubclass: CDC class sub code
  0x00,   // bDeviceProtocol: CDC Device protocol
  0x40,   // bMaxPacketSize0
  0x41,   // idVendor L
  0x23,   // idVendor H
  USB_PID_LOW,   // idProduct L
  USB_PID_HIGH,  // idProduct H
  0x00,   // bcdDevice L, here matching SAM-BA version
  0x02,   // bcdDevice H
#if 0 // TODO: pending validation
  STRING_INDEX_MANUFACTURER,   // iManufacturer
  STRING_INDEX_PRODUCT,        // iProduct
#else
  0x00,   // iManufacturer
  0x00,   // iProduct
#endif // 0
  0x00,   // SerialNumber, should be based on product unique ID
  0x01    // bNumConfigs
};

/* This data array will be consumed directly by USB_Write() and must be in SRAM.
 * We cannot send data from product internal flash.
 */
static __attribute__((__aligned__(4)))
char cfgDescriptor[] =
{
  /* ============== CONFIGURATION 1 =========== */
  /* Configuration 1 descriptor */
  0x09,   // CbLength
  0x02,   // CbDescriptorType
  0x43,   // CwTotalLength 2 EP + Control
  0x00,
  0x02,   // CbNumInterfaces
  0x01,   // CbConfigurationValue
  0x00,   // CiConfiguration
  0x80,   // CbmAttributes Bus powered without remote wakeup: 0x80, Self powered without remote wakeup: 0xc0
  0x32,   // CMaxPower, report using 100mA, enough for a bootloader

  /* Communication Class Interface Descriptor Requirement */
  0x09, // bLength
  0x04, // bDescriptorType
  0x00, // bInterfaceNumber
  0x00, // bAlternateSetting
  0x01, // bNumEndpoints
  0x02, // bInterfaceClass
  0x02, // bInterfaceSubclass
  0x00, // bInterfaceProtocol
  0x00, // iInterface

  /* Header Functional Descriptor */
  0x05, // bFunction Length
  0x24, // bDescriptor type: CS_INTERFACE
  0x00, // bDescriptor subtype: Header Func Desc
  0x10, // bcdCDC:1.1
  0x01,

  /* ACM Functional Descriptor */
  0x04, // bFunctionLength
  0x24, // bDescriptor Type: CS_INTERFACE
  0x02, // bDescriptor Subtype: ACM Func Desc
  0x00, // bmCapabilities

  /* Union Functional Descriptor */
  0x05, // bFunctionLength
  0x24, // bDescriptorType: CS_INTERFACE
  0x06, // bDescriptor Subtype: Union Func Desc
  0x00, // bMasterInterface: Communication Class Interface
  0x01, // bSlaveInterface0: Data Class Interface

  /* Call Management Functional Descriptor */
  0x05, // bFunctionLength
  0x24, // bDescriptor Type: CS_INTERFACE
  0x01, // bDescriptor Subtype: Call Management Func Desc
  0x00, // bmCapabilities: D1 + D0
  0x01, // bDataInterface: Data Class Interface 1

  /* Endpoint 1 descriptor */
  0x07,   // bLength
  0x05,   // bDescriptorType
  0x83,   // bEndpointAddress, Endpoint 03 - IN
  0x03,   // bmAttributes      INT
  0x08,   // wMaxPacketSize
  0x00,
  0xFF,   // bInterval

  /* Data Class Interface Descriptor Requirement */
  0x09, // bLength
  0x04, // bDescriptorType
  0x01, // bInterfaceNumber
  0x00, // bAlternateSetting
  0x02, // bNumEndpoints
  0x0A, // bInterfaceClass
  0x00, // bInterfaceSubclass
  0x00, // bInterfaceProtocol
  0x00, // iInterface

  /* First alternate setting */
  /* Endpoint 1 descriptor */
  0x07,   // bLength
  0x05,   // bDescriptorType
  0x81,   // bEndpointAddress, Endpoint 01 - IN
  0x02,   // bmAttributes      BULK
  USB_EP_IN_SIZE,   // wMaxPacketSize
  0x00,
  0x00,   // bInterval

  /* Endpoint 2 descriptor */
  0x07,   // bLength
  0x05,   // bDescriptorType
  0x02,   // bEndpointAddress, Endpoint 02 - OUT
  0x02,   // bmAttributes      BULK
  USB_EP_OUT_SIZE,   // wMaxPacketSize
  0x00,
  0x00    // bInterval
};

#ifndef STRING_MANUFACTURER
#  define STRING_MANUFACTURER "Arduino LLC"
#endif

#ifndef STRING_PRODUCT
#  define STRING_PRODUCT "Arduino Zero"
#endif

USB_CDC sam_ba_cdc;

/*----------------------------------------------------------------------------
 * \brief This function is a callback invoked when a SETUP packet is received
 */
void sam_ba_usb_CDC_Enumerate(P_USB_CDC pCdc)
{
  Usb *pUsb = pCdc->pUsb;
  static volatile uint8_t bmRequestType, bRequest, dir;
  static volatile uint16_t wValue, wIndex, wLength, wStatus;

  /* Clear the Received Setup flag */
  pUsb->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.RXSTP = true;

  /* Read the USB request parameters */
  bmRequestType = udd_ep_out_cache_buffer[0][0];
  bRequest      = udd_ep_out_cache_buffer[0][1];
  wValue        = (udd_ep_out_cache_buffer[0][2] & 0xFF);
  wValue       |= (udd_ep_out_cache_buffer[0][3] << 8);
  wIndex        = (udd_ep_out_cache_buffer[0][4] & 0xFF);
  wIndex       |= (udd_ep_out_cache_buffer[0][5] << 8);
  wLength       = (udd_ep_out_cache_buffer[0][6] & 0xFF);
  wLength      |= (udd_ep_out_cache_buffer[0][7] << 8);

  /* Clear the Bank 0 ready flag on Control OUT */
  pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY;

  /* Handle supported standard device request Cf Table 9-3 in USB specification Rev 1.1 */
  switch ((bRequest << 8) | bmRequestType)
  {
    case STD_GET_DESCRIPTOR:
      if (wValue>>8 == STD_GET_DESCRIPTOR_DEVICE)
      {
        /* Return Device Descriptor */
        USB_Write(pCdc->pUsb, devDescriptor, SAM_BA_MIN(sizeof(devDescriptor), wLength), USB_EP_CTRL);
      }
      else
      {
        if (wValue>>8 == STD_GET_DESCRIPTOR_CONFIGURATION)
        {
          /* Return Configuration Descriptor */
          USB_Write(pCdc->pUsb, cfgDescriptor, SAM_BA_MIN(sizeof(cfgDescriptor), wLength), USB_EP_CTRL);
        }
        else
        {
#if 0 // TODO: pending validation
          if (wValue>>8 == STD_GET_DESCRIPTOR_STRING)
          {
            switch ( wValue & 0xff )
            {
              case STRING_INDEX_LANGUAGES:
                uint16_t STRING_LANGUAGE[2] = { (STD_GET_DESCRIPTOR_STRING<<8) | 4, 0x0409 };

                USB_Write(pCdc->pUsb, (const char*)STRING_LANGUAGE, SAM_BA_MIN(sizeof(STRING_LANGUAGE), wLength), USB_EP_CTRL);
              break;

              case STRING_INDEX_MANUFACTURER:
                USB_SendString(pCdc->pUsb, STRING_MANUFACTURER, strlen(STRING_MANUFACTURER), wLength );
              break;

              case STRING_INDEX_PRODUCT:
                USB_SendString(pCdc->pUsb, STRING_PRODUCT, strlen(STRING_PRODUCT), wLength );
              break;
              default:
                /* Stall the request */
                USB_SendStall(pUsb, true);
              break;
            }
          }
          else
#endif // 0
          {
            /* Stall the request */
            USB_SendStall(pUsb, true);
          }
        }
      }
    break;

    case STD_SET_ADDRESS:
      /* Send ZLP */
      USB_SendZlp(pUsb);
      /* Set device address to the newly received address from host */
      USB_SetAddress(pCdc->pUsb, wValue);
    break;

    case STD_SET_CONFIGURATION:
      /* Store configuration */
      pCdc->currentConfiguration = (uint8_t)wValue;

      /* Send ZLP */
      USB_SendZlp(pUsb);

      /* Configure the 3 needed endpoints */
      USB_Configure(pUsb);
    break;

    case STD_GET_CONFIGURATION:
      /* Return current configuration value */
      USB_Write(pCdc->pUsb, (char *) &(pCdc->currentConfiguration), sizeof(pCdc->currentConfiguration), USB_EP_CTRL);
    break;

    case STD_GET_STATUS_ZERO:
      wStatus = 0;
      USB_Write(pCdc->pUsb, (char *) &wStatus, sizeof(wStatus), USB_EP_CTRL);
    break;

    case STD_GET_STATUS_INTERFACE:
      wStatus = 0;
      USB_Write(pCdc->pUsb, (char *) &wStatus, sizeof(wStatus), USB_EP_CTRL);
    break;

    case STD_GET_STATUS_ENDPOINT:
      wStatus = 0;
      dir = wIndex & 80;
      wIndex &= 0x0F;
      if (wIndex <= 3)
      {
        if (dir)
        {
          //wStatus = (pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUS.reg & USB_DEVICE_EPSTATUSSET_STALLRQ1) ? 1 : 0;
          wStatus = (pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUS.bit.STALLRQ & (1<<1)) ? 1 : 0;
        }
        else
        {
          //wStatus = (pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUS.reg & USB_DEVICE_EPSTATUSSET_STALLRQ0) ? 1 : 0;
          wStatus = (pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUS.bit.STALLRQ & (1<<0)) ? 1 : 0;
        }
        /* Return current status of endpoint */
        USB_Write(pCdc->pUsb, (char *) &wStatus, sizeof(wStatus), USB_EP_CTRL);
      }
      else
      {
        /* Stall the request */
        USB_SendStall(pUsb, true);
      }
    break;

    case STD_SET_FEATURE_ZERO:
      /* Stall the request */
      USB_SendStall(pUsb, true);
    break;

    case STD_SET_FEATURE_INTERFACE:
      /* Send ZLP */
      USB_SendZlp(pUsb);
    break;

    case STD_SET_FEATURE_ENDPOINT:
      dir = wIndex & 0x80;
      wIndex &= 0x0F;
      if ((wValue == 0) && wIndex && (wIndex <= 3))
      {
        /* Set STALL request for the endpoint */
        if (dir)
        {
          //pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ1;
          pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSSET.bit.STALLRQ = (1<<1);
        }
        else
        {
          //pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ0;
          pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSSET.bit.STALLRQ = (1<<0);
        }

        /* Send ZLP */
        USB_SendZlp(pUsb);
      }
      else
      {
        /* Stall the request */
        USB_SendStall(pUsb, true);
      }
    break;

    case STD_SET_INTERFACE:
    case STD_CLEAR_FEATURE_ZERO:
      /* Stall the request */
      USB_SendStall(pUsb, true);
    break;

    case STD_CLEAR_FEATURE_INTERFACE:
      /* Send ZLP */
      USB_SendZlp(pUsb);
    break;

    case STD_CLEAR_FEATURE_ENDPOINT:
      dir = wIndex & 0x80;
      wIndex &= 0x0F;

      if ((wValue == 0) && wIndex && (wIndex <= 3))
      {
        if (dir)
        {
          if (pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUS.bit.STALLRQ & (1<<1))
          {
            // Remove stall request
            //pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_STALLRQ1;
            pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSCLR.bit.STALLRQ = (1<<1);
            if (pUsb->DEVICE.DeviceEndpoint[wIndex].EPINTFLAG.bit.STALL & (1<<1))
            {
              pUsb->DEVICE.DeviceEndpoint[wIndex].EPINTFLAG.bit.STALL = (1<<1);
              // The Stall has occurred, then reset data toggle
              pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSSET_DTGLIN;
            }
          }
        }
        else
        {
          if (pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUS.bit.STALLRQ & (1<<0))
          {
            // Remove stall request
            //pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_STALLRQ0;
            pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSCLR.bit.STALLRQ = (1<<0);
            if (pUsb->DEVICE.DeviceEndpoint[wIndex].EPINTFLAG.bit.STALL & (1<<0))
            {
              pUsb->DEVICE.DeviceEndpoint[wIndex].EPINTFLAG.bit.STALL = (1<<0);
              // The Stall has occurred, then reset data toggle
              pUsb->DEVICE.DeviceEndpoint[wIndex].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSSET_DTGLOUT;
            }
          }
        }
        /* Send ZLP */
        USB_SendZlp(pUsb);
      }
      else
      {
        USB_SendStall(pUsb, true);
      }
    break;

    // handle CDC class requests
    case SET_LINE_CODING:
      /* Send ZLP */
      USB_SendZlp(pUsb);
    break;

    case GET_LINE_CODING:
      /* Send current line coding */
      USB_Write(pCdc->pUsb, (char *) &line_coding, SAM_BA_MIN(sizeof(usb_cdc_line_coding_t), wLength), USB_EP_CTRL);
    break;

    case SET_CONTROL_LINE_STATE:
      /* Store the current connection */
      pCdc->currentConnection = wValue;
      /* Send ZLP */
      USB_SendZlp(pUsb);
    break;

    default:
      /* Stall the request */
      USB_SendStall(pUsb, true);
    break;
  }
}

/*----------------------------------------------------------------------------
 * \brief
 */
P_USB_CDC usb_init(void)
{
  sam_ba_cdc.pUsb = USB;

  /* Initialize USB */
  USB_Init();
  /* Get the default CDC structure settings */
  USB_Open(&sam_ba_cdc, sam_ba_cdc.pUsb);

  return &sam_ba_cdc;
}

#if 0 // TODO: pending validation
/*----------------------------------------------------------------------------
 * \brief Send a USB descriptor string.
 *
 * The input string is plain ASCII but is sent out as UTF-16 with the correct 2-byte prefix.
 */
uint32_t USB_SendString(Usb *pUsb, const char* ascii_string, uint8_t length, uint8_t maxLength)
{
  uint8_t string_descriptor[255]; // Max USB-allowed string length
  uint16_t* unicode_string=(uint16_t*)(string_descriptor+2); // point on 3 bytes of descriptor

  int resulting_length = 1;

  for ( ; *ascii_string && (length>=0) && (resulting_length<(maxLength>>1)) ; ascii_string++, length--, resulting_length++ )
  {
    *unicode_string++ = (uint16_t)(*ascii_string);
  }

  string_descriptor[0] = (resulting_length<<1);
  string_descriptor[1] = STD_GET_DESCRIPTOR_STRING;

  return USB_Write(pUsb, (const char*)unicode_string, resulting_length, USB_EP_CTRL);
}
#endif // 0
