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

#include <stdint.h>
#include <string.h>
#include "sam_ba_usb.h"
#include "board_driver_usb.h"
#include "sam_ba_cdc.h"

__attribute__((__aligned__(4)))
const char devDescriptor[] =
{
  /* Device descriptor */
  0x12,   // bLength
  0x01,   // bDescriptorType
  0x10,   // bcdUSBL
  0x01,   //
  0x02,   // bDeviceClass:    CDC class code
  0x00,   // bDeviceSubclass: CDC class sub code
  0x00,   // bDeviceProtocol: CDC Device protocol
  0x40,   // bMaxPacketSize0
  0x41,   // idVendorL
  0x23,   //
  USB_PID_LOW,   // idProductL
  USB_PID_HIGH,   //
  0x10,   // bcdDeviceL
  0x01,   //
  0x00,   // iManufacturer    // 0x01
  0x00,   // iProduct
  0x00,   // SerialNumber
  0x01    // bNumConfigs
};

__attribute__((__aligned__(4)))
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
  0xC0,   // CbmAttributes Bus powered without remote wakeup: 0xc0, Self powered with remote wakeup: 0xa0
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
      if (wValue == 0x100)
        /* Return Device Descriptor */
        USB_Write(pCdc->pUsb, devDescriptor, SAM_BA_MIN(sizeof(devDescriptor), wLength), USB_EP_CTRL);
      else if (wValue == 0x200)
        /* Return Configuration Descriptor */
        USB_Write(pCdc->pUsb, cfgDescriptor, SAM_BA_MIN(sizeof(cfgDescriptor), wLength), USB_EP_CTRL);
      else
        /* Stall the request */
        USB_SendStall(pUsb, true);
    break;

    case STD_SET_ADDRESS:
      /* Send ZLP */
      USB_SendZlp(pUsb);
      /* Set device address to the newly received address from host */
      pUsb->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | wValue;
    break;

    case STD_SET_CONFIGURATION:
      /* Store configuration */
      pCdc->currentConfiguration = (uint8_t)wValue;
      /* Send ZLP */
      USB_SendZlp(pUsb);
      /* Configure BULK OUT endpoint for CDC Data interface*/
      pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0(3);
      /* Set maximum packet size as 64 bytes */
      usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].PCKSIZE.bit.SIZE = 3;
      pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
      /* Configure the data buffer */
      usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].ADDR.reg = (uint32_t)&udd_ep_out_cache_buffer[1];
      /* Configure BULK IN endpoint for CDC Data interface */
      pUsb->DEVICE.DeviceEndpoint[USB_EP_IN].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE1(3);
      /* Set maximum packet size as 64 bytes */
      usb_endpoint_table[USB_EP_IN].DeviceDescBank[1].PCKSIZE.bit.SIZE = 3;
      pUsb->DEVICE.DeviceEndpoint[USB_EP_IN].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
      /* Configure the data buffer */
      usb_endpoint_table[USB_EP_IN].DeviceDescBank[1].ADDR.reg = (uint32_t)&udd_ep_in_cache_buffer[1];
      /* Configure INTERRUPT IN endpoint for CDC COMM interface*/
      pUsb->DEVICE.DeviceEndpoint[USB_EP_COMM].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE1(4);
      /* Set maximum packet size as 64 bytes */
      usb_endpoint_table[USB_EP_COMM].DeviceDescBank[1].PCKSIZE.bit.SIZE = 0;
      pUsb->DEVICE.DeviceEndpoint[USB_EP_COMM].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
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
