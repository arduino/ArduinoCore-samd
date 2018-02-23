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

#include <string.h>
#include "board_driver_usb.h"
#include "sam_ba_usb.h"
#include "sam_ba_cdc.h"

#if (SAMD51)
  #define NVM_CALIBRATION_ADDRESS           NVMCTRL_SW0
  #define NVM_USB_PAD_TRANSN_POS            (32)
  #define NVM_USB_PAD_TRANSP_POS            (37)
  #define NVM_USB_PAD_TRIM_POS              (42)
#elif (SAML21)
  #define NVM_CALIBRATION_ADDRESS           NVMCTRL_OTP5
  #define NVM_USB_PAD_TRANSN_POS            (13)
  #define NVM_USB_PAD_TRANSP_POS            (18)
  #define NVM_USB_PAD_TRIM_POS              (23)
#else
  #define NVM_CALIBRATION_ADDRESS           NVMCTRL_OTP4
  #define NVM_USB_PAD_TRANSN_POS            (45)
  #define NVM_USB_PAD_TRANSP_POS            (50)
  #define NVM_USB_PAD_TRIM_POS              (55)
#endif
#define USB_PAD_TRANSN_REG_POS            (6)
#define NVM_USB_PAD_TRANSN_SIZE           (5)
#define USB_PAD_TRANSP_REG_POS            (0)
#define NVM_USB_PAD_TRANSP_SIZE           (5)
#define USB_PAD_TRIM_REG_POS              (12)
#define NVM_USB_PAD_TRIM_SIZE             (3)

/* Generic Clock Multiplexer IDs */
#if (SAMD21 || SAMD11)
  #define GCM_USB                   (0x06U)
#elif (SAML21)
  #define GCM_USB                   (0x04U)
#elif (SAMD51)
  #define GCM_USB                   (0x0AU)
#else
  #error "board_driver_usb.c: Missing dependency or unsupported chip. Please install CMSIS-Atmel from MattairTech (see Prerequisites for Building in README.md)."
#endif


__attribute__((__aligned__(4))) UsbDeviceDescriptor usb_endpoint_table[MAX_EP]; // Initialized to zero in USB_Init
__attribute__((__aligned__(4))) uint8_t udd_ep_out_cache_buffer[2][64]; //1 for CTRL, 1 for BULK
__attribute__((__aligned__(4))) uint8_t udd_ep_in_cache_buffer[2][64]; //1 for CTRL, 1 for BULK

static volatile bool read_job = false;

/*----------------------------------------------------------------------------
 * \brief
 */
P_USB_CDC USB_Open(P_USB_CDC pCdc, Usb *pUsb)
{
  pCdc->pUsb = pUsb;
  pCdc->currentConfiguration = 0;
  pCdc->currentConnection    = 0;
  pCdc->IsConfigured = USB_IsConfigured;
//  pCdc->Write        = USB_Write;
//  pCdc->Read         = USB_Read;

  pCdc->pUsb->DEVICE.CTRLA.bit.ENABLE = true;

  return pCdc;
}

/*----------------------------------------------------------------------------
 * \brief Initializes USB
 */
void USB_Init(void)
{
  uint8_t pad_transn, pad_transp, pad_trim;

  /* Enable USB clock */
#if (SAMD21 || SAMD11)
  PM->APBBMASK.reg |= PM_APBBMASK_USB;
#elif (SAML21 || SAMD51)
  MCLK->APBBMASK.reg |= MCLK_APBBMASK_USB;
#endif

  /* Set up the USB DP/DN pins */
#if (SAMD51)
  pinMux(PINMUX_PA24H_USB_DM);
  pinMux(PINMUX_PA25H_USB_DP);
#else
  pinMux(PINMUX_PA24G_USB_DM);
  pinMux(PINMUX_PA25G_USB_DP);
#endif

  /* ----------------------------------------------------------------------------------------------
   * Put Generic Clock Generator 0 as source for Generic Clock Multiplexer 6 (USB reference)
   */
#if (SAMD21 || SAMD11)
  GCLK->CLKCTRL.reg = ( GCLK_CLKCTRL_ID( GCM_USB ) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN );
  waitForSync();
#elif (SAMD51 && (VARIANT_MCK == 120000000ul))
  GCLK->PCHCTRL[GCM_USB].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK5 );  // use 48MHz clock (required for USB) from GCLK5, which was setup in board_init.c
  while ( (GCLK->PCHCTRL[GCM_USB].reg & GCLK_PCHCTRL_CHEN) == 0 );        // wait for sync
#else
  GCLK->PCHCTRL[GCM_USB].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0 );
  while ( (GCLK->PCHCTRL[GCM_USB].reg & GCLK_PCHCTRL_CHEN) == 0 );        // wait for sync
#endif

  /* Reset */
#if defined(PARANOIA)
  USB->DEVICE.CTRLA.bit.SWRST = 1;
  while (USB->DEVICE.SYNCBUSY.bit.SWRST)
  {
    /* Sync wait */
  }
#endif

  /* Load Pad Calibration */
  pad_transn =(uint8_t)( *((uint32_t *)(NVM_CALIBRATION_ADDRESS)
      + (NVM_USB_PAD_TRANSN_POS / 32))
    >> (NVM_USB_PAD_TRANSN_POS % 32))
    & ((1 << NVM_USB_PAD_TRANSN_SIZE) - 1);

  if (pad_transn == 0x1F)
  {
    pad_transn = 5;
  }

  pad_transp =(uint8_t)( *((uint32_t *)(NVM_CALIBRATION_ADDRESS)
      + (NVM_USB_PAD_TRANSP_POS / 32))
      >> (NVM_USB_PAD_TRANSP_POS % 32))
      & ((1 << NVM_USB_PAD_TRANSP_SIZE) - 1);

  if (pad_transp == 0x1F)
  {
    pad_transp = 29;
  }

  pad_trim =(uint8_t)( *((uint32_t *)(NVM_CALIBRATION_ADDRESS)
      + (NVM_USB_PAD_TRIM_POS / 32))
      >> (NVM_USB_PAD_TRIM_POS % 32))
      & ((1 << NVM_USB_PAD_TRIM_SIZE) - 1);

  if (pad_trim == 0x7)
  {
    pad_trim = 3;
  }

  USB->DEVICE.PADCAL.reg = (uint16_t)((pad_trim << USB_PAD_TRIM_REG_POS) | (pad_transn << USB_PAD_TRANSN_REG_POS) | (pad_transp << USB_PAD_TRANSP_REG_POS));

  /* Set the configuration */
  /* Set mode to Device mode */
  USB->DEVICE.CTRLA.bit.MODE = 0;
  /* Enable Run in Standby */
  USB->DEVICE.CTRLA.bit.RUNSTDBY = true;
  /* Set the descriptor address */
  USB->DEVICE.DESCADD.reg = (uint32_t)(&usb_endpoint_table[0]);
  /* Set speed configuration to Full speed */
  USB->DEVICE.CTRLB.bit.SPDCONF = USB_DEVICE_CTRLB_SPDCONF_FS_Val;
  /* Attach to the USB host */
  USB->DEVICE.CTRLB.reg &= ~USB_DEVICE_CTRLB_DETACH;

  /* Initialize endpoint table RAM location to a known value 0 */
  memset((uint8_t *)(&usb_endpoint_table[0]), 0, sizeof(usb_endpoint_table));
}

uint32_t USB_Write(Usb *pUsb, const char *pData, uint32_t length, uint8_t ep_num)
{
  uint32_t data_address;
  uint8_t buf_index;

  /* Set buffer index */
  buf_index = (ep_num == 0) ? 0 : 1;

  /* Check for requirement for multi-packet or auto zlp */
  if (length >= (1 << (usb_endpoint_table[ep_num].DeviceDescBank[1].PCKSIZE.bit.SIZE + 3)))
  {
    /* Update the EP data address */
    data_address = (uint32_t) pData;
    /* Enable auto zlp */
    usb_endpoint_table[ep_num].DeviceDescBank[1].PCKSIZE.bit.AUTO_ZLP = true;
  }
  else
  {
    /* Copy to local buffer */
    memcpy(udd_ep_in_cache_buffer[buf_index], pData, length);
    /* Update the EP data address */
    data_address = (uint32_t) &udd_ep_in_cache_buffer[buf_index];
  }

  /* Set the buffer address for ep data */
  usb_endpoint_table[ep_num].DeviceDescBank[1].ADDR.reg = data_address;
  /* Set the byte count as zero */
  usb_endpoint_table[ep_num].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = length;
  /* Set the multi packet size as zero for multi-packet transfers where length > ep size */
  usb_endpoint_table[ep_num].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
  /* Clear the transfer complete flag  */
  pUsb->DEVICE.DeviceEndpoint[ep_num].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
  /* Set the bank as ready */
  pUsb->DEVICE.DeviceEndpoint[ep_num].EPSTATUSSET.bit.BK1RDY = true;

  /* Wait for transfer to complete */
  while ( (pUsb->DEVICE.DeviceEndpoint[ep_num].EPINTFLAG.bit.TRCPT1) == 0 );

  return length;
}

/*----------------------------------------------------------------------------
 * \brief Read available data from Endpoint OUT
 */
uint32_t USB_Read(Usb *pUsb, char *pData, uint32_t length)
{
  uint32_t packetSize = 0;

  if (!read_job)
  {
    /* Set the buffer address for ep data */
    usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].ADDR.reg = (uint32_t)&udd_ep_out_cache_buffer[USB_EP_OUT-1];
    /* Set the byte count as zero */
    usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
    /* Set the byte count as zero */
    usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
    /* Start the reception by clearing the bank 0 ready bit */
    pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPSTATUSCLR.bit.BK0RDY = true;
    /* set the user flag */
    read_job = true;
  }

  /* Check for Transfer Complete 0 flag */
  if ( pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPINTFLAG.bit.TRCPT0 )
  {
    /* Set packet size */
    packetSize = SAM_BA_MIN(usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT, length);
    /* Copy read data to user buffer */
    memcpy(pData, udd_ep_out_cache_buffer[USB_EP_OUT-1], packetSize);
    /* Clear the Transfer Complete 0 flag */
    pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0;
    /* Clear the user flag */
    read_job = false;
  }

  return packetSize;
}

uint32_t USB_Read_blocking(Usb *pUsb, char *pData, uint32_t length)
{
  if (read_job)
  {
    /* Stop the reception by setting the bank 0 ready bit */
    pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPSTATUSSET.bit.BK0RDY = true;
    /* Clear the user flag */
    read_job = false;
  }

  /* Set the buffer address for ep data */
  usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].ADDR.reg = ((uint32_t)pData);
  /* Set the byte count as zero */
  usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
  /* Set the multi packet size as zero for multi-packet transfers where length > ep size */
  usb_endpoint_table[USB_EP_OUT].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = length;
  /* Clear the bank 0 ready flag */
  pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPSTATUSCLR.bit.BK0RDY = true;
  /* Wait for transfer to complete */
  while (!( pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPINTFLAG.bit.TRCPT0 ));
  /* Clear Transfer complete 0 flag */
  pUsb->DEVICE.DeviceEndpoint[USB_EP_OUT].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0;

  return length;
}

/*----------------------------------------------------------------------------
 * \brief Test if the device is configured and handle enumeration
 */
uint8_t USB_IsConfigured(P_USB_CDC pCdc)
{
  Usb *pUsb = pCdc->pUsb;

  /* Check for End of Reset flag */
  if (pUsb->DEVICE.INTFLAG.reg & USB_DEVICE_INTFLAG_EORST)
  {
    /* Clear the flag */
    pUsb->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_EORST;
    /* Set Device address as 0 */
    pUsb->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | 0;
    /* Configure endpoint 0 */
    /* Configure Endpoint 0 for Control IN and Control OUT */
    pUsb->DEVICE.DeviceEndpoint[0].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0(1) | USB_DEVICE_EPCFG_EPTYPE1(1);
    pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
    pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
    /* Configure control OUT Packet size to 64 bytes */
    usb_endpoint_table[0].DeviceDescBank[0].PCKSIZE.bit.SIZE = 3;
    /* Configure control IN Packet size to 64 bytes */
    usb_endpoint_table[0].DeviceDescBank[1].PCKSIZE.bit.SIZE = 3;
    /* Configure the data buffer address for control OUT */
    usb_endpoint_table[0].DeviceDescBank[0].ADDR.reg = (uint32_t)&udd_ep_out_cache_buffer[0];
    /* Configure the data buffer address for control IN */
    usb_endpoint_table[0].DeviceDescBank[1].ADDR.reg = (uint32_t)&udd_ep_in_cache_buffer[0];
    /* Set Multipacket size to 8 for control OUT and byte count to 0*/
    usb_endpoint_table[0].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 8;
    usb_endpoint_table[0].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
    pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY;

    // Reset current configuration value to 0
    pCdc->currentConfiguration = 0;
  }
  else
  {
    if (pUsb->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_RXSTP)
    {
      sam_ba_usb_CDC_Enumerate(pCdc);
    }
  }

  return pCdc->currentConfiguration;
}

/*----------------------------------------------------------------------------
 * \brief Stall the control endpoint
 */
void USB_SendStall(Usb *pUsb, bool direction_in)
{
  /* Check the direction */
  if (direction_in)
  {
    /* Set STALL request on IN direction */
    pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
  }
  else
  {
    /* Set STALL request on OUT direction */
    pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ0 = 1;
  }
}

/*----------------------------------------------------------------------------
 * \brief Send zero length packet through the control endpoint
 */
void USB_SendZlp(Usb *pUsb)
{
  /* Set the byte count as zero */
  usb_endpoint_table[0].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
  /* Clear the transfer complete flag  */
  pUsb->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
  /* Set the bank as ready */
  pUsb->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.BK1RDY = true;
  /* Wait for transfer to complete */
  while (!( pUsb->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1 ));
}

/*----------------------------------------------------------------------------
 * \brief Set USB device address obtained from host
 */
void USB_SetAddress(Usb *pUsb, uint16_t wValue)
{
  pUsb->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | wValue;
}

/*----------------------------------------------------------------------------
 * \brief Configure USB device
 */
void USB_Configure(Usb *pUsb)
{
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
}
