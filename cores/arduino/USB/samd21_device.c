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

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "variant.h"
#include "USB/USB_device.h"
#include "USB/samd21_device.h"
#include "sam.h"

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

#ifdef SAMD_SERIES

//#define TRACE_DEVICE(x)	x
#define TRACE_DEVICE(x)

__attribute__((__aligned__(4))) /*__attribute__((__section__(".bss_hram0")))*/ uint8_t udd_ep_out_cache_buffer[4][64];
__attribute__((__aligned__(4))) /*__attribute__((__section__(".bss_hram0")))*/ uint8_t udd_ep_in_cache_buffer[4][128];

/**
 * USB SRAM data containing pipe descriptor table
 * The content of the USB SRAM can be :
 * - modified by USB hardware interface to update pipe status.
 *   Thereby, it is read by software.
 * - modified by USB software to control pipe.
 *   Thereby, it is read by hardware.
 * This data section is volatile.
 */
 __attribute__((__aligned__(4))) UsbDeviceDescriptor usb_endpoint_table[USB_EPT_NUM];


extern void (*gpf_isr)(void);


void UDD_SetStack(void (*pf_isr)(void))
{
	gpf_isr = pf_isr;
}

// NVM Software Calibration Area Mapping
// USB TRANSN calibration value. Should be written to the USB PADCAL register.
#define NVM_USB_PAD_TRANSN_POS  45
#define NVM_USB_PAD_TRANSN_SIZE 5
// USB TRANSP calibration value. Should be written to the USB PADCAL register.
#define NVM_USB_PAD_TRANSP_POS  50
#define NVM_USB_PAD_TRANSP_SIZE 5
// USB TRIM calibration value. Should be written to the USB PADCAL register.
#define NVM_USB_PAD_TRIM_POS  55
#define NVM_USB_PAD_TRIM_SIZE 3

void UDD_Init(void)
{
	uint32_t pad_transn;
    uint32_t pad_transp;
    uint32_t pad_trim;
	uint32_t i;

	/* Enable USB clock */
	PM->APBBMASK.reg |= PM_APBBMASK_USB;

	/* Set up the USB DP/DN pins */
	PORT->Group[0].PINCFG[PIN_PA24G_USB_DM].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[PIN_PA24G_USB_DM/2].reg &= ~(0xF << (4 * (PIN_PA24G_USB_DM & 0x01u)));
	PORT->Group[0].PMUX[PIN_PA24G_USB_DM/2].reg |= MUX_PA24G_USB_DM << (4 * (PIN_PA24G_USB_DM & 0x01u));
	PORT->Group[0].PINCFG[PIN_PA25G_USB_DP].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[PIN_PA25G_USB_DP/2].reg &= ~(0xF << (4 * (PIN_PA25G_USB_DP & 0x01u)));
	PORT->Group[0].PMUX[PIN_PA25G_USB_DP/2].reg |= MUX_PA25G_USB_DP << (4 * (PIN_PA25G_USB_DP & 0x01u));

	/* ----------------------------------------------------------------------------------------------
	 * Put Generic Clock Generator 0 as source for Generic Clock Multiplexer 6 (USB reference)
	 */
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( 6 ) | // Generic Clock Multiplexer 6
					GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is source
					GCLK_CLKCTRL_CLKEN ;

	while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
	{
	/* Wait for synchronization */
	}

	/* Reset */
	USB->DEVICE.CTRLA.bit.SWRST = 1;
	while (USB->DEVICE.SYNCBUSY.bit.SWRST) {
		/* Sync wait */
	}

	udd_enable();

	/* Load Pad Calibration */
	pad_transn =( *((uint32_t *)(NVMCTRL_OTP4)  // Non-Volatile Memory Controller
	+ (NVM_USB_PAD_TRANSN_POS / 32))
	>> (NVM_USB_PAD_TRANSN_POS % 32))
	& ((1 << NVM_USB_PAD_TRANSN_SIZE) - 1);

	if (pad_transn == 0x1F) {  // maximum value (31)
		pad_transn = 5;
	}

	USB->DEVICE.PADCAL.bit.TRANSN = pad_transn;

	pad_transp =( *((uint32_t *)(NVMCTRL_OTP4)
	+ (NVM_USB_PAD_TRANSP_POS / 32))
	>> (NVM_USB_PAD_TRANSP_POS % 32))
	& ((1 << NVM_USB_PAD_TRANSP_SIZE) - 1);

	if (pad_transp == 0x1F) {  // maximum value (31)
		pad_transp = 29;
	}

	USB->DEVICE.PADCAL.bit.TRANSP = pad_transp;

	pad_trim =( *((uint32_t *)(NVMCTRL_OTP4)
	+ (NVM_USB_PAD_TRIM_POS / 32))
	>> (NVM_USB_PAD_TRIM_POS % 32))
	& ((1 << NVM_USB_PAD_TRIM_SIZE) - 1);

	if (pad_trim == 0x7) {  // maximum value (7)
		pad_trim = 3;
	}

	USB->DEVICE.PADCAL.bit.TRIM = pad_trim;

	/* Set the configuration */
	udd_force_device_mode();
	udd_device_run_in_standby();
    // Set address of USB SRAM
	USB->DEVICE.DESCADD.reg = (uint32_t)(&usb_endpoint_table[0]);
	// For USB_SPEED_FULL
	udd_force_full_speed();
 	for (i = 0; i < sizeof(usb_endpoint_table); i++) {
 		(*(uint32_t *)(&usb_endpoint_table[0]+i)) = 0;
 	}

	// Configure interrupts
	NVIC_SetPriority((IRQn_Type) USB_IRQn, 0UL);
	NVIC_EnableIRQ((IRQn_Type) USB_IRQn);
}

void UDD_Attach(void)
{
	TRACE_DEVICE(printf("=> UDD_Attach\r\n");)

    // Authorize attach if Vbus is present
	udd_attach_device();

	// Enable USB line events
	udd_enable_reset_interrupt();

    // usefull for debug
    udd_enable_sof_interrupt();
}

void UDD_Detach(void)
{
	TRACE_DEVICE(printf("=> UDD_Detach\r\n");)
    udd_detach_device();
}

void send_zlp(void)
{
	/* Set the byte count as zero */
	usb_endpoint_table[0].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
}


void UDD_InitEP( uint32_t ul_ep_nb, uint32_t ul_ep_cfg )
{
	TRACE_DEVICE(printf("=> UDD_InitEP : init EP %lu\r\n", (unsigned long)ul_ep_nb);)

	if( ul_ep_cfg == (USB_ENDPOINT_TYPE_INTERRUPT | USB_ENDPOINT_IN(0)) )
	{
		USB->DEVICE.DeviceEndpoint[ul_ep_nb].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE1(4);
		/* Set maximum packet size as 8 bytes */
		usb_endpoint_table[ul_ep_nb].DeviceDescBank[1].PCKSIZE.bit.SIZE = 0;  // 8 bytes
		/* Configure the data buffer */
		usb_endpoint_table[ul_ep_nb].DeviceDescBank[1].ADDR.reg = (uint32_t)&udd_ep_in_cache_buffer[ul_ep_nb];
	}
	else if( ul_ep_cfg == (USB_ENDPOINT_TYPE_BULK | USB_ENDPOINT_OUT(0)) )
	{
		/* Configure BULK OUT endpoint for CDC Data interface*/
		USB->DEVICE.DeviceEndpoint[ul_ep_nb].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0(3);
		/* Set maximum packet size as 64 bytes */
		usb_endpoint_table[ul_ep_nb].DeviceDescBank[0].PCKSIZE.bit.SIZE = 0x3;  // for 64 bytes
		/* Configure the data buffer */
		usb_endpoint_table[ul_ep_nb].DeviceDescBank[0].ADDR.reg = (uint32_t)&udd_ep_out_cache_buffer[ul_ep_nb];

		usb_endpoint_table[ul_ep_nb].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 64;
		usb_endpoint_table[ul_ep_nb].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
		// NACK if not ready
		udd_OUT_transfer_allowed(ul_ep_nb);
	}
	else if( ul_ep_cfg == (USB_ENDPOINT_TYPE_BULK | USB_ENDPOINT_IN(0)) )
	{
		/* Configure BULK IN endpoint for CDC Data interface */
		USB->DEVICE.DeviceEndpoint[ul_ep_nb].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE1(3);
		/* Set maximum packet size as 64 bytes */
		usb_endpoint_table[ul_ep_nb].DeviceDescBank[1].PCKSIZE.bit.SIZE = 0x3;  // for 64 bytes
		/* Configure the data buffer */
		usb_endpoint_table[ul_ep_nb].DeviceDescBank[1].ADDR.reg = (uint32_t)&udd_ep_in_cache_buffer[ul_ep_nb];
		// NACK if not ready
		udd_IN_stop_transfer(ul_ep_nb);
	}
	else if( ul_ep_cfg == USB_ENDPOINT_TYPE_CONTROL )
	{
		/* Configure CONTROL endpoint */
		USB->DEVICE.DeviceEndpoint[ul_ep_nb].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0(1) | USB_DEVICE_EPCFG_EPTYPE1(1);
		udd_OUT_stop_transfer(ul_ep_nb);
		udd_IN_stop_transfer(ul_ep_nb);

		usb_endpoint_table[ul_ep_nb].DeviceDescBank[0].PCKSIZE.reg &= ~USB_DEVICE_PCKSIZE_AUTO_ZLP;
		usb_endpoint_table[ul_ep_nb].DeviceDescBank[1].PCKSIZE.reg &= ~USB_DEVICE_PCKSIZE_AUTO_ZLP;

		/* Set maximum packet size as 64 bytes */
		usb_endpoint_table[ul_ep_nb].DeviceDescBank[0].PCKSIZE.bit.SIZE = 0x3;  // for 64 bytes
		usb_endpoint_table[ul_ep_nb].DeviceDescBank[1].PCKSIZE.bit.SIZE = 0x3;  // for 64 bytes

		/* get endpoint configuration from setting register */
		usb_endpoint_table[ul_ep_nb].DeviceDescBank[0].ADDR.reg = (uint32_t)&udd_ep_out_cache_buffer[0];
		usb_endpoint_table[ul_ep_nb].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 8;
		usb_endpoint_table[ul_ep_nb].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;

		// NACK if not ready
		udd_OUT_stop_transfer(ul_ep_nb);
		udd_IN_stop_transfer(ul_ep_nb);
	}
}


// Send packet.
void UDD_ClearIN(void)
{
	udd_IN_transfer_allowed(EP0);
}



uint32_t UDD_Send(uint32_t ep, const void* data, uint32_t len)
{
	memcpy(&udd_ep_in_cache_buffer[ep], data, len);

	/* Get endpoint configuration from setting register */
	usb_endpoint_table[ep].DeviceDescBank[1].ADDR.reg = (uint32_t)&udd_ep_in_cache_buffer[ep];
	usb_endpoint_table[ep].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
	usb_endpoint_table[ep].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = len;

	return len;
}

uint8_t UDD_Recv_data(uint32_t ep, uint32_t len)
{
	TRACE_DEVICE(printf("=> UDD_Recvdata : ep=%d\r\n", (char)ep);)

	if (len>64) len=64;
	usb_endpoint_table[ep].DeviceDescBank[0].ADDR.reg = (uint32_t)&udd_ep_out_cache_buffer[ep];
	usb_endpoint_table[ep].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = len;
	usb_endpoint_table[ep].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
	udd_OUT_transfer_allowed(ep);
	TRACE_DEVICE(printf("=> UDD_Recv_data : data=%lu\r\n", (unsigned long)data);)

	/* Wait for transfer to complete */
	while (!udd_is_OUT_transf_cplt(ep));
	/* Clear Transfer complete 0 flag */
	udd_clear_OUT_transf_cplt(ep);

	return udd_ep_out_cache_buffer[ep][0];
}

void UDD_Recv(uint32_t ep, uint8_t** ppData)
{
	*ppData = udd_ep_out_cache_buffer[ep];
}

void UDD_Stall(uint32_t ep)
{
	uint8_t ep_num = ep;

	// Stall endpoint
	USB->DEVICE.DeviceEndpoint[ep_num].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ(2);
}

uint32_t UDD_FifoByteCount(uint32_t ep)
{
	return ((uint16_t)(usb_endpoint_table[ep].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT));
}

void UDD_ReleaseRX(uint32_t ep)
{
	TRACE_DEVICE(puts("=> UDD_ReleaseRX\r\n");)
	// The RAM Buffer is empty: we can receive data
	udd_OUT_transfer_allowed(ep);
	/* Clear Transfer complete 0 flag */
	udd_clear_OUT_transf_cplt(ep);
}

void UDD_ReleaseTX(uint32_t ep)
{
	TRACE_DEVICE(printf("=> UDD_ReleaseTX ep=%lu\r\n", (unsigned long)ep);)
	// The RAM Buffer is full: we can send data
    udd_IN_transfer_allowed(ep);
 	/* Clear the transfer complete flag  */
    udd_clear_IN_transf_cplt(ep);
}

void UDD_SetAddress(uint32_t addr)
{
	TRACE_DEVICE(printf("=> UDD_SetAddress : setting address to %lu\r\n", (unsigned long)addr);)

	/* Set the byte count as zero */
	usb_endpoint_table[0].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
 	/* Clear the transfer complete flag  */
    udd_clear_IN_transf_cplt(0);
	/* Set the bank as ready */
	udd_IN_transfer_allowed(0);

	/* Wait for transfer to complete */
	while (!udd_is_IN_transf_cplt(EP0)) {}

	udd_configure_address(addr);
}

uint32_t UDD_GetFrameNumber(void)
{
	return udd_frame_number();
}

#ifdef __cplusplus
}
#endif

#endif /* SAMD_SERIES */

