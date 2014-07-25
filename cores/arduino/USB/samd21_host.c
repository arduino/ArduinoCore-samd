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
// #include <stdint.h>
// #include <string.h>
//
// #include "variant.h"
// #include "USB_host.h"
// #include "USB/samd21_host.h"
// #include "sam.h"

//#if SAM3XA_SERIES

#ifdef HOST_DEFINED

//#define TRACE_UOTGHS_HOST(x)	x
#define TRACE_UOTGHS_HOST(x)

//extern void (*gpf_isr)(void);

// Handle UOTGHS Host driver state
static uhd_vbus_state_t uhd_state = UHD_STATE_NO_VBUS;

 __attribute__((__aligned__(4))) UsbHostDescriptor usb_pipe_table[USB_EPT_NUM];


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

/**
 * \brief Initialize the SAMD21 host driver.
 */
void UHD_Init(void)
{
	uint32_t pad_transn;
    uint32_t pad_transp;
    uint32_t pad_trim;

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
	USB->HOST.CTRLA.bit.SWRST = 1;
	while (USB->HOST.SYNCBUSY.bit.SWRST) {
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

	USB->HOST.PADCAL.bit.TRANSN = pad_transn;

	pad_transp =( *((uint32_t *)(NVMCTRL_OTP4)
	+ (NVM_USB_PAD_TRANSP_POS / 32))
	>> (NVM_USB_PAD_TRANSP_POS % 32))
	& ((1 << NVM_USB_PAD_TRANSP_SIZE) - 1);

	if (pad_transp == 0x1F) {  // maximum value (31)
		pad_transp = 29;
	}

	USB->HOST.PADCAL.bit.TRANSP = pad_transp;

	pad_trim =( *((uint32_t *)(NVMCTRL_OTP4)
	+ (NVM_USB_PAD_TRIM_POS / 32))
	>> (NVM_USB_PAD_TRIM_POS % 32))
	& ((1 << NVM_USB_PAD_TRIM_SIZE) - 1);

	if (pad_trim == 0x7) {  // maximum value (7)
		pad_trim = 3;
	}

	USB->HOST.PADCAL.bit.TRIM = pad_trim;

	/* Set the configuration */
	udd_force_host_mode();
	udd_device_run_in_standby();
    // Set address of USB SRAM
	USB->HOST.DESCADD.reg = (uint32_t)(&usb_endpoint_table[0]);
	// For USB_SPEED_FULL
	udd_force_full_speed();
 	for (uint32_t i = 0; i < sizeof(usb_endpoint_table); i++) {
 		(*(uint32_t *)(&usb_endpoint_table[0]+i)) = 0;
 	}

	uhd_state = UHD_STATE_NO_VBUS;

	USB->HOST.CTRLB.bit.VBUSOK = 1;

	// Configure interrupts
	NVIC_SetPriority((IRQn_Type) USB_IRQn, 0UL);
	NVIC_EnableIRQ((IRQn_Type) USB_IRQn);
}

// /**
//  * \brief Initialize the UOTGHS host driver.
//  */
// void UHD_Init(void)
// {
// //	irqflags_t flags;
//
// 	// To avoid USB interrupt before end of initialization
// //	flags = cpu_irq_save();
//
// 	// Setup USB Host interrupt callback
// //	UHD_SetStack(&UHD_ISR);
//
// 	// Enables the USB Clock
// 	pmc_enable_upll_clock();
// 	pmc_switch_udpck_to_upllck(0); // div=0+1
// 	pmc_enable_udpck();
// 	pmc_enable_periph_clk(ID_UOTGHS);
//
// 	// Always authorize asynchronous USB interrupts to exit of sleep mode
// 	// For SAM3 USB wake up device except BACKUP mode
// 	NVIC_SetPriority((IRQn_Type) ID_UOTGHS, 0);
// 	NVIC_EnableIRQ((IRQn_Type) ID_UOTGHS);
//
// 	// ID pin not used then force host mode
// 	otg_disable_id_pin();
// 	udd_force_host_mode();
//
// 	// Signal is active low (because all SAM3X Pins are high after startup)
// 	// Hence VBOF must be low after connection request to power up the remote device
// 	// uhd_set_vbof_active_low();
//
// 	// According to the Arduino Due circuit the VBOF must be active high to power up the remote device
// 	uhd_set_vbof_active_high();
//
// 	otg_enable_pad();
// 	otg_enable();
//
// 	otg_unfreeze_clock();
//
// 	// Check USB clock
// 	while (!Is_otg_clock_usable())
// 		;
//
// 	// Clear all interrupts that may have been set by a previous host mode
// 	UOTGHS->UOTGHS_HSTICR = UOTGHS_HSTICR_DCONNIC | UOTGHS_HSTICR_DDISCIC
// 			| UOTGHS_HSTICR_HSOFIC  | UOTGHS_HSTICR_HWUPIC
// 			| UOTGHS_HSTICR_RSMEDIC | UOTGHS_HSTICR_RSTIC
// 			| UOTGHS_HSTICR_RXRSMIC;
//
// 	otg_ack_vbus_transition();
//
// 	// Enable Vbus change and error interrupts
// 	// Disable automatic Vbus control after Vbus error
// 	Set_bits(UOTGHS->UOTGHS_CTRL,
// 		UOTGHS_CTRL_VBUSHWC | UOTGHS_CTRL_VBUSTE | UOTGHS_CTRL_VBERRE);
//
// 	uhd_enable_vbus();
//
// 	// Force Vbus interrupt when Vbus is always high
// 	// This is possible due to a short timing between a Host mode stop/start.
// 	if (Is_otg_vbus_high())
// 	{
// 		otg_raise_vbus_transition();
// 	}
//
// 	// Enable main control interrupt
// 	// Connection, SOF and reset
// 	UOTGHS->UOTGHS_HSTIER = UOTGHS_HSTICR_DCONNIC;
//
// 	otg_freeze_clock();
//
// 	uhd_state = UHD_STATE_NO_VBUS;
//
// //	cpu_irq_restore(flags);
// }


/**
 * \brief Interrupt sub routine for USB Host state machine management.
 */
//static void UHD_ISR(void)
void USB_Handler(void)
{
	uint16_t flags;
	uint8_t i;
	uint8_t ept_int;

	ept_int = udd_endpoint_interrupt();

	/* Not endpoint interrupt */
	if (0 == ept_int)
	{


	}
	else
	{
		/* host interrupts */

		/* get interrupt flags */
		flags = USB->HOST.INTFLAG.reg;

		/* host SOF interrupt */
		if (flags & USB_HOST_INTFLAG_HSOF) {
			/* clear the flag */
			USB->HOST.INTFLAG.reg = USB_HOST_INTFLAG_HSOF;
			uhd_state = UHD_STATE_CONNECTED;
			return;
		}

		/* host reset interrupt */
		if (flags & USB_HOST_INTFLAG_RST) {
			/* clear the flag */
			USB->HOST.INTFLAG.reg = USB_HOST_INTFLAG_RST;
			uhd_state = UHD_STATE_DISCONNECTED; //UHD_STATE_ERROR;
			return;
		}

		/* host upstream resume interrupts */
		if (flags & USB_HOST_INTFLAG_UPRSM) {
			/* clear the flags */
			USB->HOST.INTFLAG.reg = USB_HOST_INTFLAG_UPRSM;
			uhd_state = UHD_STATE_DISCONNECTED; //UHD_STATE_ERROR;
			return;
		}

		/* host downstream resume interrupts */
		if (flags & USB_HOST_INTFLAG_DNRSM) {
			/* clear the flags */
			USB->HOST.INTFLAG.reg = USB_HOST_INTFLAG_DNRSM;
			uhd_state = UHD_STATE_DISCONNECTED; //UHD_STATE_ERROR;
			return;
		}

		/* host wakeup interrupts */
		if (flags & USB_HOST_INTFLAG_WAKEUP) {
			/* clear the flags */
			USB->HOST.INTFLAG.reg = USB_HOST_INTFLAG_WAKEUP;
			uhd_state = UHD_STATE_CONNECTED; //UHD_STATE_ERROR;
			return;
		}

		/* host ram access interrupt  */
		if (flags & USB_HOST_INTFLAG_RAMACER) {
			/* clear the flag */
			USB->HOST.INTFLAG.reg = USB_HOST_INTFLAG_RAMACER;
			uhd_state = UHD_STATE_DISCONNECTED; //UHD_STATE_ERROR;
			return;
		}

		/* host connect interrupt */
		if (flags & USB_HOST_INTFLAG_DCONN) {
			TRACE_UOTGHS_HOST(printf(">>> UHD_ISR : Connection INT\r\n");)
			/* clear the flag */
			uhd_ack_connection();
			uhd_disable_connection_int();
			uhd_ack_disconnection();
			uhd_enable_disconnection_int();
			//uhd_enable_sof();
			uhd_state = UHD_STATE_CONNECTED;
			return;
		}

		/* host disconnect interrupt 	*/
		if (flags & USB_HOST_INTFLAG_DDISC) {
			TRACE_UOTGHS_HOST(printf(">>> UHD_ISR : Disconnection INT\r\n");)
			/* clear the flag */
			uhd_ack_disconnection();
			uhd_disable_disconnection_int();
			// Stop reset signal, in case of disconnection during reset
			uhd_stop_reset();
			// Disable wakeup/resumes interrupts,
			// in case of disconnection during suspend mode
			//UOTGHS->UOTGHS_HSTIDR = UOTGHS_HSTIDR_HWUPIEC
			//		| UOTGHS_HSTIDR_RSMEDIEC
			//		| UOTGHS_HSTIDR_RXRSMIEC;
			uhd_ack_connection();
			uhd_enable_connection_int();
			uhd_state = UHD_STATE_DISCONNECTED;
			return;
		}

	}

}




/**
 * \brief Trigger a USB bus reset.
 */
void UHD_BusReset(void)
{
	USB->HOST.CTRLB.bit.BUSRESET = 1;;
}

/**
 * \brief Get VBUS state.
 *
 * \return VBUS status.
 */
uhd_vbus_state_t UHD_GetVBUSState(void)
{
	return uhd_state;
}



/**
 * \brief Allocate FIFO for pipe 0.
 *
 * \param ul_add Address of remote device for pipe 0.
 * \param ul_ep_size Actual size of the FIFO in bytes.
 *
 * \retval 0 success.
 * \retval 1 error.
 */
uint32_t UHD_Pipe0_Alloc(uint32_t ul_add, uint32_t ul_ep_size)
{
	struct usb_host_pipe_config cfg;

	if (ep_size < 8)
	{
		return 0;
	}

	/* set pipe config */
	USB->HOST.HostPipe[0].PCFG.bit.BK = 0;
	USB->HOST.HostPipe[0].PCFG.bit.PTYPE = USB_HOST_PIPE_TYPE_CONTROL;
	USB->HOST.HostPipe[0].BINTERVAL.reg = 0;
	USB->HOST.HostPipe[0].PCFG.bit.PTOKEN = USB_HOST_PIPE_TOKEN_SETUP;

	memset((uint8_t *)&usb_pipe_table[pipe_num], 0, sizeof(usb_pipe_table[0]));
	usb_pipe_table[pipe_num].HostDescBank[0].CTRL_PIPE.bit.PDADDR = 0;
	usb_pipe_table[pipe_num].HostDescBank[0].CTRL_PIPE.bit.PEPNUM = 0;
	usb_pipe_table[pipe_num].HostDescBank[0].PCKSIZE.bit.SIZE = 0x03;  // 64 bytes

	USB->HOST.HostPipe[pipe_num].PINTENSET.reg = USB_HOST_PINTENSET_TRCPT_Msk;
	USB->HOST.HostPipe[pipe_num].PINTENSET.reg = USB_HOST_PINTENSET_TRFAIL | USB_HOST_PINTENSET_PERR;
	USB->HOST.HostPipe[pipe_num].PINTENSET.reg = USB_HOST_PINTENSET_TXSTP;
	USB->HOST.HostPipe[pipe_num].PINTENSET.reg = USB_HOST_PINTENSET_STALL;

	return 1;
}

/**
 * \brief Allocate a new pipe.
 *
 * \note UOTGHS maximum pipe number is limited to 10, meaning that only a limited
 * amount of devices can be connected. Unfortunately, using only one pipe shared accross
 * various endpoints and devices is not possible because the UOTGHS IP does not allow to
 * change the data toggle value through register interface.
 *
 * \param ul_dev_addr Address of remote device.
 * \param ul_dev_ep Targeted endpoint of remote device.
 * \param ul_type Pipe type.
 * \param ul_dir Pipe direction.
 * \param ul_maxsize Pipe size.
 * \param ul_interval Polling interval (if applicable to pipe type).
 * \param ul_nb_bank Number of banks associated with this pipe.
 *
 * \return the newly allocated pipe number on success, 0 otherwise.
 */

//		pipe = UHD_Pipe_Alloc(bAddress, epInfo[index].deviceEpNum, UOTGHS_HSTPIPCFG_PTYPE_BLK, UOTGHS_HSTPIPCFG_PTOKEN_IN, epInfo[index].maxPktSize, 0, UOTGHS_HSTPIPCFG_PBK_1_BANK);

uint32_t UHD_Pipe_Alloc(uint32_t ul_dev_addr, uint32_t ul_dev_ep, uint32_t ul_type, uint32_t ul_dir, uint32_t ul_maxsize, uint32_t ul_interval, uint32_t ul_nb_bank)
//bool uhd_ep_alloc(usb_add_t add, usb_ep_desc_t *ep_desc)
{
	/* set pipe config */
	USB->HOST.HostPipe[ul_dev_ep].PCFG.bit.BK = ul_nb_bank;
	USB->HOST.HostPipe[ul_dev_ep].PCFG.bit.PTYPE = ul_type;
	USB->HOST.HostPipe[ul_dev_ep].BINTERVAL.reg = ul_interval;

	if (ul_dir & USB_EP_DIR_IN)
	{
		USB->HOST.HostPipe[ul_dev_ep].PCFG.bit.PTOKEN = USB_HOST_PIPE_TOKEN_IN;
		USB->HOST.HostPipe[ul_dev_ep].PSTATUSSET.reg = USB_HOST_PSTATUSSET_BK0RDY;
	}
	else
	{
		USB->HOST.HostPipe[ul_dev_ep].PCFG.bit.PTOKEN = USB_HOST_PIPE_TOKEN_OUT;
		USB->HOST.HostPipe[ul_dev_ep].PSTATUSCLR.reg =  USB_HOST_PSTATUSCLR_BK0RDY;
	}

	memset((uint8_t *)&usb_descriptor_table.usb_pipe_table[ul_dev_ep], 0, sizeof(usb_pipe_table[ul_dev_ep]));

	usb_descriptor_table.usb_pipe_table[ul_dev_ep].HostDescBank[0].CTRL_PIPE.bit.PDADDR = ul_dev_addr;
	usb_descriptor_table.usb_pipe_table[ul_dev_ep].HostDescBank[0].CTRL_PIPE.bit.PEPNUM = ul_dev_ep;
	usb_descriptor_table.usb_pipe_table[ul_dev_ep].HostDescBank[0].PCKSIZE.bit.SIZE = 0x03;  // 64 bytes

	USB->HOST.HostPipe[pipe_num].PINTENSET.reg = USB_HOST_PINTENSET_TRCPT_Msk;
	USB->HOST.HostPipe[pipe_num].PINTENSET.reg = USB_HOST_PINTENSET_TRFAIL | USB_HOST_PINTENSET_PERR;
//	USB->HOST.HostPipe[pipe_num].PINTENSET.reg = USB_HOST_PINTENSET_TXSTP;
	USB->HOST.HostPipe[pipe_num].PINTENSET.reg = USB_HOST_PINTENSET_STALL;

	return 1;
}


/**
 * \brief Free a pipe.
 *
 * \param ul_pipe Pipe number to free.
 */
void UHD_Pipe_Free(uint32_t ul_pipe)
{
	// Unalloc pipe
	uhd_disable_pipe(ul_pipe);
	uhd_unallocate_memory(ul_pipe);
	uhd_reset_pipe(ul_pipe);

	// The Pipe is frozen and no additional requests will be sent to the device on this pipe address.
	USB->HOST.HostPipe[pipe_num].PSTATUSSET.reg = USB_HOST_PSTATUSSET_PFREEZE;
}

/**
 * \brief Read from a pipe.
 *
 * \param ul_pipe Pipe number.
 * \param ul_size Maximum number of data to read.
 * \param data Buffer to store the data.
 *
 * \return number of data read.
 */
uint32_t UHD_Pipe_Read(uint32_t pipe_num, uint32_t buf_size, uint8_t* buf)
{
	if (USB->HOST.HostPipe[pipe_num].PCFG.bit.PTYPE == USB_HOST_PIPE_TYPE_DISABLE)
	{
		return 0;
	}

	/* get pipe config from setting register */
	usb_descriptor_table.usb_pipe_table[pipe_num].HostDescBank[0].ADDR.reg = (uint32_t)buf;
	usb_descriptor_table.usb_pipe_table[pipe_num].HostDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
	usb_descriptor_table.usb_pipe_table[pipe_num].HostDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = buf_size;
	USB->HOST.HostPipe[pipe_num].PCFG.bit.PTOKEN = USB_HOST_PIPE_TOKEN_IN;

	/* Start transfer */
	USB->HOST.HostPipe[pipe_num].PSTATUSCLR.reg = USB_HOST_PSTATUSCLR_BK0RDY;

	// Unfreeze pipe
	USB->HOST.HostPipe[pipe_num].PSTATUSCLR.reg = USB_HOST_PSTATUSCLR_PFREEZE;

	return buf_size;
}

/**
 * \brief Write into a pipe.
 *
 * \param ul_pipe Pipe number.
 * \param ul_size Maximum number of data to read.
 * \param data Buffer containing data to write.
 */
void UHD_Pipe_Write(uint32_t ul_pipe, uint32_t ul_size, uint8_t* data)
{

	if (USB->HOST.HostPipe[pipe_num].PCFG.bit.PTYPE == USB_HOST_PIPE_TYPE_DISABLE)
	{
		return 0;
	}

	/* get pipe config from setting register */
	usb_descriptor_table.usb_pipe_table[pipe_num].HostDescBank[0].ADDR.reg = (uint32_t)buf;
	usb_descriptor_table.usb_pipe_table[pipe_num].HostDescBank[0].PCKSIZE.bit.BYTE_COUNT = buf_size;
	usb_descriptor_table.usb_pipe_table[pipe_num].HostDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
	USB->HOST.HostPipe[pipe_num].PCFG.bit.PTOKEN = USB_HOST_PIPE_TOKEN_OUT;


	return 1;


// 	volatile uint8_t *ptr_ep_data = 0;
// 	uint32_t i = 0;
//
// 	// Check pipe
// 	if (!Is_uhd_pipe_enabled(ul_pipe))
// 	{
// 		// Endpoint not valid
// 		TRACE_UOTGHS_HOST(printf("/!\\ UHD_EP_Send : pipe is not enabled!\r\n");)
// 		return;
// 	}
//
// 	ptr_ep_data = (volatile uint8_t *)&uhd_get_pipe_fifo_access(ul_pipe, 8);
// 	for (i = 0; i < ul_size; ++i)
// 		*ptr_ep_data++ = *data++;
}

/**
 * \brief Send a pipe content.
 *
 * \param ul_pipe Pipe number.
 * \param ul_token_type Token type.
 */
void UHD_Pipe_Send(uint32_t ul_pipe, uint32_t ul_token_type)
{
	/* Start transfer */
	USB->HOST.HostPipe[pipe_num].PSTATUSSET.reg = USB_HOST_PSTATUSSET_BK0RDY;

	// Unfreeze pipe
	USB->HOST.HostPipe[pipe_num].PSTATUSCLR.reg = USB_HOST_PSTATUSCLR_PFREEZE;

// 	// Check pipe
// 	if (!Is_uhd_pipe_enabled(ul_pipe))
// 	{
// 		// Endpoint not valid
// 		TRACE_UOTGHS_HOST(printf("/!\\ UHD_EP_Send : pipe %lu is not enabled!\r\n", ul_pipe);)
// 		return;
// 	}
//
// 	// Set token type for zero length packet
// 	// When actually using the FIFO, pipe token MUST be configured first
// 	uhd_configure_pipe_token(ul_pipe, ul_token_type);
//
// 	// Clear interrupt flags
// 	uhd_ack_setup_ready(ul_pipe);
// 	uhd_ack_in_received(ul_pipe);
// 	uhd_ack_out_ready(ul_pipe);
// 	uhd_ack_short_packet(ul_pipe);
// 	uhd_ack_nak_received(ul_pipe);
//
// 	// Send actual packet
// 	uhd_ack_fifocon(ul_pipe);
// 	uhd_unfreeze_pipe(ul_pipe);
}

/**
 * \brief Check for pipe transfer completion.
 *
 * \param ul_pipe Pipe number.
 * \param ul_token_type Token type.
 *
 * \retval 0 transfer is not complete.
 * \retval 1 transfer is complete.
 */
uint32_t UHD_Pipe_Is_Transfer_Complete(uint32_t ul_pipe, uint32_t ul_token_type)
{

	// Freeze pipe
	USB->HOST.HostPipe[pipe_num].PSTATUSSET.reg = USB_HOST_PSTATUSSET_PFREEZE;
	switch(uhd_ctrl_request_phase) {
		case UHD_CTRL_REQ_PHASE_DATA_IN:
		_uhd_ctrl_phase_data_in(p_callback_para->transfered_size);
		break;
		case UHD_CTRL_REQ_PHASE_ZLP_IN:
		_uhd_ctrl_request_end(UHD_TRANS_NOERROR);
		break;
		case UHD_CTRL_REQ_PHASE_DATA_OUT:
		_uhd_ctrl_phase_data_out();
		break;
		case UHD_CTRL_REQ_PHASE_ZLP_OUT:
		_uhd_ctrl_request_end(UHD_TRANS_NOERROR);
		break;
	}


// 	// Check for transfer completion depending on token type
// 	switch (ul_token_type)
// 	{
// 		case UOTGHS_HSTPIPCFG_PTOKEN_SETUP:
// 			if (Is_uhd_setup_ready(ul_pipe))
// 			{
// 				uhd_freeze_pipe(ul_pipe);
// 				uhd_ack_setup_ready(ul_pipe);
// 				return 1;
// 			}
//
// 		case UOTGHS_HSTPIPCFG_PTOKEN_IN:
// 			if (Is_uhd_in_received(ul_pipe))
// 			{
// 				// In case of low USB speed and with a high CPU frequency,
// 				// a ACK from host can be always running on USB line
// 				// then wait end of ACK on IN pipe.
// 				while(!Is_uhd_pipe_frozen(ul_pipe))
// 					;
//
// 				// IN packet received
// 				uhd_ack_in_received(ul_pipe);
//
// 				return 1;
// 			}
//
// 		case UOTGHS_HSTPIPCFG_PTOKEN_OUT:
// 			if (Is_uhd_out_ready(ul_pipe))
// 			{
// 				// OUT packet sent
// 				uhd_freeze_pipe(ul_pipe);
// 				uhd_ack_out_ready(ul_pipe);
//
// 				return 1;
// 			}
// 	}
//
	return 0;
}

#endif /* SAM3XA_SERIES HOST_DEFINED */

