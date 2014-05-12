/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2011-2012, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following condition is met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

#include <stdio.h>
#include <stdint.h>

#include "variant.h"
#include "USB_device.h"
#include "samd21_device.h"
#include "sam.h"

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

#ifdef SAMD_SERIES
extern uint8_t usb_device_endpoint_is_configured(uint8_t ep);

#define TRACE_DEVICE(x)	x
//#define TRACE_DEVICE(x)

// Endpoint transfer direction is IN
#define  USB_EP_DIR_IN        0x80
// Endpoint transfer direction is OUT
#define  USB_EP_DIR_OUT       0x00

extern void (*gpf_isr)(void);

static volatile uint32_t ul_send_fifo_ptr[USB_EPT_NUM];
static volatile uint32_t ul_recv_fifo_ptr[USB_EPT_NUM];

/**
 * USB SRAM data containing pipe descriptor table
 * The content of the USB SRAM can be :
 * - modified by USB hardware interface to update pipe status.
 *   Thereby, it is read by software.
 * - modified by USB software to control pipe.
 *   Thereby, it is read by hardware.
 * This data section is volatile.
 */

#if (defined __GNUC__) || defined(__CC_ARM)
#define COMPILER_WORD_ALIGNED         __attribute__((__aligned__(4)))
#elif (defined __ICCARM__)
#define COMPILER_WORD_ALIGNED         _Pragma("data_alignment = 4")
#endif

#ifdef __ICCARM__
_Pragma("pack(4)")
#else
COMPILER_WORD_ALIGNED
#endif
union {
	UsbDeviceDescriptor usb_endpoint_table[USB_EPT_NUM];
	UsbHostDescriptor usb_pipe_table[USB_PIPE_NUM];
} usb_descriptor_table; 
_Pragma("pack()")


//void (*gpf_isr)(void) = (0UL);
//
//void UOTGHS_Handler( void )
//{
//	if (gpf_isr)
//		gpf_isr();
//} 
//void UDD_SetStack(void (*pf_isr)(void))
//{
//	gpf_isr = pf_isr;
//}

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

uint32_t UDD_Init(void)
{
	uint32_t i;
	uint32_t pad_transn;
    uint32_t  pad_transp;
    uint32_t  pad_trim;
//	struct system_pinmux_config pin_config;
//	struct system_gclk_chan_config gclk_chan_config;
//
//	/* Turn on the digital interface clock */
//	system_apb_clock_set_mask(SYSTEM_CLOCK_APB_APBB, PM_APBBMASK_USB);
//
//	/* Set up the USB DP/DN pins */
//	system_pinmux_get_config_defaults(&pin_config);
//	pin_config.mux_position = MUX_PA24G_USB_DM;
//	system_pinmux_pin_set_config(PIN_PA24G_USB_DM, &pin_config);
//	pin_config.mux_position = MUX_PA25G_USB_DP;
//	system_pinmux_pin_set_config(PIN_PA25G_USB_DP, &pin_config);
//
//	/* Setup clock for module */
//	system_gclk_chan_get_config_defaults(&gclk_chan_config);
//	gclk_chan_config.source_generator = module_config->source_generator;
//	system_gclk_chan_set_config(USB_GCLK_ID, &gclk_chan_config);
//	system_gclk_chan_enable(USB_GCLK_ID);
//	pin_config.mux_position = MUX_PB14H_GCLK_IO0;
//	pin_config.direction    = SYSTEM_PINMUX_PIN_DIR_OUTPUT;
//	system_pinmux_pin_set_config(PIN_PB14H_GCLK_IO0, &pin_config);

	/* Reset */
	USB->DEVICE.CTRLA.bit.SWRST = 1;
	while (USB->DEVICE.SYNCBUSY.bit.SWRST) {
		/* Sync wait */
	}

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
	USB->DEVICE.DESCADD.reg = (uint32_t)(&usb_descriptor_table.usb_endpoint_table[0]);
	// For USB_SPEED_FULL 
	udd_force_full_speed();
	// For USB_LOW_FULL 
	//udd_force_low_speed();

    // Clear USB RAM
//	memset((uint8_t *)(&usb_descriptor_table.usb_endpoint_table[0]), 0,
//			sizeof(usb_descriptor_table.usb_endpoint_table));

	for (i = 0; i < sizeof(usb_descriptor_table.usb_endpoint_table); i++) {
		(*(uint32_t *)(&usb_descriptor_table.usb_endpoint_table[0]+i)) = 0;
	}

    
//	/*  device callback related */
//	for (i = 0; i < USB_DEVICE_CALLBACK_N; i++) {
//		module_inst->device_callback[i] = NULL;
//	}
//	for (i = 0; i < USB_EPT_NUM; i++) {
//		for(j = 0; j < USB_DEVICE_EP_CALLBACK_N; j++) {
//			module_inst->device_endpoint_callback[i][j] = NULL;
//		}
//	}
//	module_inst->device_registered_callback_mask = 0;
//	module_inst->device_enabled_callback_mask = 0;
//	for (j = 0; j < USB_EPT_NUM; j++) {
//		module_inst->device_endpoint_registered_callback_mask[j] = 0;
//		module_inst->device_endpoint_enabled_callback_mask[j] = 0;
//	}

	/* Enable interrupts for this USB module */
//	system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_USB);
	// Configure interrupts
	NVIC_SetPriority((IRQn_Type) USB_IRQn, 0UL);
	NVIC_EnableIRQ((IRQn_Type) USB_IRQn);

//    uint32_t i;
//
//	for (i = 0; i < USB_EPT_NUM; ++i)
//	{
//		ul_send_fifo_ptr[i] = 0;
//		ul_recv_fifo_ptr[i] = 0;
//	}
//
//	// Enables the USB Clock
//	pmc_enable_periph_clk(ID_USB);
//	pmc_enable_upll_clock();
//	pmc_switch_udpck_to_upllck(0); // div=0+1
//	pmc_enable_udpck();
//
//	// Configure interrupts
//	NVIC_SetPriority((IRQn_Type) USB_IRQn, 0UL);
//	NVIC_EnableIRQ((IRQn_Type) USB_IRQn);
//
//	// Always authorize asynchrone USB interrupts to exit from sleep mode
//	//   for SAM3 USB wake up device except BACKUP mode
//	//pmc_set_fast_startup_input(PMC_FSMR_USBAL);
//
//	// ID pin not used then force device mode
////	otg_disable_id_pin();
//	udd_force_device_mode();
//
//	// Enable USB hardware
//
//    // The Pad Calibration values must be loaded from the NVM Software Calibration Area into the USB Pad Calibration
//    // register by software, before enabling the USB, to achieve the specified accuracy. Refer to “NVM Software Calibration
//    // Area Mapping” on page 21 for further details.
//    JCB TODO;
//
//    //	otg_disable_pad();
////	otg_enable_pad();
//	udd_enable();
////	otg_unfreeze_clock();
//
//	// Check USB clock
//	//while (!Is_otg_clock_usable())
//	//	;
//
//	// Force full Speed
//	udd_force_full_speed();
//
//	//otg_ack_vbus_transition();
//	// Force Vbus interrupt in case of Vbus always with a high level
//	// This is possible with a short timing between a Host mode stop/start.
//	/*if (Is_otg_vbus_high()) {
//		otg_raise_vbus_transition();
//	}
//	otg_enable_vbus_interrupt();*/
////	otg_freeze_clock();
//
	return 0UL ;
}

void UDD_Attach(void)
{
//	irqflags_t flags = cpu_irq_save();

	TRACE_DEVICE(printf("=> UDD_Attach\r\n");)

    // Authorize attach if Vbus is present
	udd_attach_device();

	// Enable USB line events
	udd_enable_reset_interrupt();

    // usefull for debug
    udd_enable_sof_interrupt();

//	cpu_irq_restore(flags);
}

void UDD_Detach(void)
{
	TRACE_DEVICE(printf("=> UDD_Detach\r\n");)
    udd_detach_device();
}


/**
 * \brief Check if current endpoint is configured
 *
 * \param module_inst   Pointer to USB software instance struct
 * \param ep            Endpoint address (direction & number)
 *
 * \return \c true if endpoint is configured and ready to use
 */
uint8_t usb_device_endpoint_is_configured(uint8_t ep)
{
	uint8_t ep_num = ep & 0xF;
	uint8_t flag;

	if (ep & USB_EP_DIR_IN) {
		flag = USB->DEVICE.DeviceEndpoint[ep_num].EPCFG.bit.EPTYPE1;
	} else {
		flag = USB->DEVICE.DeviceEndpoint[ep_num].EPCFG.bit.EPTYPE0;
	}
	return( flag != 0 );  // JCB to be checked
}




//// CONTROL configuration:
//enum status_code usb_device_endpoint_set_config(struct usb_module *module_inst,
		//struct usb_device_endpoint_config *ep_config)
void UDD_InitEP( uint32_t ul_ep_nb, uint32_t ul_ep_cfg )
{
	/* Sanity check arguments */
//	Assert(module_inst);
//	Assert(ep_config);

	TRACE_DEVICE(printf("=> UDD_InitEP : init EP %lu\r\n", (unsigned long)ul_ep_nb);)

    if ((USB->DEVICE.DeviceEndpoint[ul_ep_nb].EPCFG.reg & USB_DEVICE_EPCFG_EPTYPE0_Msk) == 0 && \
        (USB->DEVICE.DeviceEndpoint[ul_ep_nb].EPCFG.reg & USB_DEVICE_EPCFG_EPTYPE1_Msk) == 0) {

        USB->DEVICE.DeviceEndpoint[ul_ep_nb].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0(1) | USB_DEVICE_EPCFG_EPTYPE1(1);
        USB->DEVICE.DeviceEndpoint[ul_ep_nb].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;
        USB->DEVICE.DeviceEndpoint[ul_ep_nb].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY;
    } else {
		TRACE_DEVICE(printf("=> UDD_InitEP : ERROR ALREADY INITIALIZED EP %lu\r\n", (unsigned long)ul_ep_nb);)
		while(1);
    }

    usb_descriptor_table.usb_endpoint_table[ul_ep_nb].DeviceDescBank[0].PCKSIZE.reg &= ~USB_DEVICE_PCKSIZE_AUTO_ZLP;
    usb_descriptor_table.usb_endpoint_table[ul_ep_nb].DeviceDescBank[1].PCKSIZE.reg &= ~USB_DEVICE_PCKSIZE_AUTO_ZLP;

    usb_descriptor_table.usb_endpoint_table[ul_ep_nb].DeviceDescBank[0].PCKSIZE.bit.SIZE = 0x3;  // for 64 bytes
    usb_descriptor_table.usb_endpoint_table[ul_ep_nb].DeviceDescBank[1].PCKSIZE.bit.SIZE = 0x3;  // for 64 bytes
}




void UDD_InitEndpoints(const uint8_t* eps_table, const uint8_t ul_eps_table_size)
{
	uint8_t ul_ep_nb;
	uint8_t ep_bank;

	for (ul_ep_nb = 1; ul_ep_nb < ul_eps_table_size; ul_ep_nb++)
	{
//		// Configure EP
//		UOTGHS->UOTGHS_DEVEPTCFG[ul_ep_nb] = eps_table[ul_ep_nb];
//		// Enable EP
//		udd_enable_endpoint(ul_ep_nb);
//
//		if (!Is_udd_endpoint_configured(ul_ep_nb)) {
//			TRACE_DEVICE(printf("=> UDD_InitEP : ERROR FAILED TO INIT EP %lu\r\n", ul_ep_nb);)
//			while(1);
//		}

        TRACE_DEVICE(printf("=> UDD_InitEndpoints : init EP %lu\r\n", (unsigned long)ul_ep_nb);)

        USB->DEVICE.DeviceEndpoint[ul_ep_nb].EPCFG.reg = eps_table[ul_ep_nb]& 0xEF;
        ep_bank = eps_table[ul_ep_nb] >> 7;
        USB->DEVICE.DeviceEndpoint[ul_ep_nb].EPSTATUSCLR.reg = ep_bank;
        // JCB only 1 bank ????

        usb_descriptor_table.usb_endpoint_table[ul_ep_nb].DeviceDescBank[ep_bank].PCKSIZE.bit.SIZE = 0x06; // for 512

//        if (true == ep_config->auto_zlp) {
            usb_descriptor_table.usb_endpoint_table[ul_ep_nb].DeviceDescBank[ep_bank].PCKSIZE.reg |= USB_DEVICE_PCKSIZE_AUTO_ZLP;
//        } else {
//            usb_descriptor_table.usb_endpoint_table[ul_ep_nb].DeviceDescBank[ep_bank].PCKSIZE.reg &= ~USB_DEVICE_PCKSIZE_AUTO_ZLP;
//        }
    }
}

//// Wait until ready to accept IN packet.
//void UDD_WaitIN(void)
//{
//	while (!(UOTGHS->UOTGHS_DEVEPTISR[EP0] & UOTGHS_DEVEPTISR_TXINI))
//		;
//}
//
//void UDD_WaitOUT(void)
//{
//	while (!(UOTGHS->UOTGHS_DEVEPTISR[EP0] & UOTGHS_DEVEPTISR_RXOUTI))
//		;
//}

// Send packet.
void UDD_ClearIN(void)
{
	TRACE_DEVICE(printf("=> UDD_ClearIN: sent %lu bytes\r\n", (unsigned long)ul_send_fifo_ptr[EP0]);)

	//UOTGHS->UOTGHS_DEVEPTICR[EP0] = UOTGHS_DEVEPTICR_TXINIC;
	USB->DEVICE.DeviceEndpoint[EP0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK1RDY;
	ul_send_fifo_ptr[EP0] = 0;
}

void UDD_ClearOUT(void)
{
	//UOTGHS->UOTGHS_DEVEPTICR[EP0] = UOTGHS_DEVEPTICR_RXOUTIC;
	USB->DEVICE.DeviceEndpoint[EP0].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY;

	ul_recv_fifo_ptr[EP0] = 0;
}

// Wait for IN FIFO to be ready to accept data or OUT FIFO to receive data.
// Return true if new IN FIFO buffer available.
uint32_t UDD_WaitForINOrOUT(void)
{
    while (!(USB->DEVICE.DeviceEndpoint[EP0].EPSTATUS.reg & (USB_DEVICE_EPSTATUSCLR_BK0RDY | USB_DEVICE_EPSTATUSCLR_BK1RDY)))
      ;
//	while (!(UOTGHS->UOTGHS_DEVEPTISR[EP0] & (UOTGHS_DEVEPTISR_TXINI | UOTGHS_DEVEPTISR_RXOUTI)))
//		;
//	return ((UOTGHS->UOTGHS_DEVEPTISR[EP0] & UOTGHS_DEVEPTISR_RXOUTI) == 0);

    return ((USB->DEVICE.DeviceEndpoint[EP0].EPSTATUS.reg & USB_DEVICE_EPSTATUSCLR_BK0RDY));
}


/**
 * \brief Start setup packet read job on a endpoint
 *
 * \param module_inst Pointer to USB device module instance
 * \param pbuf        Pointer to buffer
 *
 * \return Status of procedure
 * \retval STATUS_OK Job started successfully
 * \retval STATUS_ERR_DENIED Endpoint is not ready
 */
//enum status_code usb_device_endpoint_setup_buffer_job(struct usb_module *module_inst,
//		uint8_t* pbuf)
//{
//	/* Sanity check arguments */
//	Assert(module_inst);
//	Assert(module_inst->hw);
//
//
//	return STATUS_OK;
//}
uint8_t udd_ctrl_buffer[64];


uint32_t UDD_ReceivedSetupInt(void)
{
	/* get endpoint configuration from setting register */
	usb_descriptor_table.usb_endpoint_table[0].DeviceDescBank[0].ADDR.reg = (uint32_t)udd_ctrl_buffer;
	usb_descriptor_table.usb_endpoint_table[0].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 8;
	usb_descriptor_table.usb_endpoint_table[0].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
	USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY;

//	return UOTGHS->UOTGHS_DEVEPTISR[EP0] & UOTGHS_DEVEPTISR_RXSTPI;
    return (USB->DEVICE.DeviceEndpoint[0].EPSTATUS.reg &  USB_DEVICE_EPSTATUSCLR_BK0RDY);
}

void UDD_ClearSetupInt(void)
{
	//UOTGHS->UOTGHS_DEVEPTICR[EP0] = (UOTGHS_DEVEPTICR_RXSTPIC);
	USB->DEVICE.DeviceEndpoint[EP0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK1RDY;
}

uint32_t UDD_Send(uint32_t ep, const void* data, uint32_t len)
{
	const uint8_t *ptr_src = data;
//	uint8_t *ptr_dest = (uint8_t *) &udd_get_endpoint_fifo_access8(ep);
	uint32_t i;

	TRACE_DEVICE(printf("=> UDD_Send (1): ep=%lu ul_send_fifo_ptr=%lu len=%lu\r\n", (unsigned long)ep, (unsigned long)ul_send_fifo_ptr[ep], (unsigned long)len);)

//	while( UOTGHS_DEVEPTISR_TXINI != (UOTGHS->UOTGHS_DEVEPTISR[ep] & UOTGHS_DEVEPTISR_TXINI )) {}
    while (!(USB->DEVICE.DeviceEndpoint[ep].EPSTATUS.reg &  USB_DEVICE_EPSTATUSCLR_BK1RDY))
      ;

    usb_descriptor_table.usb_endpoint_table[0].DeviceDescBank[0].ADDR.reg = (uint32_t)&ul_send_fifo_ptr[ep];

	if (ep == EP0)
	{
		if (ul_send_fifo_ptr[ep] + len > EP0_SIZE)
			len = EP0_SIZE - ul_send_fifo_ptr[ep];
	}
	else
	{
		ul_send_fifo_ptr[ep] = 0;
	}
//	for (i = 0, ptr_dest += ul_send_fifo_ptr[ep]; i < len; ++i)
//		*ptr_dest++ = *ptr_src++;

	ul_send_fifo_ptr[ep] += len;//i;

	if (ep == EP0)
	{
		TRACE_DEVICE(printf("=> UDD_Send (2): ep=%lu ptr_dest=%lu maxlen=%d\r\n", (unsigned long)ep, (unsigned long)ul_send_fifo_ptr[ep], EP0_SIZE);)
		if (ul_send_fifo_ptr[ep] == EP0_SIZE)
		{
			UDD_ClearIN();	// Fifo is full, release this packet
        }
	}
	else
	{
		//UOTGHS->UOTGHS_DEVEPTICR[ep] = UOTGHS_DEVEPTICR_TXINIC;
        USB->DEVICE.DeviceEndpoint[EP0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK1RDY;
		//UOTGHS->UOTGHS_DEVEPTIDR[ep] = UOTGHS_DEVEPTIDR_FIFOCONC;
	}
	return len;
}

void UDD_Send8(uint32_t ep,  uint8_t data )
{
//	uint8_t *ptr_dest = (uint8_t *) &udd_get_endpoint_fifo_access8(ep);
//
//	TRACE_DEVICE(printf("=> UDD_Send8 : ul_send_fifo_ptr=%lu data=0x%x\r\n", ul_send_fifo_ptr[ep], data);)
//
//	ptr_dest[ul_send_fifo_ptr[ep]] = data;
//	ul_send_fifo_ptr[ep] += 1;

	uint8_t flag;
	flag = (uint8_t)(USB->DEVICE.DeviceEndpoint[ep].EPCFG.bit.EPTYPE1);
	if (flag == 0) {   // Bank1 is disabled
		return;
	};

	/* get endpoint configuration from setting register */
	usb_descriptor_table.usb_endpoint_table[ep].DeviceDescBank[1].ADDR.reg = (uint32_t)&data;
	usb_descriptor_table.usb_endpoint_table[ep].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
	usb_descriptor_table.usb_endpoint_table[ep].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 8;
	USB->DEVICE.DeviceEndpoint[ep].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK1RDY;
}

uint8_t UDD_Recv8(uint32_t ep)
{
//	uint8_t *ptr_dest = (uint8_t *) &udd_get_endpoint_fifo_access8(ep);
//	uint8_t data = ptr_dest[ul_recv_fifo_ptr[ep]];

	TRACE_DEVICE(printf("=> UDD_Recv8 : ep=%d\r\n", (char)ep);)

//	ul_recv_fifo_ptr[ep] += 1;
//	return data;

      
    volatile uint8_t data;

    usb_descriptor_table.usb_endpoint_table[ep].DeviceDescBank[0].ADDR.reg = (uint32_t)&data;
	usb_descriptor_table.usb_endpoint_table[ep].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 8;
	usb_descriptor_table.usb_endpoint_table[ep].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
	USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY;

	TRACE_DEVICE(printf("=> UDD_Recv8 : data=%lu\r\n", (unsigned long)data);)

    return data;
}

void UDD_Recv(uint32_t ep, uint8_t* data, uint32_t len)
{
//	uint8_t *ptr_src = (uint8_t *) &udd_get_endpoint_fifo_access8(ep);
//	uint8_t *ptr_dest = data;
//	uint32_t i;
//
//	for (i = 0, ptr_src += ul_recv_fifo_ptr[ep]; i < len; ++i)
//		*ptr_dest++ = *ptr_src++;
//
//	ul_recv_fifo_ptr[ep] += i;

	TRACE_DEVICE(printf("=> UDD_Recv : data=%lu, len=%d\r\n", data[0], (char)len);)

    usb_descriptor_table.usb_endpoint_table[ep].DeviceDescBank[0].ADDR.reg = (uint32_t)data;
	usb_descriptor_table.usb_endpoint_table[ep].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = len;
	usb_descriptor_table.usb_endpoint_table[ep].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;
	USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY;
}

void UDD_Stall(uint32_t ep)
{
//	UOTGHS->UOTGHS_DEVEPT = (UOTGHS_DEVEPT_EPEN0 << EP0);
//	UOTGHS->UOTGHS_DEVEPTIER[EP0] = UOTGHS_DEVEPTIER_STALLRQS;
	uint8_t ep_num = ep & 0xF;

	// Stall endpoint
	if (ep & USB_EP_DIR_IN) {
		USB->DEVICE.DeviceEndpoint[ep_num].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ1;
	} else {
		USB->DEVICE.DeviceEndpoint[ep_num].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ0;
	}
}
//		if (ep & USB_EP_DIR_IN) {
//			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENSET.reg = USB_DEVICE_EPINTENSET_STALL1;
//		} else {
//			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENSET.reg = USB_DEVICE_EPINTENSET_STALL0;
//		}
//		if (ep & USB_EP_DIR_IN) {
//			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENCLR.reg = USB_DEVICE_EPINTENCLR_STALL1;
//		} else {
//			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTENCLR.reg = USB_DEVICE_EPINTENCLR_STALL0;
//		}
//	if (ep & USB_EP_DIR_IN) {
//		return (module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUS.reg & USB_DEVICE_EPSTATUSSET_STALLRQ1);
//	} else {
//		return (module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUS.reg & USB_DEVICE_EPSTATUSSET_STALLRQ0);
//	}
//	// Stall endpoint
//	if (ep & USB_EP_DIR_IN) {
//		module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ1;
//	} else {
//		module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ0;
//	}
//
//	if (ep & USB_EP_DIR_IN) {
//		if (module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUS.reg & USB_DEVICE_EPSTATUSSET_STALLRQ1) {
//			// Remove stall request
//			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_STALLRQ1;
//			if (module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_STALL1) {
//				module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_STALL1;
//				// The Stall has occurred, then reset data toggle
//				module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSSET_DTGLIN;
//			}
//		}
//	} else {
//		if (module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUS.reg & USB_DEVICE_EPSTATUSSET_STALLRQ0) {
//			// Remove stall request
//			module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_STALLRQ0;
//			if (module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_STALL0) {
//				module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_STALL0;
//				// The Stall has occurred, then reset data toggle
//				module_inst->hw->DEVICE.DeviceEndpoint[ep_num].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSSET_DTGLOUT;
//			}
//		}
//	}
//				// endpoint transfer stall interrupt
//				if (flags & USB_DEVICE_EPINTFLAG_STALL_Msk) {
//					if (_usb_instances->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_STALL1) {
//						_usb_instances->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_STALL1;
//						ep_callback_para.endpoint_address = USB_EP_DIR_IN | i;
//					} else if (_usb_instances->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_STALL0) {
//						_usb_instances->hw->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_STALL0;
//						ep_callback_para.endpoint_address = USB_EP_DIR_OUT | i;
//					}
//
//					if (flags_run & USB_DEVICE_EPINTFLAG_STALL_Msk) {
//						(_usb_instances->device_endpoint_callback[i][USB_DEVICE_ENDPOINT_CALLBACK_STALL])(_usb_instances,&ep_callback_para);
//					}
//					return;
//				}

uint32_t UDD_FifoByteCount(uint32_t ep)
{
	uint8_t ep_num = ep & 0xF;

	//return ((UOTGHS->UOTGHS_DEVEPTISR[ep] & UOTGHS_DEVEPTISR_BYCT_Msk) >> UOTGHS_DEVEPTISR_BYCT_Pos);
	//return (USB->DEVICE.PCKSIZE.reg & USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk);
	return ((uint16_t)(usb_descriptor_table.usb_endpoint_table[ep_num].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT));
}

void UDD_ReleaseRX(uint32_t ep)
{
	TRACE_DEVICE(puts("=> UDD_ReleaseRX\r\n");)
//	UOTGHS->UOTGHS_DEVEPTICR[ep] = (UOTGHS_DEVEPTICR_NAKOUTIC | UOTGHS_DEVEPTICR_RXOUTIC);
	//UOTGHS->UOTGHS_DEVEPTICR[ep] = UOTGHS_DEVEPTICR_RXOUTIC;
	USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY;
    USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0;
    //UOTGHS->UOTGHS_DEVEPTIDR[ep] = UOTGHS_DEVEPTIDR_FIFOCONC;
	ul_recv_fifo_ptr[ep] = 0;
}

void UDD_ReleaseTX(uint32_t ep)
{
	TRACE_DEVICE(printf("=> UDD_ReleaseTX ep=%lu\r\n", (unsigned long)ep);)
//	UOTGHS->UOTGHS_DEVEPTICR[ep] = (UOTGHS_DEVEPTICR_NAKINIC | UOTGHS_DEVEPTICR_RXOUTIC | UOTGHS_DEVEPTICR_TXINIC);
	//UOTGHS->UOTGHS_DEVEPTICR[ep] = UOTGHS_DEVEPTICR_TXINIC;
    USB->DEVICE.DeviceEndpoint[ep].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK1RDY;
    USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
	//UOTGHS->UOTGHS_DEVEPTIDR[ep] = UOTGHS_DEVEPTIDR_FIFOCONC;
	ul_send_fifo_ptr[ep] = 0;
}

// Return true if the current bank is not full.
//uint32_t UDD_ReadWriteAllowed(uint32_t ep)
//{
//	//return (UOTGHS->UOTGHS_DEVEPTISR[ep] & UOTGHS_DEVEPTISR_RWALL);
//    return 1;
//}

void UDD_SetAddress(uint32_t addr)
{
	TRACE_DEVICE(printf("=> UDD_SetAddress : setting address to %lu\r\n", (unsigned long)addr);)

	udd_configure_address(addr);
	//udd_enable_address();
}

uint32_t UDD_GetFrameNumber(void)
{
	return udd_frame_number();
}

#ifdef __cplusplus
}
#endif

#endif /* SAMD_SERIES */

