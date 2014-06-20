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

#ifndef SAMD21_DEVICE_H_INCLUDED
#define SAMD21_DEVICE_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#define EP0				0
#define EPX_SIZE		64// 64 for Full Speed, EPT size max is 1024

// Force device low speed mode
#define udd_force_low_speed()               USB->DEVICE.CTRLB.reg &= ~USB_DEVICE_CTRLB_SPDCONF_Msk; USB->DEVICE.CTRLB.reg |= USB_DEVICE_CTRLB_SPDCONF_1_Val
// Force full speed mode
#define udd_force_full_speed()              USB->DEVICE.CTRLB.reg &= ~USB_DEVICE_CTRLB_SPDCONF_Msk

// Attaches to USB bus
#define udd_attach_device()                  USB->DEVICE.CTRLB.reg &= ~USB_DEVICE_CTRLB_DETACH
#define udd_detach_device()                  USB->DEVICE.CTRLB.reg |= USB_DEVICE_CTRLB_DETACH

// Manage reset event
// Set when a USB "End of Reset" has been detected
#define udd_enable_reset_interrupt()              USB->DEVICE.INTENSET.reg = USB_DEVICE_INTENSET_EORST
#define udd_ack_reset()                           USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_EORST
#define Is_udd_reset()                            (USB->DEVICE.INTFLAG.reg & USB_DEVICE_INTFLAG_EORST)

// Manage start of frame (SOF) event
#define udd_enable_sof_interrupt()                USB->DEVICE.INTENSET.reg = USB_DEVICE_INTENSET_SOF
#define udd_ack_sof()                             USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_SOF
#define Is_udd_sof()                              (USB->DEVICE.INTENSET.reg & USB_DEVICE_INTENSET_SOF)
#define udd_frame_number()                        ((USB->DEVICE.FNUM.reg & USB_DEVICE_FNUM_FNUM_Msk) >> USB_DEVICE_FNUM_FNUM_Pos)

// configures the USB device address and enable it.
#define udd_configure_address(address)            USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | address

// Enables SETUP received interrupt
#define udd_enable_setup_received_interrupt(ep)   USB->DEVICE.DeviceEndpoint[ep].EPINTENSET.reg = USB_DEVICE_EPINTFLAG_RXSTP
// ACKs OUT received
#define udd_ack_out_received(ep)                  USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY
// ACKs OUT received
#define udd_ack_in_received(ep)                   USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY
// Is transfert completed ?
#define udd_is_transf_cplt(ep)                   (USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_TRCPT1)
// Clear the transfer complete flag
#define udd_clear_transf_cplt(ep)                 USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.bit.TRCPT1 = 1
// Set the bank as ready
#define udd_bk_rdy(ep)                            USB->DEVICE.DeviceEndpoint[ep].EPSTATUSSET.bit.BK1RDY = 1
#define udd_read_endpoint_flag(ep)                USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg
// Enable interrupt transfer complete
#define udd_ept_enable_it_transf_cplt_in(ep)      USB->DEVICE.DeviceEndpoint[ep].EPINTENSET.reg = USB_DEVICE_EPINTENSET_TRCPT1
#define udd_ept_enable_it_transf_cplt_out(ep)     USB->DEVICE.DeviceEndpoint[ep].EPINTENSET.reg = USB_DEVICE_EPINTENSET_TRCPT0
// Clear the stall flag
#define udd_clear_stall_request(ep)			      USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_STALL1
// Remove stall 
#define udd_remove_stall_request(ep)		      USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_STALLRQ1

// Endpoint Interrupt Summary
#define udd_endpoint_interrupt()            USB->DEVICE.EPINTSMRY.reg

#define udd_clear_wakeup_interrupt()        USB->DEVICE.INTENCLR.reg = USB_DEVICE_INTENCLR_WAKEUP
#define udd_clear_eorsm_interrupt()         USB->DEVICE.INTENCLR.reg = USB_DEVICE_INTENCLR_EORSM
#define udd_clear_suspend_interrupt()       USB->DEVICE.INTENCLR.reg = USB_DEVICE_INTENCLR_SUSPEND



// Force device mode
#define udd_force_device_mode()             USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_MODE
// Run in Standby
#define udd_device_run_in_standby()         USB->DEVICE.CTRLA.reg |= USB_CTRLA_RUNSTDBY 
// Force host mode
#define udd_force_host_mode()               USB->DEVICE.CTRLA.reg |= USB_CTRLA_MODE


// Enable USB macro
#define udd_enable()                        USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE
// Disable USB macro
#define udd_disable()                       USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE

#ifdef __cplusplus
}
#endif

#endif /* SAMD21_DEVICE_H_INCLUDED */

