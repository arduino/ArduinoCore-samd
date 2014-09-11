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

// Clear BK0RDY for know that the BANK0 ram buffer (udd_g_ep_table[ep].DeviceDescBank[0]) is empty and can receive data 
#define udd_OUT_transfer_allowed(ep)              USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY
// Set BK1RDY for know that the BANK1 ram buffer (udd_g_ep_table[ep].DeviceDescBank[1]) is full and can send data
#define udd_IN_transfer_allowed(ep)               USB->DEVICE.DeviceEndpoint[ep].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK1RDY

// Stop OUT transfer
#define udd_OUT_stop_transfer(ep)                 USB->DEVICE.DeviceEndpoint[ep].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY
// Stop IN transfer
#define udd_IN_stop_transfer(ep)                  USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY

#define udd_read_endpoint_flag(ep)                USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg
// Is transfer completed ?
#define udd_is_IN_transf_cplt(ep)                 (USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_TRCPT(2))
#define udd_is_OUT_transf_cplt(ep)                (USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg & USB_DEVICE_EPINTFLAG_TRCPT(1))
// Clear the transfer complete flag
#define udd_clear_IN_transf_cplt(ep)              USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT(2)
#define udd_clear_OUT_transf_cplt(ep)             USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT(1)
// Enable interrupt transfer complete
#define udd_ept_enable_it_IN_transf_cplt(ep)      USB->DEVICE.DeviceEndpoint[ep].EPINTENSET.reg = USB_DEVICE_EPINTENSET_TRCPT(2)
#define udd_ept_enable_it_OUT_transf_cplt(ep)     USB->DEVICE.DeviceEndpoint[ep].EPINTENSET.reg = USB_DEVICE_EPINTENSET_TRCPT(1)

// Enables SETUP received interrupt
#define udd_enable_setup_received_interrupt(ep)   USB->DEVICE.DeviceEndpoint[ep].EPINTENSET.reg = USB_DEVICE_EPINTFLAG_RXSTP
// Clear the stall flag
#define udd_clear_stall_request(ep)			      USB->DEVICE.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_STALL(2)
// Remove stall
#define udd_remove_stall_request(ep)		      USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_STALLRQ(2)

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

