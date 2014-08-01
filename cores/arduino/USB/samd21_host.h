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

#ifndef SAMD21_HOST_H_INCLUDED
#define SAMD21_HOST_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

extern __attribute__((__aligned__(4))) volatile UsbHostDescriptor usb_pipe_table[USB_EPT_NUM];

#define  USB_EP_DIR_IN        0x80
#define  USB_EP_DIR_OUT       0x00

#define USB_HOST_PCFG_PTYPE_DIS     USB_HOST_PCFG_PTYPE(0x0) // Pipe is disabled
#define USB_HOST_PCFG_PTYPE_CTRL    USB_HOST_PCFG_PTYPE(0x1) // Pipe is enabled and configured as CONTROL
#define USB_HOST_PCFG_PTYPE_ISO     USB_HOST_PCFG_PTYPE(0x2) // Pipe is enabled and configured as ISO
#define USB_HOST_PCFG_PTYPE_BULK    USB_HOST_PCFG_PTYPE(0x3) // Pipe is enabled and configured as BULK
#define USB_HOST_PCFG_PTYPE_INT     USB_HOST_PCFG_PTYPE(0x4) // Pipe is enabled and configured as INTERRUPT
#define USB_HOST_PCFG_PTYPE_EXT     USB_HOST_PCFG_PTYPE(0x5) // Pipe is enabled and configured as EXTENDED

#define USB_HOST_PCFG_PTOKEN_SETUP  USB_HOST_PCFG_PTOKEN(0x0)
#define USB_HOST_PCFG_PTOKEN_IN     USB_HOST_PCFG_PTOKEN(0x1)
#define USB_HOST_PCFG_PTOKEN_OUT    USB_HOST_PCFG_PTOKEN(0x2)

#define USB_PCKSIZE_SIZE_8_BYTES        0
#define USB_PCKSIZE_SIZE_16_BYTES       1
#define USB_PCKSIZE_SIZE_32_BYTES       2
#define USB_PCKSIZE_SIZE_64_BYTES       3 
#define USB_PCKSIZE_SIZE_128_BYTES      4
#define USB_PCKSIZE_SIZE_256_BYTES      5
#define USB_PCKSIZE_SIZE_512_BYTES      6
#define USB_PCKSIZE_SIZE_1023_BYTES_FS  7   
#define USB_PCKSIZE_SIZE_1024_BYTES_HS  7  

// USB device connection/disconnection monitoring
#define uhd_enable_connection_int()           USB->HOST.INTENSET.reg = USB_HOST_INTENSET_DCONN
#define uhd_disable_connection_int()          USB->HOST.INTENCLR.reg = USB_HOST_INTENCLR_DCONN
#define uhd_ack_connection()                  USB->HOST.INTFLAG.reg = USB_HOST_INTFLAG_DCONN

#define uhd_enable_disconnection_int()        USB->HOST.INTENSET.reg = USB_HOST_INTENSET_DDISC
#define uhd_disable_disconnection_int()       USB->HOST.INTENCLR.reg = USB_HOST_INTENCLR_DDISC
#define uhd_ack_disconnection()               USB->HOST.INTFLAG.reg = USB_HOST_INTFLAG_DDISC

// Initiates a reset event
#define uhd_start_reset()                            USB->HOST.CTRLA.bit.SWRST = 1;
#define Is_uhd_starting_reset()                      (USB->HOST.CTRLA & USB_CTRLA_SWRST)
#define uhd_stop_reset()                             (USB->DEVICE.STATUS.reg & USB_DEVICE_STATUS_LINESTATE_0_Val)

#define uhd_ack_reset_sent()                         USB->HOST.INTFLAG.reg = USB_HOST_INTFLAG_RST
#define Is_uhd_reset_sent()                          (USB->HOST.INTFLAG.reg & USB_HOST_INTFLAG_RST)

// Initiates a SOF events
#define uhd_enable_sof()                             USB->HOST.CTRLB.bit.SOFE = 1
#define uhd_disable_sof()                            USB->HOST.CTRLB.bit.SOFE = 0
#define Is_uhd_sof_enabled()                         (USB->HOST.CTRLB & USB_HOST_CTRLB_SOFE)
#define Is_uhd_sof()                                 (USB->HOST.INTFLAG.reg & USB_HOST_INTFLAG_HSOF)

// USB address of pipes
#define uhd_configure_address(pipe_num, addr) usb_pipe_table[pipe_num].HostDescBank[0].CTRL_PIPE.bit.PDADDR = addr
#define uhd_get_configured_address(pipe_num)  usb_pipe_table[pipe_num].HostDescBank[0].CTRL_PIPE.bit.PDADDR

// Pipes
#define uhd_freeze_pipe(p)                       USB->HOST.HostPipe[p].PSTATUSSET.reg = USB_HOST_PSTATUSSET_PFREEZE
#define uhd_unfreeze_pipe(p)                     USB->HOST.HostPipe[p].PSTATUSCLR.reg = USB_HOST_PSTATUSCLR_PFREEZE
#define Is_uhd_pipe_frozen(p)                    ((USB->HOST.HostPipe[p].PSTATUS.reg&USB_HOST_PSTATUS_PFREEZE)==USB_HOST_PSTATUS_PFREEZE)

// Pipe configuration
#define uhd_configure_pipe_token(p, token)       USB->HOST.HostPipe[p].PCFG.bit.PTOKEN = token

// Pipe data management
#define uhd_byte_count(p)                        usb_pipe_table[p].HostDescBank[0].PCKSIZE.bit.BYTE_COUNT
#define uhd_ack_setup_ready(p)                   USB->HOST.HostPipe[p].PINTFLAG.reg = USB_HOST_PINTFLAG_TXSTP
#define Is_uhd_setup_ready(p)                    ((USB->HOST.HostPipe[p].PINTFLAG.reg&USB_HOST_PINTFLAG_TXSTP) == USB_HOST_PINTFLAG_TXSTP)
#define uhd_ack_in_received(p)                   USB->HOST.HostPipe[p].PINTFLAG.reg = USB_HOST_PINTFLAG_TRCPT0
#define Is_uhd_in_received(p)                    ( (USB->HOST.HostPipe[p].PINTFLAG.reg&USB_HOST_PINTFLAG_TRCPT0) == USB_HOST_PINTFLAG_TRCPT0   )
#define uhd_ack_out_ready(p)                     USB->HOST.HostPipe[p].PINTFLAG.reg = USB_HOST_PINTFLAG_TRCPT0
#define Is_uhd_out_ready(p)                      ((USB->HOST.HostPipe[p].PINTFLAG.reg&USB_HOST_PINTFLAG_TRCPT0) == USB_HOST_PINTFLAG_TRCPT0)
#define uhd_ack_nak_received(p)                  usb_pipe_table[p].HostDescBank[1].STATUS_BK.reg &= ~USB_DEVICE_STATUS_BK_ERRORFLOW
#define Is_uhd_nak_received(p)                   (usb_pipe_table[p].HostDescBank[1].STATUS_BK.reg & USB_DEVICE_STATUS_BK_ERRORFLOW)

// Endpoint Interrupt Summary
#define uhd_endpoint_interrupt()            USB->HOST.PINTSMRY.reg

// Run in Standby
#define uhd_run_in_standby()                USB->HOST.CTRLA.reg |= USB_CTRLA_RUNSTDBY
// Force host mode
#define uhd_force_host_mode()               USB->HOST.CTRLA.reg |= USB_CTRLA_MODE

// Enable USB macro
#define uhd_enable()                        USB->HOST.CTRLA.reg |= USB_CTRLA_ENABLE
// Disable USB macro
#define uhd_disable()                       USB->HOST.CTRLA.reg &= ~USB_CTRLA_ENABLE

// Force full speed mode
#define uhd_force_full_speed()              USB->HOST.CTRLB.reg &= ~USB_HOST_CTRLB_SPDCONF_Msk

#ifdef __cplusplus
}
#endif

#endif /* SAMD21_HOST_H_INCLUDED */
