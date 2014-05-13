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
#define EP0_SIZE		64
#define EPX_SIZE		512  // 512 for Full Speed, EPT size max is 1024

#define EP_SINGLE_64 (0x32UL) // EP0
#define EP_DOUBLE_64 (0x36UL) // Other endpoints



#define USB_REGWRITEPROTECT()        PAC0->WPCLR.reg = (1<<5)


/*
// Control Endpoint
#define EP_TYPE_CONTROL				(USB_DEVICE_EPCFG_EPTYPE0(1) | USB_DEVICE_EPCFG_EPTYPE1(1))

// CDC Endpoints
#define EP_TYPE_BULK_IN				(USB_DEVICE_EPCFG_EPTYPE1(3) | USB_DEVICE_EPSTATUSCLR_BK1RDY)

#define EP_TYPE_BULK_OUT          	(USB_DEVICE_EPCFG_EPTYPE0(3) | USB_DEVICE_EPSTATUSCLR_BK0RDY)

#define EP_TYPE_INTERRUPT_IN      	(USB_DEVICE_EPCFG_EPTYPE1(4) | USB_DEVICE_EPSTATUSCLR_BK1RDY)

// HID Endpoints
#define EP_TYPE_INTERRUPT_IN_HID	(USB_DEVICE_EPCFG_EPTYPE1(4) | USB_DEVICE_EPSTATUSCLR_BK1RDY)

// Various definitions
#define EP_TYPE_INTERRUPT_OUT		(USB_DEVICE_EPCFG_EPTYPE0(4) | USB_DEVICE_EPSTATUSCLR_BK0RDY)


*/
//! \ingroup usb_device_group
//! \defgroup udd_group USB Device Driver (UDD)
//! UOTGHS low-level driver for USB device mode
//!
//! @{

/*#ifndef UOTGHS_DEVEPTCFG_EPDIR_Pos*/
/*// Bit pos is not defined in SAM header file but we need it.*/
/*# define UOTGHS_DEVEPTCFG_EPDIR_Pos 8*/
/*#endif*/

//! @name UOTGHS Device IP properties
//! These macros give access to IP properties
//! @{
  //! Get maximal number of endpoints
#define udd_get_endpoint_max_nbr()             (7)
#define UDD_MAX_PEP_NB                         (udd_get_endpoint_max_nbr() + 1)
  //! Get maximal number of banks of endpoints
#define udd_get_endpoint_bank_max_nbr(ep)      ((ep == 0) ? 1 : 2)
  //! Get maximal size of endpoint (3X, 1024/64)
#define udd_get_endpoint_size_max(ep)          (64)  // for bulk full speed
  //! Get DMA support of endpoints
#define Is_udd_endpoint_dma_supported(ep)      (((ep) >= 1) ? true : false)
  //! Get High Band Width support of endpoints
#define Is_udd_endpoint_high_bw_supported(ep)  (((ep) >= 1) ? true : false)
//! @}

//! @name USB Device speeds management
//! @{
// Force device low speed mode
#define udd_force_low_speed()               USB_REGWRITEPROTECT(); USB->DEVICE.CTRLB.reg &= ~USB_DEVICE_CTRLB_SPDCONF_Msk; USB->DEVICE.CTRLB.reg |= USB_DEVICE_CTRLB_SPDCONF_1_Val
// Force full speed mode
#define udd_force_full_speed()              USB_REGWRITEPROTECT(); USB->DEVICE.CTRLB.reg &= ~USB_DEVICE_CTRLB_SPDCONF_Msk



/*
//! @name UOTGHS Device vbus management
//! @{
#define udd_enable_vbus_interrupt()       (USB->UOTGHS_CTRL |= UOTGHS_CTRL_VBUSTE)
#define udd_disable_vbus_interrupt()      (Clr_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_VBUSTE))
#define Is_udd_vbus_interrupt_enabled()   (Tst_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_VBUSTE))
#define Is_udd_vbus_high()                (Tst_bits(USB->UOTGHS_SR, UOTGHS_SR_VBUS))
#define Is_udd_vbus_low()                 (!Is_udd_vbus_high())
#define udd_ack_vbus_transition()         (USB->UOTGHS_SCR = UOTGHS_SCR_VBUSTIC)
#define udd_raise_vbus_transition()       (USB->UOTGHS_SFR = UOTGHS_SFR_VBUSTIS)
#define Is_udd_vbus_transition()          (Tst_bits(USB->UOTGHS_SR, UOTGHS_SR_VBUSTI))
//! @}
*/


// Attaches to USB bus
#define udd_attach_device()                  USB_REGWRITEPROTECT(); USB->DEVICE.CTRLB.reg &= ~USB_DEVICE_CTRLB_DETACH
#define udd_detach_device()                  USB_REGWRITEPROTECT(); USB->DEVICE.CTRLB.reg |= USB_DEVICE_CTRLB_DETACH

//! @name UOTGHS device bus events control
//! These macros manage the UOTGHS Device bus events.
//! @{

//! Initiates a remote wake-up event
//! @{
//#define udd_initiate_remote_wake_up()     (USB->UOTGHS_DEVCTRL |= UOTGHS_DEVCTRL_RMWKUP)
//#define Is_udd_pending_remote_wake_up()   (Tst_bits(USB->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_RMWKUP))
//! @}

//! Manage upstream resume event (=remote wakeup)
//! The USB driver sends a resume signal called "Upstream Resume"
//! @{
#define udd_enable_remote_wake_up_interrupt()     (USB->UOTGHS_DEVIER = UOTGHS_DEVIER_UPRSMES)
#define udd_disable_remote_wake_up_interrupt()    (USB->UOTGHS_DEVIDR = UOTGHS_DEVIDR_UPRSMEC)
#define Is_udd_remote_wake_up_interrupt_enabled() (Tst_bits(USB->UOTGHS_DEVIMR, UOTGHS_DEVIMR_UPRSME))
#define udd_ack_remote_wake_up_start()            (USB->UOTGHS_DEVICR = UOTGHS_DEVICR_UPRSMC)
#define udd_raise_remote_wake_up_start()          (USB->UOTGHS_DEVIFR = UOTGHS_DEVIFR_UPRSMS)
#define Is_udd_remote_wake_up_start()             (Tst_bits(USB->UOTGHS_DEVISR, UOTGHS_DEVISR_UPRSM))
//! @}

//! Manage downstream resume event (=remote wakeup from host)
//! The USB controller detects a valid "End of Resume" signal initiated by the host
//! @{
#define udd_enable_resume_interrupt()             (USB->UOTGHS_DEVIER = UOTGHS_DEVIER_EORSMES)
#define udd_disable_resume_interrupt()            (USB->UOTGHS_DEVIDR = UOTGHS_DEVIDR_EORSMEC)
#define Is_udd_resume_interrupt_enabled()         (Tst_bits(USB->UOTGHS_DEVIMR, UOTGHS_DEVIMR_EORSME))
#define udd_ack_resume()                          (USB->UOTGHS_DEVICR = UOTGHS_DEVICR_EORSMC)
#define udd_raise_resume()                        (USB->UOTGHS_DEVIFR = UOTGHS_DEVIFR_EORSMS)
#define Is_udd_resume()                           (Tst_bits(USB->UOTGHS_DEVISR, UOTGHS_DEVISR_EORSM))
//! @}

//! Manage wake-up event (=usb line activity)
//! The USB controller is reactivated by a filtered non-idle signal from the lines
//! @{
#define udd_enable_wake_up_interrupt()            (USB->UOTGHS_DEVIER = UOTGHS_DEVIER_WAKEUPES)
#define udd_disable_wake_up_interrupt()           (USB->UOTGHS_DEVIDR = UOTGHS_DEVIDR_WAKEUPEC)
#define Is_udd_wake_up_interrupt_enabled()        (Tst_bits(USB->UOTGHS_DEVIMR, UOTGHS_DEVIMR_WAKEUPE))
#define udd_ack_wake_up()                         (USB->UOTGHS_DEVICR = UOTGHS_DEVICR_WAKEUPC)
#define udd_raise_wake_up()                       (USB->UOTGHS_DEVIFR = UOTGHS_DEVIFR_WAKEUPS)
#define Is_udd_wake_up()                          (Tst_bits(USB->UOTGHS_DEVISR, UOTGHS_DEVISR_WAKEUP))
//! @}

//! Manage reset event
//! Set when a USB "End of Reset" has been detected
//! @{
#define udd_enable_reset_interrupt()              USB->DEVICE.INTENSET.reg = USB_DEVICE_INTENSET_EORST
#define udd_disable_reset_interrupt()             (USB->UOTGHS_DEVIDR = UOTGHS_DEVIDR_EORSTEC)
#define Is_udd_reset_interrupt_enabled()          (Tst_bits(USB->UOTGHS_DEVIMR, UOTGHS_DEVIMR_EORSTE))
#define udd_ack_reset()                           USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_EORST //(USB->UOTGHS_DEVICR = UOTGHS_DEVICR_EORSTC)
#define udd_raise_reset()                         (USB->UOTGHS_DEVIFR = UOTGHS_DEVIFR_EORSTS)
//#define Is_udd_reset()                          (Tst_bits(USB->UOTGHS_DEVISR, UOTGHS_DEVISR_EORST))
#define Is_udd_reset()                            USB->DEVICE.INTFLAG.reg & USB_DEVICE_INTFLAG_EORST
//! @}

//! Manage start of frame (SOF) event
//! @{
#define udd_enable_sof_interrupt()                USB->DEVICE.INTENSET.reg = USB_DEVICE_INTENSET_SOF
#define udd_disable_sof_interrupt()               (USB->UOTGHS_DEVIDR = UOTGHS_DEVIDR_SOFEC)
#define Is_udd_sof_interrupt_enabled()            (Tst_bits(USB->UOTGHS_DEVIMR, UOTGHS_DEVIMR_SOFE))
#define udd_ack_sof()                             USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_SOF //(USB->UOTGHS_DEVICR = UOTGHS_DEVICR_SOFC)
#define udd_raise_sof()                           (USB->UOTGHS_DEVIFR = UOTGHS_DEVIFR_SOFS)
#define Is_udd_sof()                              (USB->DEVICE.INTENSET.reg & USB_DEVICE_INTENSET_SOF) //(Tst_bits(USB->UOTGHS_DEVISR, UOTGHS_DEVISR_SOF))
#define udd_frame_number()                        ((USB->DEVICE.FNUM.reg & USB_DEVICE_FNUM_FNUM_Msk) >> USB_DEVICE_FNUM_FNUM_Pos)
#define Is_udd_frame_number_crc_error()           (Tst_bits(USB->UOTGHS_DEVFNUM, UOTGHS_DEVFNUM_FNCERR))
//! @}


//! Manage suspend event
//! @{
#define udd_enable_suspend_interrupt()            (USB->UOTGHS_DEVIER = UOTGHS_DEVIER_SUSPES)
#define udd_disable_suspend_interrupt()           (USB->UOTGHS_DEVIDR = UOTGHS_DEVIDR_SUSPEC)
#define Is_udd_suspend_interrupt_enabled()        (Tst_bits(USB->UOTGHS_DEVIMR, UOTGHS_DEVIMR_SUSPE))
#define udd_ack_suspend()                         (USB->UOTGHS_DEVICR = UOTGHS_DEVICR_SUSPC)
#define udd_raise_suspend()                       (USB->UOTGHS_DEVIFR = UOTGHS_DEVIFR_SUSPS)
#define Is_udd_suspend()                          (Tst_bits(USB->UOTGHS_DEVISR, UOTGHS_DEVISR_SUSP))
//! @}

//! @}

//! @name UOTGHS device address control
//! These macros manage the UOTGHS Device address.
//! @{
  //! enables USB device address
#define udd_enable_address()                      (USB->UOTGHS_DEVCTRL |= UOTGHS_DEVCTRL_ADDEN)
  //! disables USB device address
#define udd_disable_address()                     (Clr_bits(USB->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_ADDEN))
#define Is_udd_address_enabled()                  (Tst_bits(USB->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_ADDEN))
  //! configures the USB device address and enable it.
#define udd_configure_address(address)            USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | address
  //! gets the currently configured USB device address
#define udd_get_configured_address()              (Rd_bitfield(USB->UOTGHS_DEVCTRL, UOTGHS_DEVCTRL_UADD_Msk))
//! @}


//! @name UOTGHS Device endpoint drivers
//! These macros manage the common features of the endpoints.
//! @{

//! Generic macro for UOTGHS registers that can be arrayed
//! @{
//#define UOTGHS_ARRAY(reg,index)                   ((&(USB->reg))[(index)])
//! @}

//! @name UOTGHS Device endpoint configuration
//! @{
  //! enables the selected endpoint
//#define udd_enable_endpoint(ep)                   (USB->UOTGHS_DEVEPT |= UOTGHS_DEVEPT_EPEN0 << (ep))
  //! disables the selected endpoint
//#define udd_disable_endpoint(ep)                  (Clr_bits(USB->UOTGHS_DEVEPT, UOTGHS_DEVEPT_EPEN0 << (ep)))
  //! tests if the selected endpoint is enabled
//#define Is_udd_endpoint_enabled(ep)               (Tst_bits(USB->UOTGHS_DEVEPT, UOTGHS_DEVEPT_EPEN0 << (ep)))
  //! resets the selected endpoint
#define udd_reset_endpoint(ep)                                         \
	do {                                                               \
		Set_bits(USB->UOTGHS_DEVEPT, UOTGHS_DEVEPT_EPRST0 << (ep)); \
		Clr_bits(USB->UOTGHS_DEVEPT, UOTGHS_DEVEPT_EPRST0 << (ep)); \
	} while (0)
  //! Tests if the selected endpoint is being reset
#define Is_udd_resetting_endpoint(ep)             (Tst_bits(USB->UOTGHS_DEVEPT, UOTGHS_DEVEPT_EPRST0 << (ep)))

  //! Configures the selected endpoint type
#define udd_configure_endpoint_type(ep, type)     (Wr_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPTYPE_Msk, type))
  //! Gets the configured selected endpoint type
#define udd_get_endpoint_type(ep)                 (Rd_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPTYPE_Msk))
  //! Enables the bank autoswitch for the selected endpoint
#define udd_enable_endpoint_bank_autoswitch(ep)   (Set_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_AUTOSW))
  //! Disables the bank autoswitch for the selected endpoint
#define udd_disable_endpoint_bank_autoswitch(ep)    (Clr_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_AUTOSW))
#define Is_udd_endpoint_bank_autoswitch_enabled(ep) (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_AUTOSW))
  //! Configures the selected endpoint direction
#define udd_configure_endpoint_direction(ep, dir) (Wr_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPDIR, dir))
  //! Gets the configured selected endpoint direction
#define udd_get_endpoint_direction(ep)            (Rd_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPDIR))
#define Is_udd_endpoint_in(ep)                    (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPDIR))
  //! Bounds given integer size to allowed range and rounds it up to the nearest
  //! available greater size, then applies register format of UOTGHS controller
  //! for endpoint size bit-field.
#define udd_format_endpoint_size(size)            (32 - clz(((uint32_t)min(max(size, 8), 1024) << 1) - 1) - 1 - 3)
  //! Configures the selected endpoint size
#define udd_configure_endpoint_size(ep, size)     (Wr_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPSIZE_Msk, udd_format_endpoint_size(size)))
  //! Gets the configured selected endpoint size
#define udd_get_endpoint_size(ep)                 (8 << Rd_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPSIZE_Msk))
  //! Configures the selected endpoint number of banks
#define udd_configure_endpoint_bank(ep, bank)     (Wr_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPBK_Msk, bank))
  //! Gets the configured selected endpoint number of banks
#define udd_get_endpoint_bank(ep)                 (Rd_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPBK_Msk)+1)
  //! Allocates the configuration selected endpoint in DPRAM memory
#define udd_allocate_memory(ep)                   (Set_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_ALLOC))
  //! un-allocates the configuration selected endpoint in DPRAM memory
#define udd_unallocate_memory(ep)                 (Clr_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_ALLOC))
#define Is_udd_memory_allocated(ep)               (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_ALLOC))

  //! Configures selected endpoint in one step
#define udd_configure_endpoint(ep, type, dir, size, bank) (\
	Wr_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTCFG[0], ep), UOTGHS_DEVEPTCFG_EPTYPE_Msk |\
			UOTGHS_DEVEPTCFG_EPDIR  |\
			UOTGHS_DEVEPTCFG_EPSIZE_Msk |\
			UOTGHS_DEVEPTCFG_EPBK_Msk ,   \
			(((uint32_t)(type) << UOTGHS_DEVEPTCFG_EPTYPE_Pos) & UOTGHS_DEVEPTCFG_EPTYPE_Msk) |\
			(((uint32_t)(dir ) << UOTGHS_DEVEPTCFG_EPDIR_Pos ) & UOTGHS_DEVEPTCFG_EPDIR) |\
			( (uint32_t)udd_format_endpoint_size(size) << UOTGHS_DEVEPTCFG_EPSIZE_Pos) |\
			(((uint32_t)(bank) << UOTGHS_DEVEPTCFG_EPBK_Pos) & UOTGHS_DEVEPTCFG_EPBK_Msk))\
)
  //! Tests if current endpoint is configured
#define Is_udd_endpoint_configured(ep)            (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_CFGOK))
  //! Returns the control direction
#define udd_control_direction()                   (Rd_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], EP_CONTROL), UOTGHS_DEVEPTISR_CTRLDIR))

  //! Resets the data toggle sequence
#define udd_reset_data_toggle(ep)                 (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_RSTDTS)
  //! Tests if the data toggle sequence is being reset
#define Is_udd_data_toggle_reset(ep)              (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_RSTDT))
  //! Returns data toggle
#define udd_data_toggle(ep)                       (Rd_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_DTSEQ_Msk))
//! @}


//! @name UOTGHS Device control endpoint
//! These macros control the endpoints.
//! @{

//! @name UOTGHS Device control endpoint interrupts
//! These macros control the endpoints interrupts.
//! @{
  //! Enables the selected endpoint interrupt
#define udd_enable_endpoint_interrupt(ep)         (USB->UOTGHS_DEVIER = UOTGHS_DEVIER_PEP_0 << (ep))
  //! Disables the selected endpoint interrupt
#define udd_disable_endpoint_interrupt(ep)        (USB->UOTGHS_DEVIDR = UOTGHS_DEVIDR_PEP_0 << (ep))
  //! Tests if the selected endpoint interrupt is enabled
//#define Is_udd_endpoint_interrupt_enabled(ep)     (Tst_bits(USB->UOTGHS_DEVIMR, UOTGHS_DEVIMR_PEP_0 << (ep)))
  //! Tests if an interrupt is triggered by the selected endpoint
#define Is_udd_endpoint_interrupt(ep)             ((USB->DEVICE.EPINTSMRY.reg & (1<<ep)) != 0 )
  //! Returns the lowest endpoint number generating an endpoint interrupt or MAX_PEP_NB if none
#define udd_get_interrupt_endpoint_number()       (ctz(((USB->UOTGHS_DEVISR >> UOTGHS_DEVISR_PEP_Pos) & \
                                                   (USB->UOTGHS_DEVIMR >> UOTGHS_DEVIMR_PEP_Pos)) |     \
                                                   (1 << MAX_PEP_NB)))
#define UOTGHS_DEVISR_PEP_Pos   12
#define UOTGHS_DEVIMR_PEP_Pos   12
//! @}

//! @name UOTGHS Device control endpoint errors
//! These macros control the endpoint errors.
//! @{
  //! Enables the STALL handshake
#define udd_enable_stall_handshake(ep)            (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_STALLRQS)
  //! Disables the STALL handshake
#define udd_disable_stall_handshake(ep)           (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_STALLRQC)
  //! Tests if STALL handshake request is running
#define Is_udd_endpoint_stall_requested(ep)       (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_STALLRQ))
  //! Tests if STALL sent
#define Is_udd_stall(ep)                          (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_STALLEDI))
  //! ACKs STALL sent
#define udd_ack_stall(ep)                         (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_STALLEDIC)
  //! Raises STALL sent
#define udd_raise_stall(ep)                       (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_STALLEDIS)
  //! Enables STALL sent interrupt
#define udd_enable_stall_interrupt(ep)            (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_STALLEDES)
  //! Disables STALL sent interrupt
#define udd_disable_stall_interrupt(ep)           (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_STALLEDEC)
  //! Tests if STALL sent interrupt is enabled
#define Is_udd_stall_interrupt_enabled(ep)        (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_STALLEDE))

  //! Tests if NAK OUT received
#define Is_udd_nak_out(ep)                        (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_NAKOUTI))
  //! ACKs NAK OUT received
#define udd_ack_nak_out(ep)                       (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_NAKOUTIC)
  //! Raises NAK OUT received
#define udd_raise_nak_out(ep)                     (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_NAKOUTIS)
  //! Enables NAK OUT interrupt
#define udd_enable_nak_out_interrupt(ep)          (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_NAKOUTES)
  //! Disables NAK OUT interrupt
#define udd_disable_nak_out_interrupt(ep)         (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_NAKOUTEC)
  //! Tests if NAK OUT interrupt is enabled
#define Is_udd_nak_out_interrupt_enabled(ep)      (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_NAKOUTE))

  //! Tests if NAK IN received
#define Is_udd_nak_in(ep)                         (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_NAKINI))
  //! ACKs NAK IN received
#define udd_ack_nak_in(ep)                        (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_NAKINIC)
  //! Raises NAK IN received
#define udd_raise_nak_in(ep)                      (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_NAKINIS)
  //! Enables NAK IN interrupt
#define udd_enable_nak_in_interrupt(ep)           (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_NAKINES)
  //! Disables NAK IN interrupt
#define udd_disable_nak_in_interrupt(ep)          (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_NAKINEC)
  //! Tests if NAK IN interrupt is enabled
#define Is_udd_nak_in_interrupt_enabled(ep)       (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_NAKINE))

  //! ACKs endpoint isochronous overflow interrupt
#define udd_ack_overflow_interrupt(ep)            (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_OVERFIC)
  //! Raises endpoint isochronous overflow interrupt
#define udd_raise_overflow_interrupt(ep)          (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_OVERFIS)
  //! Tests if an overflow occurs
#define Is_udd_overflow(ep)                       (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_OVERFI))
  //! Enables overflow interrupt
#define udd_enable_overflow_interrupt(ep)         (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_OVERFES)
  //! Disables overflow interrupt
#define udd_disable_overflow_interrupt(ep)        (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_OVERFEC)
  //! Tests if overflow interrupt is enabled
#define Is_udd_overflow_interrupt_enabled(ep)     (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_OVERFE))

  //! ACKs endpoint isochronous underflow interrupt
#define udd_ack_underflow_interrupt(ep)           (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_UNDERFIC)
  //! Raises endpoint isochronous underflow interrupt
#define udd_raise_underflow_interrupt(ep)         (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_UNDERFIS)
  //! Tests if an underflow occurs
#define Is_udd_underflow(ep)                      (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_UNDERFI))
  //! Enables underflow interrupt
#define udd_enable_underflow_interrupt(ep)        (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_UNDERFES)
  //! Disables underflow interrupt
#define udd_disable_underflow_interrupt(ep)       (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_UNDERFEC)
  //! Tests if underflow interrupt is enabled
#define Is_udd_underflow_interrupt_enabled(ep)    (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_UNDERFE))

  //! Tests if CRC ERROR ISO OUT detected
#define Is_udd_crc_error(ep)                      (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_CRCERRI))
  //! ACKs CRC ERROR ISO OUT detected
#define udd_ack_crc_error(ep)                     (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_CRCERRIC)
  //! Raises CRC ERROR ISO OUT detected
#define udd_raise_crc_error(ep)                   (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_CRCERRIS)
  //! Enables CRC ERROR ISO OUT detected interrupt
#define udd_enable_crc_error_interrupt(ep)        (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_CRCERRES)
  //! Disables CRC ERROR ISO OUT detected interrupt
#define udd_disable_crc_error_interrupt(ep)       (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_CRCERREC)
  //! Tests if CRC ERROR ISO OUT detected interrupt is enabled
#define Is_udd_crc_error_interrupt_enabled(ep)    (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_CRCERRE))
//! @}

//! @name UOTGHS Device control endpoint transfer
//! These macros control the endpoint transfer.
//! @{

  //! Tests if endpoint read allowed
#define Is_udd_read_enabled(ep)                   (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_RWALL))
  //! Tests if endpoint write allowed
#define Is_udd_write_enabled(ep)                  (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_RWALL))

  //! Returns the byte count
#define udd_byte_count(ep)                        (Rd_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_BYCT_Msk))
  //! Clears FIFOCON bit
#define udd_ack_fifocon(ep)                       (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_FIFOCONC)
  //! Tests if FIFOCON bit set
#define Is_udd_fifocon(ep)                        (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_FIFOCON))

  //! Returns the number of busy banks
#define udd_nb_busy_bank(ep)                      (Rd_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_NBUSYBK_Msk))
  //! Returns the number of the current bank
#define udd_current_bank(ep)                      (Rd_bitfield(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_CURRBK_Msk))
  //! Kills last bank
#define udd_kill_last_in_bank(ep)                 (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_KILLBKS)
#define Is_udd_kill_last(ep)                      (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_KILLBK))
  //! Tests if last bank killed
#define Is_udd_last_in_bank_killed(ep)            (!Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_KILLBK))
  //! Forces all banks full (OUT) or free (IN) interrupt
#define udd_force_bank_interrupt(ep)              (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_NBUSYBKS)
  //! Unforces all banks full (OUT) or free (IN) interrupt
#define udd_unforce_bank_interrupt(ep)            (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_NBUSYBKS)
  //! Enables all banks full (OUT) or free (IN) interrupt
#define udd_enable_bank_interrupt(ep)             (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_NBUSYBKES)
  //! Disables all banks full (OUT) or free (IN) interrupt
#define udd_disable_bank_interrupt(ep)            (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_NBUSYBKEC)
  //! Tests if all banks full (OUT) or free (IN) interrupt enabled
#define Is_udd_bank_interrupt_enabled(ep)         (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_NBUSYBKE))

  //! Tests if SHORT PACKET received
#define Is_udd_short_packet(ep)                   (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_SHORTPACKET))
  //! ACKs SHORT PACKET received
#define udd_ack_short_packet(ep)                  (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_SHORTPACKETC)
  //! Raises SHORT PACKET received
#define udd_raise_short_packet(ep)                (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_SHORTPACKETS)
  //! Enables SHORT PACKET received interrupt
#define udd_enable_short_packet_interrupt(ep)     (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_SHORTPACKETES)
  //! Disables SHORT PACKET received interrupt
#define udd_disable_short_packet_interrupt(ep)    (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_SHORTPACKETEC)
  //! Tests if SHORT PACKET received interrupt is enabled
#define Is_udd_short_packet_interrupt_enabled(ep) (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_SHORTPACKETE))

  //! Tests if SETUP received
#define Is_udd_setup_received(ep)                    (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_RXSTPI))
  //! ACKs SETUP received
#define udd_ack_setup_received(ep)                   (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_RXSTPIC)
  //! Raises SETUP received
#define udd_raise_setup_received(ep)                 (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_RXSTPIS)
  //! Enables SETUP received interrupt
//#define udd_enable_setup_received_interrupt(ep)    (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_RXSTPES)
#define udd_enable_setup_received_interrupt(ep)      USB->DEVICE.DeviceEndpoint[ep].EPINTENCLR.reg = USB_DEVICE_EPINTFLAG_RXSTP
  //! Disables SETUP received interrupt
#define udd_disable_setup_received_interrupt(ep)     (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_RXSTPEC)
  //! Tests if SETUP received interrupt is enabled
#define Is_udd_setup_received_interrupt_enabled(ep)  (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_RXSTPE))

  //! Tests if OUT received
#define Is_udd_out_received(ep)                   (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_RXOUTI))
  //! ACKs OUT received
#define udd_ack_out_received(ep)                  USB->DEVICE.DeviceEndpoint[ep].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY; //(UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_RXOUTIC)
  //! Raises OUT received
#define udd_raise_out_received(ep)                (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_RXOUTIS)
  //! Enables OUT received interrupt
#define udd_enable_out_received_interrupt(ep)     USB->DEVICE.DeviceEndpoint[ep].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK0RDY;  //(UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_RXOUTES)
  //! Disables OUT received interrupt
#define udd_disable_out_received_interrupt(ep)    (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_RXOUTEC)
  //! Tests if OUT received interrupt is enabled
#define Is_udd_out_received_interrupt_enabled(ep) (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_RXOUTE))

  //! Tests if IN sending
#define Is_udd_in_send(ep)                        (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTISR[0], ep), UOTGHS_DEVEPTISR_TXINI))
  //! ACKs IN sending
#define udd_ack_in_send(ep)                       (UOTGHS_ARRAY(UOTGHS_DEVEPTICR[0], ep) = UOTGHS_DEVEPTICR_TXINIC)
  //! Raises IN sending
#define udd_raise_in_send(ep)                     (UOTGHS_ARRAY(UOTGHS_DEVEPTIFR[0], ep) = UOTGHS_DEVEPTIFR_TXINIS)
  //! Enables IN sending interrupt
#define udd_enable_in_send_interrupt(ep)          (UOTGHS_ARRAY(UOTGHS_DEVEPTIER[0], ep) = UOTGHS_DEVEPTIER_TXINES)
  //! Disables IN sending interrupt
#define udd_disable_in_send_interrupt(ep)         (UOTGHS_ARRAY(UOTGHS_DEVEPTIDR[0], ep) = UOTGHS_DEVEPTIDR_TXINEC)
  //! Tests if IN sending interrupt is enabled
#define Is_udd_in_send_interrupt_enabled(ep)      (Tst_bits(UOTGHS_ARRAY(UOTGHS_DEVEPTIMR[0], ep), UOTGHS_DEVEPTIMR_TXINE))


  //! 8-bit access to FIFO data register of selected endpoint.
  //! @param ep Endpoint of which to access FIFO data register
  //! @return Volatile 8-bit data pointer to FIFO data register
  //! @warning It is up to the user of this macro to make sure that all accesses
  //! are aligned with their natural boundaries
  //! @warning It is up to the user of this macro to make sure that used HSB
  //! addresses are identical to the DPRAM internal pointer modulo 32 bits.
#define udd_get_endpoint_fifo_access8(ep) \
		(((volatile uint8_t (*)[0x8000])UOTGHS_RAM_ADDR)[(ep)])

//! @}

/*********************************************************************************************************************/

//! @name UOTGHS IP properties
//! These macros give access to IP properties (not defined in 3X)
//! @{
   //! Get IP name part 1 or 2
#define otg_get_ip_name()
#define otg_data_memory_barrier()
   //! Get IP version
#define otg_get_ip_version()
   //! Get DPRAM size (FIFO maximal size) in bytes
#define otg_get_dpram_size()
   //! Get size of USBB PB address space
#define otg_get_ip_paddress_size()
//! @}

//! @name UOTGHS OTG ID pin management
//! The ID pin come from the USB OTG connector (A and B receptable) and
//! allows to select the USB mode host or device.
//! The USBB hardware can manage it automaticaly. This feature is optional.
//! When otg_ID_PIN equals true in conf_usb_host.h, the USB_ID must be defined in board.h.
//!
//! @{
   //! PIO, PIO ID and MASK for USB_ID according to configuration from OTG_ID
#define OTG_ID_PIN                          USB_ID_GPIO
#define OTG_ID_FUNCTION                     USB_ID_FLAGS
   //! Input USB_ID from its pin
#define otg_input_id_pin() do {                     \
    pio_configure_pin(OTG_ID_PIN, OTG_ID_FUNCTION); \
} while (0)

   //! Enable external OTG_ID pin (listened to by USB)
#define otg_enable_id_pin()                 (Set_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_UIDE))
   //! Disable external OTG_ID pin (ignored by USB)
#define otg_disable_id_pin()                (Clr_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_UIDE))
   //! Test if external OTG_ID pin enabled (listened to by USB)
#define Is_otg_id_pin_enabled()             (Tst_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_UIDE))
// Force device mode
#define udd_force_device_mode()             (USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_MODE)
// Run in Standby
#define udd_device_run_in_standby()         (USB->DEVICE.CTRLA.reg |= USB_CTRLA_RUNSTDBY) 
   //! Test if device mode is forced
//#define Is_otg_device_mode_forced()         (!Is_otg_id_pin_enabled() && Tst_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_UIMOD))
// Force host mode
#define otg_force_host_mode()               (USB->DEVICE.CTRLA.reg |= USB_CTRLA_MODE)
   //! Test if host mode is forced
#define Is_otg_host_mode_forced()           (!Is_otg_id_pin_enabled() && !Tst_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_UIMOD))

//! @name UOTGHS OTG ID pin interrupt management
//! These macros manage the ID pin interrupt
//! @{
#define otg_enable_id_interrupt()           (Set_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_IDTE))
#define otg_disable_id_interrupt()          (Clr_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_IDTE))
#define Is_otg_id_interrupt_enabled()       (Tst_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_IDTE))
#define Is_otg_id_device()                  (Tst_bits(USB->UOTGHS_SR, UOTGHS_SR_ID))
#define Is_otg_id_host()                    (!Is_otg_id_device())
#define otg_ack_id_transition()             (USB->UOTGHS_SCR = UOTGHS_SCR_IDTIC)
#define otg_raise_id_transition()           (USB->UOTGHS_SFR = UOTGHS_SFR_IDTIS)
#define Is_otg_id_transition()              (Tst_bits(USB->UOTGHS_SR, UOTGHS_SR_IDTI))
//! @}


//! @name USBB OTG Vbus management
//! @{
#define otg_enable_vbus_interrupt()         (Set_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_VBUSTE))
//#define otg_disable_vbus_interrupt()        (Clr_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_VBUSTE))
//#define Is_otg_vbus_interrupt_enabled()     (Tst_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_VBUSTE))
//#define Is_otg_vbus_high()                  ((USB->CTRLB & USB_CTRLB_VBUSOK) == USB_CTRLB_VBUSOK)
#define is_usb_vbus_high()                  port_pin_get_input_level(USB_VBUS_PIN)

//#define Is_otg_vbus_low()                   (!Is_otg_vbus_high())
#define otg_ack_vbus_transition()           (USB->UOTGHS_SCR = UOTGHS_SCR_VBUSTIC)
#define otg_raise_vbus_transition()         (USB->UOTGHS_SFR = UOTGHS_SFR_VBUSTIS)
#define Is_otg_vbus_transition()            (Tst_bits(USB->UOTGHS_SR, UOTGHS_SR_VBUSTI))
//! @}

//! @name UOTGHS OTG main management
//! These macros allows to enable/disable pad and UOTGHS hardware
//! @{
  //! Enable USB macro
#define udd_enable()                        (USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE)
//(Set_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_USBE))
  //! Disable USB macro
#define udd_disable()                       (USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE)
//#define Is_otg_enabled()                    (Tst_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_USBE))

  //! Enable OTG pad
//#define otg_enable_pad()                    (Set_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_OTGPADE))
  //! Disable OTG pad
//#define otg_disable_pad()                   (Clr_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_OTGPADE))
//#define Is_otg_pad_enabled()                (Tst_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_OTGPADE))

  //! Check Clock Usable
  //! For parts with HS feature, this one corresponding at UTMI clock
//#define Is_otg_clock_usable()               (Tst_bits(USB->UOTGHS_SR, UOTGHS_SR_CLKUSABLE))

  //! Stop (freeze) internal USB clock
//#define otg_freeze_clock()                  (Set_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_FRZCLK))
//#define otg_unfreeze_clock()                (Clr_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_FRZCLK))
//#define Is_otg_clock_frozen()               (Tst_bits(USB->UOTGHS_CTRL, UOTGHS_CTRL_FRZCLK))

//! @}

#ifdef __cplusplus
}
#endif

#endif /* SAMD21_DEVICE_H_INCLUDED */

