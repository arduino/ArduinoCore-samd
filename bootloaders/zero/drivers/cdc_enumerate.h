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
 * Redistributions of source code must retain the above copyright notice,
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

#ifndef CDC_ENUMERATE_H
#define CDC_ENUMERATE_H

#include "sam.h"
#include "stdbool.h"

#define USB_EP_OUT              2
#define USB_EP_OUT_SIZE         0x40
#define USB_EP_IN               1
#define USB_EP_IN_SIZE          0x40
#define USB_EP_COMM             3
#define MAX_EP                    4

#define NVM_USB_PAD_TRANSN_POS    45
#define NVM_USB_PAD_TRANSN_SIZE   5
#define NVM_USB_PAD_TRANSP_POS    50
#define NVM_USB_PAD_TRANSP_SIZE   5
#define NVM_USB_PAD_TRIM_POS      55
#define NVM_USB_PAD_TRIM_SIZE     3

typedef struct _USB_CDC
{
	// Private members
	Usb *pUsb;
	uint8_t currentConfiguration;
	uint8_t currentConnection;
	// Public Methods:
	uint8_t (*IsConfigured)(struct _USB_CDC *pCdc);
	uint32_t  (*Write) (struct _USB_CDC *pCdc, const char *pData, uint32_t length, uint8_t ep_num);
	uint32_t  (*Read)  (struct _USB_CDC *pCdc, char *pData, uint32_t length);
} USB_CDC, *P_USB_CDC;

typedef struct {
	uint32_t dwDTERate;
	uint8_t bCharFormat;
	uint8_t bParityType;
	uint8_t bDataBits;
} usb_cdc_line_coding_t;

/**
 * \brief Initializes the USB module
 *
 * \return Pointer to the USB CDC structure
 */
P_USB_CDC usb_init(void);

/**
 * \brief Sends a single byte through USB CDC
 *
 * \param Data to send
 * \return number of data sent
 */
int cdc_putc(int value);

/**
 * \brief Reads a single byte through USB CDC
 *
 * \return Data read through USB
 */
int cdc_getc(void);

/**
 * \brief Checks if a character has been received on USB CDC
 *
 * \return \c 1 if a byte is ready to be read.
 */
bool cdc_is_rx_ready(void);

/**
 * \brief Sends buffer on USB CDC
 *
 * \param data pointer
 * \param number of data to send
 * \return number of data sent
 */
uint32_t cdc_write_buf(void const* data, uint32_t length);

/**
 * \brief Gets data on USB CDC
 *
 * \param data pointer
 * \param number of data to read
 * \return number of data read
 */
uint32_t cdc_read_buf(void* data, uint32_t length);

/**
 * \brief Gets specified number of bytes on USB CDC
 *
 * \param data pointer
 * \param number of data to read
 * \return number of data read
 */
uint32_t cdc_read_buf_xmd(void* data, uint32_t length);


#endif // CDC_ENUMERATE_H
