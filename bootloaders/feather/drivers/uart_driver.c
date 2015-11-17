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

#include "uart_driver.h"

bool uart_drv_error_flag = false;

uint32_t uart_get_sercom_index(Sercom *sercom_instance)
{
	/* Save all available SERCOM instances for compare. */
	Sercom *sercom_instances[SERCOM_INST_NUM] = SERCOM_INSTS;

	/* Find index for sercom instance. */
	for (uint32_t i = 0; i < SERCOM_INST_NUM; i++) {
		if ((uintptr_t)sercom_instance == (uintptr_t)sercom_instances[i]) {
			return i;
		}
	}

	return 0;
}

void uart_basic_init(Sercom *sercom, uint16_t baud_val, enum uart_pad_settings pad_conf)
{
	/* Wait for synchronization */
	while(sercom->USART.SYNCBUSY.bit.ENABLE);
	/* Disable the SERCOM UART module */
	sercom->USART.CTRLA.bit.ENABLE = 0;
	/* Wait for synchronization */
	while(sercom->USART.SYNCBUSY.bit.SWRST);
	/* Perform a software reset */
	sercom->USART.CTRLA.bit.SWRST = 1;
	/* Wait for synchronization */
	while(sercom->USART.CTRLA.bit.SWRST);
	/* Wait for synchronization */
	while(sercom->USART.SYNCBUSY.bit.SWRST || sercom->USART.SYNCBUSY.bit.ENABLE);
	/* Update the UART pad settings, mode and data order settings */
	sercom->USART.CTRLA.reg = pad_conf | SERCOM_USART_CTRLA_MODE(1) | SERCOM_USART_CTRLA_DORD;
	/* Wait for synchronization */
	while(sercom->USART.SYNCBUSY.bit.CTRLB);
	/* Enable transmit and receive and set data size to 8 bits */
	sercom->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_CHSIZE(0);
	/* Load the baud value */
	sercom->USART.BAUD.reg = baud_val;
	/* Wait for synchronization */
	while(sercom->USART.SYNCBUSY.bit.ENABLE);
	/* Enable SERCOM UART */
	sercom->USART.CTRLA.bit.ENABLE = 1;
}

void uart_disable(Sercom *sercom)
{
	/* Wait for synchronization */
	while(sercom->USART.SYNCBUSY.bit.ENABLE);
	/* Disable SERCOM UART */
	sercom->USART.CTRLA.bit.ENABLE = 0;
}

void uart_write_byte(Sercom *sercom, uint8_t data)
{
	/* Wait for Data Register Empty flag */
	while(!sercom->USART.INTFLAG.bit.DRE);
	/* Write the data to DATA register */
	sercom->USART.DATA.reg = (uint16_t)data;
}

uint8_t uart_read_byte(Sercom *sercom)
{
	/* Wait for Receive Complete flag */
	while(!sercom->USART.INTFLAG.bit.RXC);
	/* Check for errors */
	if (sercom->USART.STATUS.bit.PERR || sercom->USART.STATUS.bit.FERR || sercom->USART.STATUS.bit.BUFOVF)
		/* Set the error flag */
		uart_drv_error_flag = true;
	/* Return the read data */
	return((uint8_t)sercom->USART.DATA.reg);
}

void uart_write_buffer_polled(Sercom *sercom, uint8_t *ptr, uint16_t length)
{
	/* Do the following for specified length */
	do {
		/* Wait for Data Register Empty flag */
		while(!sercom->USART.INTFLAG.bit.DRE);
		/* Send data from the buffer */
		sercom->USART.DATA.reg = (uint16_t)*ptr++;
	} while (length--);
}

void uart_read_buffer_polled(Sercom *sercom, uint8_t *ptr, uint16_t length)
{
	/* Do the following for specified length */
	do {
		/* Wait for Receive Complete flag */
		while(!sercom->USART.INTFLAG.bit.RXC);
		/* Check for errors */
		if (sercom->USART.STATUS.bit.PERR || sercom->USART.STATUS.bit.FERR || sercom->USART.STATUS.bit.BUFOVF)
			/* Set the error flag */
			uart_drv_error_flag = true;
		/* Store the read data to the buffer */
		*ptr++ = (uint8_t)sercom->USART.DATA.reg;
	} while (length--);
}