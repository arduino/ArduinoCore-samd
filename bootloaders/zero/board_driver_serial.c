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

#include "board_driver_serial.h"

bool uart_drv_error_flag = false;

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
