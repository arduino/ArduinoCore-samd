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

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdio.h>
#include <stdbool.h>
#include <sam.h>

#define PINMUX_UNUSED          0xFFFFFFFF
#define GCLK_ID_SERCOM0_CORE   0x14

/* SERCOM UART available pad settings */
enum uart_pad_settings {
	UART_RX_PAD0_TX_PAD2 = SERCOM_USART_CTRLA_RXPO(0) | SERCOM_USART_CTRLA_TXPO(1),
	UART_RX_PAD1_TX_PAD2 = SERCOM_USART_CTRLA_RXPO(1) | SERCOM_USART_CTRLA_TXPO(1),
	UART_RX_PAD2_TX_PAD0 = SERCOM_USART_CTRLA_RXPO(2),
	UART_RX_PAD3_TX_PAD0 = SERCOM_USART_CTRLA_RXPO(3),
	UART_RX_PAD1_TX_PAD0 = SERCOM_USART_CTRLA_RXPO(1),
	UART_RX_PAD3_TX_PAD2 = SERCOM_USART_CTRLA_RXPO(3) | SERCOM_USART_CTRLA_TXPO(1),
};

/**
 * \brief Initializes the UART
 *
 * \param Pointer to SERCOM instance
 * \param Baud value corresponding to the desired baudrate
 * \param SERCOM pad settings
 */
void uart_basic_init(Sercom *sercom, uint16_t baud_val, enum uart_pad_settings pad_conf);

/**
 * \brief Disables UART interface
 *
 * \param Pointer to SERCOM instance
 */
void uart_disable(Sercom *sercom);

/**
 * \brief Sends a single byte through UART interface
 *
 * \param Pointer to SERCOM instance
 * \param Data to send
 */
void uart_write_byte(Sercom *sercom, uint8_t data);

/**
 * \brief Reads a single character from UART interface
 *
 * \param Pointer to SERCOM instance
 * \return Data byte read
 */
uint8_t uart_read_byte(Sercom *sercom);

/**
 * \brief Sends buffer on UART interface
 *
 * \param Pointer to SERCOM instance
 * \param Pointer to data to send
 * \param Number of bytes to send
 */
void uart_write_buffer_polled(Sercom *sercom, uint8_t *ptr, uint16_t length);

/**
 * \brief Reads data on UART interface
 *
 * \param Pointer to SERCOM instance
 * \param Pointer to store read data
 * \param Number of bytes to read
 */
void uart_read_buffer_polled(Sercom *sercom, uint8_t *ptr, uint16_t length);

#endif
