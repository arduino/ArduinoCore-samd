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

#ifndef _SAM_BA_SERIAL_H_
#define _SAM_BA_SERIAL_H_

#include <stdint.h>
#include <stdbool.h>


/* USART buffer size (must be a power of two) */
#define USART_BUFFER_SIZE        (128)

/* Define the default time-out value for USART. */
#define USART_DEFAULT_TIMEOUT    (1000)

/* Xmodem related defines */
/* CRC16  polynomial */
#define CRC16POLY                (0x1021)

#define SHARP_CHARACTER          '#'

/* X/Ymodem protocol: */
#define SOH                      (0x01)
//#define STX                    (0x02)
#define EOT                      (0x04)
#define ACK                      (0x06)
#define NAK                      (0x15)
#define CAN                      (0x18)
#define ESC                      (0x1b)

#define PKTLEN_128               (128)


/**
 * \brief Open the given USART
 */
void serial_open(void);

/**
 * \brief Stops the USART
 */
void serial_close(void);

/**
 * \brief Puts a byte on usart line
 *
 * \param value      Value to put
 *
 * \return \c 1 if function was successfully done, otherwise \c 0.
 */
int serial_putc(int value);

/**
 * \brief Waits and gets a value on usart line
 *
 * \return value read on usart line
 */
int serial_getc(void);

/**
 * \brief Returns true if the SAM-BA Uart received the sharp char
 *
 * \return Returns true if the SAM-BA Uart received the sharp char
 */
int serial_sharp_received(void);

/**
 * \brief This function checks if a character has been received on the usart line
 *
 * \return \c 1 if a byte is ready to be read.
 */
bool serial_is_rx_ready(void);

/**
 * \brief Gets a value on usart line
 *
 * \return value read on usart line
 */
int serial_readc(void);

/**
 * \brief Send buffer on usart line
 *
 * \param data pointer
 * \param number of data to send
 * \return number of data sent
 */
uint32_t serial_putdata(void const* data, uint32_t length); //Send given data (polling)

/**
 * \brief Gets data from usart line
 *
 * \param data pointer
 * \param number of data to get
 * \return value read on usart line
 */
uint32_t serial_getdata(void* data, uint32_t length); //Get data from comm. device

/**
 * \brief Send buffer on usart line using Xmodem protocol
 *
 * \param data pointer
 * \param number of data to send
 * \return number of data sent
 */
uint32_t serial_putdata_xmd(void const* data, uint32_t length); //Send given data (polling) using xmodem (if necessary)

/**
 * \brief Gets data from usart line using Xmodem protocol
 *
 * \param data pointer
 * \param number of data to get
 * \return value read on usart line
 */
uint32_t serial_getdata_xmd(void* data, uint32_t length); //Get data from comm. device using xmodem (if necessary)

/**
 * \brief Compute the CRC
 *
 * \param Char to add to CRC
 * \param Previous CRC
 * \return The new computed CRC
 */
unsigned short serial_add_crc(char c, unsigned short crc);

#endif // _SAM_BA_SERIAL_H_
