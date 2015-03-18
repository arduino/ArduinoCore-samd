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

#ifndef _USART_SAM_BA_H_
#define _USART_SAM_BA_H_

#include "stdint.h"
#include "stdbool.h"


/* USART buffer size (must be a power of two) */
#define USART_BUFFER_SIZE        128

/* Define the default time-out value for USART. */
#define USART_DEFAULT_TIMEOUT    1000

/* Xmodem related defines */
/* CRC16  polynomial */
#define CRC16POLY                0x1021

#define SHARP_CHARACTER          '#'

/* X/Ymodem protocol: */
#define SOH                      0x01
//#define STX                    0x02
#define EOT                      0x04
#define ACK                      0x06
#define NAK                      0x15
#define CAN                      0x18
#define ESC                      0x1b

#define PKTLEN_128               128


/**
 * \brief Open the given USART
 */
void usart_open(void);

/**
 * \brief Stops the USART
 */
void usart_close(void);

/**
 * \brief Puts a byte on usart line
 *
 * \param value      Value to put
 *
 * \return \c 1 if function was successfully done, otherwise \c 0.
 */
int usart_putc(int value);

/**
 * \brief Waits and gets a value on usart line
 *
 * \return value read on usart line
 */
int usart_getc(void);

/**
 * \brief Returns true if the SAM-BA Uart received the sharp char
 *
 * \return Returns true if the SAM-BA Uart received the sharp char
 */
int usart_sharp_received(void);

/**
 * \brief This function checks if a character has been received on the usart line
 *
 * \return \c 1 if a byte is ready to be read.
 */
bool usart_is_rx_ready(void);

/**
 * \brief Gets a value on usart line
 *
 * \return value read on usart line
 */
int usart_readc(void);

/**
 * \brief Send buffer on usart line
 *
 * \param data pointer
 * \param number of data to send
 * \return number of data sent
 */
uint32_t usart_putdata(void const* data, uint32_t length); //Send given data (polling)

/**
 * \brief Gets data from usart line
 *
 * \param data pointer
 * \param number of data to get
 * \return value read on usart line
 */
uint32_t usart_getdata(void* data, uint32_t length); //Get data from comm. device

/**
 * \brief Send buffer on usart line using Xmodem protocol
 *
 * \param data pointer
 * \param number of data to send
 * \return number of data sent
 */
uint32_t usart_putdata_xmd(void const* data, uint32_t length); //Send given data (polling) using xmodem (if necessary)

/**
 * \brief Gets data from usart line using Xmodem protocol
 *
 * \param data pointer
 * \param number of data to get
 * \return value read on usart line
 */
uint32_t usart_getdata_xmd(void* data, uint32_t length); //Get data from comm. device using xmodem (if necessary)

/**
 * \brief Compute the CRC
 *
 * \param Char to add to CRC
 * \param Previous CRC
 * \return The new computed CRC
 */
unsigned short add_crc(char c, unsigned short crc);

uint8_t getPacket(uint8_t *pData, uint8_t sno);

#endif // _USART_SAM_BA_H_
