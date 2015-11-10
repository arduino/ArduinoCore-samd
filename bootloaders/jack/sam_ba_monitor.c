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

#include "sam.h"
#include <string.h>
#include "sam_ba_monitor.h"
#include "usart_sam_ba.h"
#include "uart_driver.h"
#include "compiler.h"
#include "cdc_enumerate.h"

const char RomBOOT_Version[] = SAM_BA_VERSION;
const char RomBOOT_ExtendedCapabilities[] = "[Arduino:XYZ]";

/* Provides one common interface to handle both USART and USB-CDC */
typedef struct
{
	/* send one byte of data */
	int (*put_c)(int value);
	/* Get one byte */
	int (*get_c)(void);
	/* Receive buffer not empty */
	bool (*is_rx_ready)(void);
	/* Send given data (polling) */
	uint32_t (*putdata)(void const* data, uint32_t length);
	/* Get data from comm. device */
	uint32_t (*getdata)(void* data, uint32_t length);
	/* Send given data (polling) using xmodem (if necessary) */
	uint32_t (*putdata_xmd)(void const* data, uint32_t length);
	/* Get data from comm. device using xmodem (if necessary) */
	uint32_t (*getdata_xmd)(void* data, uint32_t length);
} t_monitor_if;

#if SAM_BA_INTERFACE == SAM_BA_UART_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
/* Initialize structures with function pointers from supported interfaces */
const t_monitor_if uart_if =
{ usart_putc, usart_getc, usart_is_rx_ready, usart_putdata, usart_getdata,
		usart_putdata_xmd, usart_getdata_xmd };
#endif

#if SAM_BA_INTERFACE == SAM_BA_USBCDC_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
//Please note that USB doesn't use Xmodem protocol, since USB already includes flow control and data verification
//Data are simply forwarded without further coding.
const t_monitor_if usbcdc_if =
{ cdc_putc, cdc_getc, cdc_is_rx_ready, cdc_write_buf,
		cdc_read_buf, cdc_write_buf, cdc_read_buf_xmd };
#endif

/* The pointer to the interface object use by the monitor */
t_monitor_if * ptr_monitor_if;

/* b_terminal_mode mode (ascii) or hex mode */
volatile bool b_terminal_mode = false;
volatile bool b_sam_ba_interface_usart = false;

void sam_ba_monitor_init(uint8_t com_interface)
{
#if SAM_BA_INTERFACE == SAM_BA_UART_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES       
	//Selects the requested interface for future actions
	if (com_interface == SAM_BA_INTERFACE_USART) {
		ptr_monitor_if = (t_monitor_if*) &uart_if;
		b_sam_ba_interface_usart = true;
	}
#endif
#if SAM_BA_INTERFACE == SAM_BA_USBCDC_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES        
	if (com_interface == SAM_BA_INTERFACE_USBCDC) {
		ptr_monitor_if = (t_monitor_if*) &usbcdc_if;
	}
#endif
}

/**
 * \brief This function allows data rx by USART
 *
 * \param *data  Data pointer
 * \param length Length of the data
 */
void sam_ba_putdata_term(uint8_t* data, uint32_t length)
{
	uint8_t temp, buf[12], *data_ascii;
	uint32_t i, int_value;

	if (b_terminal_mode)
	{
		if (length == 4)
			int_value = *(uint32_t *) data;
		else if (length == 2)
			int_value = *(uint16_t *) data;
		else
			int_value = *(uint8_t *) data;

		data_ascii = buf + 2;
		data_ascii += length * 2 - 1;

		for (i = 0; i < length * 2; i++)
		{
			temp = (uint8_t) (int_value & 0xf);

			if (temp <= 0x9)
				*data_ascii = temp | 0x30;
			else
				*data_ascii = temp + 0x37;

			int_value >>= 4;
			data_ascii--;
		}
		buf[0] = '0';
		buf[1] = 'x';
		buf[length * 2 + 2] = '\n';
		buf[length * 2 + 3] = '\r';
		ptr_monitor_if->putdata(buf, length * 2 + 4);
	}
	else
		ptr_monitor_if->putdata(data, length);
	return;
}

volatile uint32_t sp;
void call_applet(uint32_t address)
{
	uint32_t app_start_address;

	cpu_irq_disable();

	sp = __get_MSP();

	/* Rebase the Stack Pointer */
	__set_MSP(*(uint32_t *) address);

	/* Load the Reset Handler address of the application */
	app_start_address = *(uint32_t *)(address + 4);

	/* Jump to application Reset Handler in the application */
	asm("bx %0"::"r"(app_start_address));
}


uint32_t current_number;
uint32_t i, length;
uint8_t command, *ptr_data, *ptr, data[SIZEBUFMAX];
uint8_t j;
uint32_t u32tmp;

uint32_t PAGE_SIZE, PAGES, MAX_FLASH;

/**
 * \brief This function starts the SAM-BA monitor.
 */
void sam_ba_monitor_run(void)
{
	uint32_t pageSizes[] = { 8, 16, 32, 64, 128, 256, 512, 1024 };
	PAGE_SIZE = pageSizes[NVMCTRL->PARAM.bit.PSZ];
	PAGES = NVMCTRL->PARAM.bit.NVMP;
	MAX_FLASH = PAGE_SIZE * PAGES;

	ptr_data = NULL;
	command = 'z';
	while (1) {
		sam_ba_monitor_loop();
	}
}

// Prints a 32-bit integer in hex.
void put_uint32(uint32_t n) {
	char buff[8];
	int i;
	for (i=0; i<8; i++) {
		int d = n & 0XF;
		n = (n >> 4);

		buff[7-i] = d > 9 ? 'A' + d - 10 : '0' + d;
	}
	ptr_monitor_if->putdata(buff, 8);
}

void sam_ba_monitor_loop(void)
{
	length = ptr_monitor_if->getdata(data, SIZEBUFMAX);
	ptr = data;
	for (i = 0; i < length; i++, ptr++)
	{
		if (*ptr == 0xff) continue;

		if (*ptr == '#')
		{
			if (b_terminal_mode)
			{
				ptr_monitor_if->putdata("\n\r", 2);
			}
			if (command == 'S')
			{
				//Check if some data are remaining in the "data" buffer
				if(length>i)
				{
					//Move current indexes to next avail data (currently ptr points to "#")
					ptr++;
					i++;
					//We need to add first the remaining data of the current buffer already read from usb
					//read a maximum of "current_number" bytes
					u32tmp=min((length-i),current_number);
					memcpy(ptr_data, ptr, u32tmp);
					i += u32tmp;
					ptr += u32tmp;
					j = u32tmp;
				}
				//update i with the data read from the buffer
				i--;
				ptr--;
				//Do we expect more data ?
				if(j<current_number)
					ptr_monitor_if->getdata_xmd(ptr_data, current_number-j);
				
				__asm("nop");
			}
			else if (command == 'R')
			{
				ptr_monitor_if->putdata_xmd(ptr_data, current_number);
			}
			else if (command == 'O')
			{
				*ptr_data = (char) current_number;
			}
			else if (command == 'H')
			{
				*((uint16_t *) ptr_data) = (uint16_t) current_number;
			}
			else if (command == 'W')
			{
				*((int *) ptr_data) = current_number;
			}
			else if (command == 'o')
			{
				sam_ba_putdata_term(ptr_data, 1);
			}
			else if (command == 'h')
			{
				current_number = *((uint16_t *) ptr_data);
				sam_ba_putdata_term((uint8_t*) &current_number, 2);
			}
			else if (command == 'w')
			{
				current_number = *((uint32_t *) ptr_data);
				sam_ba_putdata_term((uint8_t*) &current_number, 4);
			}
			else if (command == 'G')
			{
				call_applet(current_number);
				/* Rebase the Stack Pointer */
				__set_MSP(sp);
				cpu_irq_enable();
				if (b_sam_ba_interface_usart) {
					ptr_monitor_if->put_c(0x6);
				}
			}
			else if (command == 'T')
			{
				b_terminal_mode = 1;
				ptr_monitor_if->putdata("\n\r", 2);
			}
			else if (command == 'N')
			{
				if (b_terminal_mode == 0)
				{
					ptr_monitor_if->putdata("\n\r", 2);
				}
				b_terminal_mode = 0;
			}
			else if (command == 'V')
			{
				ptr_monitor_if->putdata("v", 1);
				ptr_monitor_if->putdata((uint8_t *) RomBOOT_Version,
						strlen(RomBOOT_Version));
				ptr_monitor_if->putdata(" ", 1);
				ptr_monitor_if->putdata((uint8_t *) RomBOOT_ExtendedCapabilities,
						strlen(RomBOOT_ExtendedCapabilities));
				ptr_monitor_if->putdata(" ", 1);
				ptr = (uint8_t*) &(__DATE__);
				i = 0;
				while (*ptr++ != '\0')
					i++;
				ptr_monitor_if->putdata((uint8_t *) &(__DATE__), i);
				ptr_monitor_if->putdata(" ", 1);
				i = 0;
				ptr = (uint8_t*) &(__TIME__);
				while (*ptr++ != '\0')
					i++;
				ptr_monitor_if->putdata((uint8_t *) &(__TIME__), i);
				ptr_monitor_if->putdata("\n\r", 2);
			}
			else if (command == 'X')
			{
				// Syntax: X[ADDR]#
				// Erase the flash memory starting from ADDR to the end of flash.

				// Note: the flash memory is erased in ROWS, that is in block of 4 pages.
				//       Even if the starting address is the last byte of a ROW the entire
				//       ROW is erased anyway.

				uint32_t dst_addr = current_number; // starting address

				while (dst_addr < MAX_FLASH) {
					// Execute "ER" Erase Row
					NVMCTRL->ADDR.reg = dst_addr / 2;
					NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
					while (NVMCTRL->INTFLAG.bit.READY == 0)
						;
					dst_addr += PAGE_SIZE * 4; // Skip a ROW
				}

				// Notify command completed
				ptr_monitor_if->putdata("X\n\r", 3);
			}
			else if (command == 'Y')
			{
				// This command writes the content of a buffer in SRAM into flash memory.

				// Syntax: Y[ADDR],0#
				// Set the starting address of the SRAM buffer.

				// Syntax: Y[ROM_ADDR],[SIZE]#
				// Write the first SIZE bytes from the SRAM buffer (previously set) into
				// flash memory starting from address ROM_ADDR

				static uint32_t *src_buff_addr = NULL;

				if (current_number == 0) {
					// Set buffer address
					src_buff_addr = ptr_data;

				} else {
					// Write to flash
					uint32_t size = current_number/4;
					uint32_t *src_addr = src_buff_addr;
					uint32_t *dst_addr = ptr_data;

					// Set automatic page write
					NVMCTRL->CTRLB.bit.MANW = 0;

					// Do writes in pages
					while (size) {
						// Execute "PBC" Page Buffer Clear
						NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
						while (NVMCTRL->INTFLAG.bit.READY == 0)
							;

						// Fill page buffer
						uint32_t i;
						for (i=0; i<(PAGE_SIZE/4) && i<size; i++) {
							dst_addr[i] = src_addr[i];
						}

						// Execute "WP" Write Page
						//NVMCTRL->ADDR.reg = ((uint32_t)dst_addr) / 2;
						NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
						while (NVMCTRL->INTFLAG.bit.READY == 0)
							;

						// Advance to next page
						dst_addr += i;
						src_addr += i;
						size     -= i;
					}
				}

				// Notify command completed
				ptr_monitor_if->putdata("Y\n\r", 3);
			}
			else if (command == 'Z')
			{
				// This command calculate CRC for a given area of memory.
				// It's useful to quickly check if a transfer has been done
				// successfully.

				// Syntax: Z[START_ADDR],[SIZE]#
				// Returns: Z[CRC]#

				uint8_t *data = (uint8_t *)ptr_data;
				uint32_t size = current_number;
				uint16_t crc = 0;
				uint32_t i = 0;
				for (i=0; i<size; i++)
					crc = add_crc(*data++, crc);

				// Send response
				ptr_monitor_if->putdata("Z", 1);
				put_uint32(crc);
				ptr_monitor_if->putdata("#\n\r", 3);
			}

			command = 'z';
			current_number = 0;

			if (b_terminal_mode)
			{
				ptr_monitor_if->putdata(">", 1);
			}
		}
		else
		{
			if (('0' <= *ptr) && (*ptr <= '9'))
			{
				current_number = (current_number << 4) | (*ptr - '0');
			}
			else if (('A' <= *ptr) && (*ptr <= 'F'))
			{
				current_number = (current_number << 4) | (*ptr - 'A' + 0xa);
			}
			else if (('a' <= *ptr) && (*ptr <= 'f'))
			{
				current_number = (current_number << 4) | (*ptr - 'a' + 0xa);
			}
			else if (*ptr == ',')
			{
				ptr_data = (uint8_t *) current_number;
				current_number = 0;
			}
			else
			{
				command = *ptr;
				current_number = 0;
			}
		}
	}
}




