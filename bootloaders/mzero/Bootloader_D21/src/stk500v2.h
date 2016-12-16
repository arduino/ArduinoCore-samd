/*----------  GNU PUBLIC LICENZE V3  ----------
*
* Copyright Arduino srl (c) 2015
*
* This file is part of Bootloader_D21.
*
* Bootloader_D21 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License.
*
* bootloader_D21 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Bootloader_D21.  If not, see <http://www.gnu.org/licenses/>.
*
*/


#ifndef STK500V2_H_
#define STK500V2_H_

#include"command.h"
#define NVM_MEMORY        ((volatile uint16_t *)FLASH_ADDR)
#define APP_START_ADDR (0x4000)
#define TIMOUT_PERIOD 8000
volatile uint32_t timeout = 0;
volatile uint32_t current_address = APP_START_ADDR;
uint8_t GET_COMMAND_BYTE()
{
	port_pin_set_output_level(LED_RX_PIN,LED_RX_INACTIVE);		// RX LED management
	
	while(!udi_cdc_is_rx_ready())
	{
		
	}
	
	port_pin_set_output_level(LED_RX_PIN,LED_RX_ACTIVE);		// RX LED management

	return udi_cdc_multi_getc(0);
}

void SEND_RESPONSE(char response)
{
	//if (!udi_cdc_is_tx_ready()) {
	//	/* Fifo full */
	//	udi_cdc_signal_overrun();
	//	ui_com_overflow();//not implemented yet
	//}
	
	port_pin_set_output_level(LED_TX_PIN,LED_TX_INACTIVE);		// TX LED management
	
	while(!udi_cdc_is_tx_ready())
	{
		
	}
	
	port_pin_set_output_level(LED_TX_PIN,LED_TX_ACTIVE);		// TX LED management
	
	udi_cdc_putc(response);
	return;
}

void commit_page()
{
		/*if current row is not written fully, fill rest of row with zeros, on last write in page, 
	whole buffer is written (automatic writes are on)*/
	if(current_address >= APP_START_ADDR)
	{
		while(current_address%NVMCTRL_ROW_SIZE != 0)
		{
			NVM_MEMORY[current_address/2] = 0xffff;
			current_address += 2;
		}
		/*wait until write operation is done*/
		while(!(NVMCTRL->INTFLAG.bit.READY))
		{
			;
		}
	}
}


/* DEFINES FOR DEVICE, pull out to header later TODOTODOTODO*/
//signature for mega2560
#define DEVICE_SIGNATURE 0x1E9801
#define CONF_PARAM_BUILD_NUMBER_LOW 1
#define CONF_PARAM_BUILD_NUMBER_HIGH 2
#define CONF_PARAM_HW_VER 3
#define CONF_PARAM_SW_MAJOR 4
#define CONF_PARAM_SW_MINOR 5


void handle_message(void);

/* based on
http://www.atmel.com/images/doc2591.pdf
AVR068: STK500 communication protocol
*/
#define MAX_MESSAGE_LENGTH 286

typedef enum
{
	STATE_START,
	STATE_GET_SEQUENCE_NUMBER,
	STATE_GET_MESSAGE_SIZE_1,
	STATE_GET_MESSAGE_SIZE_2,
	STATE_GET_TOKEN,
	STATE_GET_DATA,
	STATE_GET_CHECKSUM,

} stk500v2_state;

uint8_t message[MAX_MESSAGE_LENGTH];
uint16_t message_length = 0;
uint8_t sequence_number = 0;

uint8_t get_checksum(uint16_t current_byte_in_message, uint8_t *message_to_check)
{
	uint8_t checksum = 0;
	for(uint16_t i = 0; i<current_byte_in_message; i++) 
	{
		checksum ^= message_to_check[i];
	}
	return checksum;
}

void get_message()
{
	stk500v2_state current_state = STATE_START;
	uint32_t current_byte_in_message = 0;
	uint32_t bytes_in_data_read = 0;
	bool building_message = true;
	while(building_message)
	{
		uint8_t received_char = GET_COMMAND_BYTE();
		message[current_byte_in_message] = received_char;
		current_byte_in_message++;
		switch(current_state)
		{
			case STATE_START:
				if(received_char == MESSAGE_START)
				{
					current_state = STATE_GET_SEQUENCE_NUMBER;
				}
				else
				{
					current_state = STATE_START;
					current_byte_in_message = 0;
				}
				break;
			case STATE_GET_SEQUENCE_NUMBER:
				sequence_number = received_char;		//ignore sequence number, always accept
				current_state = STATE_GET_MESSAGE_SIZE_1;
				break;
			case STATE_GET_MESSAGE_SIZE_1:
				message_length = (received_char << 8)&0xff00;
				current_state = STATE_GET_MESSAGE_SIZE_2;
				break;
			case STATE_GET_MESSAGE_SIZE_2:
				message_length |= received_char;
				current_state = STATE_GET_TOKEN;
				break;
			case STATE_GET_TOKEN:
				if(received_char == TOKEN)
				{
					current_state = STATE_GET_DATA;
					bytes_in_data_read = 0;
				}
				else
				{
					current_state = STATE_START;
					current_byte_in_message = 0;
				}
				break;
			case STATE_GET_DATA:
				bytes_in_data_read++;
				if(bytes_in_data_read == message_length)
				{
					current_state = STATE_GET_CHECKSUM;
				}
				break;
			case STATE_GET_CHECKSUM:
				if(get_checksum(current_byte_in_message-1, message) == received_char)		//ignore last byte (should not include checksum in checksum)
				{
					building_message = false;
				}
				else
				{
					current_state = STATE_START;
					current_byte_in_message = 0;
				}
				break;
		}
	}
	timeout = 0;
	handle_message();

}

void handle_message()
{
	uint8_t return_message[MAX_MESSAGE_LENGTH];
	uint8_t *message_body = message + 5;
	uint16_t return_message_length = 0;
	return_message[0] = message_body[0];
	uint16_t bytes_to_read_or_write = 0;
	switch(return_message[0])
	{
		case CMD_SIGN_ON:
			return_message_length = 11;
			return_message[1] = STATUS_CMD_OK;
			return_message[2] 	=	8;
			return_message[3] 	=	'A';
			return_message[4] 	=	'V';
			return_message[5] 	=	'R';
			return_message[6] 	=	'I';
			return_message[7] 	=	'S';
			return_message[8] 	=	'P';
			return_message[9] 	=	'_';
			return_message[10]	=	'2';
			break;
		case CMD_READ_SIGNATURE_ISP:
			return_message_length = 4;
			uint8_t signature_index = message_body[4];
			return_message[1] = STATUS_CMD_OK;
			return_message[2] = (DEVICE_SIGNATURE>>((2-signature_index)*8))&0xFF;
			return_message[3] = STATUS_CMD_OK;
			break;
		case CMD_GET_PARAMETER:
			return_message_length = 3;
			uint8_t parameter = message_body[1];
			return_message[1] = STATUS_CMD_OK;
			switch(parameter)
			{
				case PARAM_BUILD_NUMBER_LOW:
					return_message[2] = CONF_PARAM_BUILD_NUMBER_LOW;
					break;
				case PARAM_BUILD_NUMBER_HIGH:
					return_message[2] = CONF_PARAM_BUILD_NUMBER_HIGH;
					break;
				case PARAM_HW_VER:
					return_message[2] = CONF_PARAM_HW_VER;
					break;
				case PARAM_SW_MAJOR:
					return_message[2] = CONF_PARAM_SW_MAJOR;
					break;
				case PARAM_SW_MINOR:
					return_message[2] = CONF_PARAM_SW_MINOR;
					break;
				default:
					return_message[2] = 0;
					break;
			}
			break;
		
		case CMD_ENTER_PROGMODE_ISP: //Ignore this one for now
			return_message_length = 2;	
			return_message[1] = STATUS_CMD_OK;
			break;
		case CMD_LEAVE_PROGMODE_ISP:
			return_message_length = 2;
			return_message[1] = STATUS_CMD_OK;
			timeout = (TIMOUT_PERIOD - 500);//reset and enter main app after 500 ms
			break;
		case CMD_SET_PARAMETER: //Ignore this one for now
			return_message_length = 2;
			return_message[1] = STATUS_CMD_OK;
			break;		
		case CMD_LOAD_ADDRESS:
			current_address = (((message_body[1]<<24) | (message_body[2]<<16) | (message_body[3]<<8) | (message_body[4]))<<1);//shift left 1 to ignore msb and convert from word to byte
//			current_address = current_address + APP_START_ADDR;
			return_message_length = 2;
			return_message[1] = STATUS_CMD_OK;
			break;		
		case CMD_SPI_MULTI: //only partially implemented
			switch(message_body[4])
			{
				case 0x30:
					return_message_length = 7;
					return_message[1]	=	STATUS_CMD_OK;
					return_message[2] 	=	0;
					return_message[3] 	=	message_body[4];
					return_message[4] 	=	0;
					return_message[5] 	=	(DEVICE_SIGNATURE>>((2-message_body[6])*8))&0xFF;;
					return_message[6] 	=	STATUS_CMD_OK;
					break;
				default:
					return_message_length = 7;
					return_message[1]	=	STATUS_CMD_OK;
					return_message[2] 	=	0;
					return_message[3] 	=	message_body[4];
					return_message[4] 	=	0;
					return_message[5] 	=	0;
					return_message[6] 	=	STATUS_CMD_OK;
					break;
			}

			break;
		case CMD_CHIP_ERASE_ISP:
			for (uint32_t current_flash_address = APP_START_ADDR; current_flash_address < NVMCTRL_FLASH_SIZE; current_flash_address += NVMCTRL_ROW_SIZE)
			{
				enum status_code error_code;
				do
				{
					error_code = nvm_is_ready();
					NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;

					/* Set address and command */
					NVMCTRL->ADDR.reg  = (uintptr_t)&NVM_MEMORY[current_flash_address / 4];
					NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_ER | NVMCTRL_CTRLA_CMDEX_KEY;
			} while (error_code == STATUS_BUSY);
			}
			return_message_length = 2;
			return_message[1] = STATUS_CMD_OK;
			break;
		case CMD_PROGRAM_FLASH_ISP:

			bytes_to_read_or_write = (message_body[1]<<8) + message_body[2];
			if(current_address >= APP_START_ADDR)
			{
				uint32_t i;
				for(i = 0; i<bytes_to_read_or_write; i+=2)
				{
					NVM_MEMORY[((current_address+i)/2)] = ((message_body[10+i])+(message_body[10+i+1]<<8));
				}
				//nvm_write_buffer(current_address, message_body+10, bytes_to_read_or_write);		
			}
			current_address += bytes_to_read_or_write;
			commit_page();
			return_message_length = 2;
			return_message[1] = STATUS_CMD_OK;
			break;
		case CMD_READ_FLASH_ISP:
			bytes_to_read_or_write = (message_body[1]<<8) + message_body[2];
			return_message_length = bytes_to_read_or_write + 3;
			return_message[1] = STATUS_CMD_OK;
			if(current_address >= APP_START_ADDR)
			{
				uint32_t i;
				for(i = 0; i<bytes_to_read_or_write; i+=2)
				{
					return_message[2+i] = NVM_MEMORY[((current_address+i)/2)]&0xff;
					return_message[2+i+1] = NVM_MEMORY[((current_address+i)/2)]>>8;
				}
				//nvm_read_buffer(current_address, return_message+2, bytes_to_read_or_write);
			}
			else
			{
				for(uint32_t i = 0; i<(message_body[1]<<8) + message_body[2]; i++)
				{
					return_message[i+2] = 0xFFFF;
				}
			}
			current_address += bytes_to_read_or_write;
			return_message[return_message_length-1] = STATUS_CMD_OK;
			break;	
	}
	//send message back
	uint8_t checksum = 0;
	checksum ^= MESSAGE_START;
	SEND_RESPONSE(MESSAGE_START);
	checksum ^= sequence_number;
	SEND_RESPONSE(sequence_number);
	checksum ^= (return_message_length>>8)&0xFF;
	SEND_RESPONSE((return_message_length>>8)&0xFF);
	checksum ^= return_message_length&0xFF;
	SEND_RESPONSE((return_message_length)&0xFF);
	checksum ^= TOKEN;
	SEND_RESPONSE(TOKEN);
	for(uint16_t i = 0; i< return_message_length; i++)
	{
		SEND_RESPONSE(return_message[i]);
	}
	uint8_t body_checksum = get_checksum(return_message_length, return_message);
	checksum ^= body_checksum;
	SEND_RESPONSE(checksum);
}



#endif /* STK500V2_H_ */