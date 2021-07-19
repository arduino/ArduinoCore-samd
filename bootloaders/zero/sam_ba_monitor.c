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

#include "sam.h"
#include <string.h>
#include "sam_ba_monitor.h"
#include "sam_ba_serial.h"
#include "board_driver_serial.h"
#include "board_driver_usb.h"
#include "board_driver_jtag.h"
#include "sam_ba_usb.h"
#include "sam_ba_cdc.h"
#include "board_driver_led.h"
#include <stdlib.h>

const char RomBOOT_Version[] = SAM_BA_VERSION;
// X = Chip Erase, Y = Write Buffer, Z = Checksum Buffer, P = Secure Bit Aware
const char RomBOOT_ExtendedCapabilities[] = "[Arduino:XYZP]";

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

#if defined(SAM_BA_UART_ONLY)  ||  defined(SAM_BA_BOTH_INTERFACES)
/* Initialize structures with function pointers from supported interfaces */
const t_monitor_if uart_if =
{
  .put_c =       serial_putc,
  .get_c =       serial_getc,
  .is_rx_ready = serial_is_rx_ready,
  .putdata =     serial_putdata,
  .getdata =     serial_getdata,
  .putdata_xmd = serial_putdata_xmd,
  .getdata_xmd = serial_getdata_xmd
};
#endif

#if defined(SAM_BA_USBCDC_ONLY)  ||  defined(SAM_BA_BOTH_INTERFACES)
//Please note that USB doesn't use Xmodem protocol, since USB already includes flow control and data verification
//Data are simply forwarded without further coding.
const t_monitor_if usbcdc_if =
{
  .put_c =         cdc_putc,
  .get_c =         cdc_getc,
  .is_rx_ready =   cdc_is_rx_ready,
  .putdata =       cdc_write_buf,
  .getdata =       cdc_read_buf,
  .putdata_xmd =   cdc_write_buf,
  .getdata_xmd =   cdc_read_buf_xmd
};
#endif

/* The pointer to the interface object use by the monitor */
t_monitor_if * ptr_monitor_if;

#ifdef SECURE_BY_DEFAULT
bool b_security_enabled = true;
#else
bool b_security_enabled = false;
#endif

/* b_terminal_mode mode (ascii) or hex mode */
volatile bool b_terminal_mode = false;
volatile bool b_sam_ba_interface_usart = false;

/* Pulse generation counters to keep track of the time remaining for each pulse type */
#define TX_RX_LED_PULSE_PERIOD 100
volatile uint16_t txLEDPulse = 0; // time remaining for Tx LED pulse
volatile uint16_t rxLEDPulse = 0; // time remaining for Rx LED pulse

void sam_ba_monitor_init(uint8_t com_interface)
{
#if defined(SAM_BA_UART_ONLY)  ||  defined(SAM_BA_BOTH_INTERFACES)
  //Selects the requested interface for future actions
  if (com_interface == SAM_BA_INTERFACE_USART)
  {
    ptr_monitor_if = (t_monitor_if*) &uart_if;
    b_sam_ba_interface_usart = true;
  }
#endif
#if defined(SAM_BA_USBCDC_ONLY)  ||  defined(SAM_BA_BOTH_INTERFACES)
  if (com_interface == SAM_BA_INTERFACE_USBCDC)
  {
    ptr_monitor_if = (t_monitor_if*) &usbcdc_if;
  }
#endif
}

/*
 * Central SAM-BA monitor putdata function using the board LEDs
 */
static uint32_t sam_ba_putdata(t_monitor_if* pInterface, void const* data, uint32_t length)
{
	uint32_t result ;

	result=pInterface->putdata(data, length);

	LEDTX_on();
	txLEDPulse = TX_RX_LED_PULSE_PERIOD;

	return result;
}

/*
 * Central SAM-BA monitor getdata function using the board LEDs
 */
static uint32_t sam_ba_getdata(t_monitor_if* pInterface, void* data, uint32_t length)
{
	uint32_t result ;

	result=pInterface->getdata(data, length);

	if (result)
	{
		LEDRX_on();
		rxLEDPulse = TX_RX_LED_PULSE_PERIOD;
	}

	return result;
}

/*
 * Central SAM-BA monitor putdata function using the board LEDs
 */
static uint32_t sam_ba_putdata_xmd(t_monitor_if* pInterface, void const* data, uint32_t length)
{
	uint32_t result ;

	result=pInterface->putdata_xmd(data, length);

	LEDTX_on();
	txLEDPulse = TX_RX_LED_PULSE_PERIOD;

	return result;
}

/*
 * Central SAM-BA monitor getdata function using the board LEDs
 */
static uint32_t sam_ba_getdata_xmd(t_monitor_if* pInterface, void* data, uint32_t length)
{
	uint32_t result ;

	result=pInterface->getdata_xmd(data, length);

	if (result)
	{
		LEDRX_on();
		rxLEDPulse = TX_RX_LED_PULSE_PERIOD;
	}

	return result;
}

/**
 * \brief This function allows data emission by USART
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
    sam_ba_putdata(ptr_monitor_if, buf, length * 2 + 4);
  }
  else
    sam_ba_putdata(ptr_monitor_if, data, length);
  return;
}

#ifndef SECURE_BY_DEFAULT
volatile uint32_t sp;
void call_applet(uint32_t address)
{
  if (b_security_enabled) {
    return;
  }

  uint32_t app_start_address;

  __disable_irq();

  sp = __get_MSP();

  /* Rebase the Stack Pointer */
  __set_MSP(*(uint32_t *) address);

  /* Load the Reset Handler address of the application */
  app_start_address = *(uint32_t *)(address + 4);

  /* Jump to application Reset Handler in the application */
  asm("bx %0"::"r"(app_start_address));
}
#endif

uint32_t current_number;
uint32_t erased_from = 0;
uint32_t i, length;
uint8_t command, *ptr_data, *ptr, data[SIZEBUFMAX];
uint8_t j;
uint32_t u32tmp;

uint32_t PAGE_SIZE, PAGES, MAX_FLASH;

// Prints a 32-bit integer in hex.
static void put_uint32(uint32_t n)
{
  char buff[8];
  int i;
  for (i=0; i<8; i++)
  {
    int d = n & 0XF;
    n = (n >> 4);

    buff[7-i] = d > 9 ? 'A' + d - 10 : '0' + d;
  }
  sam_ba_putdata( ptr_monitor_if, buff, 8);
}

static void eraseFlash(uint32_t dst_addr)
{
	erased_from = dst_addr;
	while (dst_addr < MAX_FLASH)
	{
		// Execute "ER" Erase Row
		NVMCTRL->ADDR.reg = dst_addr / 2;
		NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
		while (NVMCTRL->INTFLAG.bit.READY == 0)
			;
		dst_addr += PAGE_SIZE * 4; // Skip a ROW
	}
}

#ifdef ENABLE_JTAG_LOAD
static uint32_t offset = __UINT32_MAX__;
static bool flashNeeded = false;
#endif

static void sam_ba_monitor_loop(void)
{
  length = sam_ba_getdata(ptr_monitor_if, data, SIZEBUFMAX);
  ptr = data;

  for (i = 0; i < length; i++, ptr++)
  {
    if (*ptr == 0xff) continue;

    if (*ptr == '#')
    {
      if (b_terminal_mode)
      {
        sam_ba_putdata(ptr_monitor_if, "\n\r", 2);
      }
      if (command == 'S') // Write memory (normally RAM, but might be flash, if client handles the Flash MCU commands?)
      {
        //Check if some data are remaining in the "data" buffer
        if(length>i)
        {
          //Move current indexes to next avail data (currently ptr points to "#")
          ptr++;
          i++;

          //We need to add first the remaining data of the current buffer already read from usb
          //read a maximum of "current_number" bytes
          if ((length-i) < current_number)
          {
            u32tmp=(length-i);
          }
          else
          {
            u32tmp=current_number;
          }

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
          sam_ba_getdata_xmd(ptr_monitor_if, ptr_data, current_number-j);

        __asm("nop");
      }
      else if (command == 'R') // Read memory (flash or RAM)
      {
        // Flash memory starts at address 0 and runs to flash size 0x40000 (256 KByte)

        // Internal RWW section is at adress 0x400000. RWW is flash used for EEPROM emulation. Will not let anyone read that, when in secure mode, either.
        // Bootloader ends at 0x1FFF, so user programs start at 0x2000
        // RAM starts at 0x20000000, so redirect FLASH reads into RAM reads, when in secure mode
        if (b_security_enabled && ((uint32_t)ptr_data >= 0x0000 && (uint32_t)ptr_data < 0x20000000))
        {
          ptr_data = (uint8_t *)0x20005000;
        }

        sam_ba_putdata_xmd(ptr_monitor_if, ptr_data, current_number);
      }
      else if (command == 'O') // write byte
      {
        *ptr_data = (char) current_number;
      }
      else if (command == 'H') // Write half word
      {
        *((uint16_t *) ptr_data) = (uint16_t) current_number;
      }
      else if (command == 'W') // Write word
      {
        *((int *) ptr_data) = current_number;
      }
      else if (command == 'o') // Read byte
      {
        // Flash memory starts at address 0 and runs to flash size 0x40000 (256 KByte). RAM starts at 0x20000000.
        // Intern RWW section is at adress 0x400000. RWW is flash used for EEPROM emulation. Will not let anyone read that, when in secure mode, either.
        // BOSSA reads address 0 to check something, but using read word instead of read byte, but in any case allow reading first byte
        // Bootloader ends at 0x1FFF, so user programs start at 0x2000
        if (b_security_enabled && ((uint32_t)ptr_data > 0x0003 && (uint32_t)ptr_data < 0x20000000))
        {
          ptr_data = (uint8_t*) &current_number;
        }

        sam_ba_putdata_term(ptr_data, 1);
      }
      else if (command == 'h') // Read half word
      {
        // Flash memory starts at address 0 and runs to flash size 0x40000 (256 KByte). RAM starts at 0x20000000.
        // Intern RWW section is at adress 0x400000. RWW is flash used for EEPROM emulation. Will not let anyone read that, when in secure mode, either.
        // BOSSA reads address 0 to check something, but using read word instead of read byte, but in any case allow reading first byte
        // Bootloader ends at 0x1FFF, so user programs start at 0x2000
        if (b_security_enabled && ((uint32_t)ptr_data > 0x0003 && (uint32_t)ptr_data < 0x20000000))
        {
          current_number = 0;
        }
        else
        {
          current_number = *((uint16_t *) ptr_data);
        }

        sam_ba_putdata_term((uint8_t*) &current_number, 2);
      }
      else if (command == 'w') // Read word
      {
        // Flash memory starts at address 0 and runs to flash size 0x40000 (256 KByte). RAM starts at 0x20000000.
        // Intern RWW section is at adress 0x400000. RWW is flash used for EEPROM emulation. Will not let anyone read that, when in secure mode, either.
        // BOSSA reads address 0 to check something, but using read word instead of read byte, but in any case allow reading first byte
        // Bootloader ends at 0x1FFF, so user programs start at 0x2000
        if (b_security_enabled && ((uint32_t)ptr_data > 0x0003 && (uint32_t)ptr_data < 0x20000000))
        {
          current_number = 0;
        }
        else
        {
          current_number = *((uint32_t *) ptr_data);
        }

        sam_ba_putdata_term((uint8_t*) &current_number, 4);
      }
#ifndef SECURE_BY_DEFAULT
      else if (!b_security_enabled && command == 'G')  // Execute code. Will not allow when security is enabled.
      {
        call_applet(current_number);
        /* Rebase the Stack Pointer */
        __set_MSP(sp);
        __enable_irq();
        if (b_sam_ba_interface_usart) {
          ptr_monitor_if->put_c(0x6);
        }
      }
#endif
      else if (command == 'T') // Turn on terminal mode
      {
        b_terminal_mode = 1;
        sam_ba_putdata(ptr_monitor_if, "\n\r", 2);
      }
      else if (command == 'N') // Turn off terminal mode
      {
        if (b_terminal_mode == 0)
        {
          sam_ba_putdata( ptr_monitor_if, "\n\r", 2);
        }
        b_terminal_mode = 0;
      }
      else if (command == 'V') // Read version information
      {
        sam_ba_putdata( ptr_monitor_if, "v", 1);
        sam_ba_putdata( ptr_monitor_if, (uint8_t *) RomBOOT_Version, strlen(RomBOOT_Version));
        sam_ba_putdata( ptr_monitor_if, " ", 1);
        sam_ba_putdata( ptr_monitor_if, (uint8_t *) RomBOOT_ExtendedCapabilities, strlen(RomBOOT_ExtendedCapabilities));
        ptr = (uint8_t*) &(" " __DATE__ " " __TIME__ "\n\r");
        sam_ba_putdata( ptr_monitor_if, ptr, strlen(ptr));
      }
      else if (command == 'X') // Erase flash
      {
        // Syntax: X[ADDR]#
        // Erase the flash memory starting from ADDR to the end of flash.

        // Note: the flash memory is erased in ROWS, that is in block of 4 pages.
        //       Even if the starting address is the last byte of a ROW the entire
        //       ROW is erased anyway.

        // BOSSAC.exe always erase with 0x2000 as argument, but an attacker might try to erase just parts of the flash, to be able to copy or analyze the untouched parts.
        // To mitigate this, always erase all sketch flash, that is, starting from address 0x2000. This butloader always assume 8 KByte for itself, and sketch starting at 0x2000.
        eraseFlash(b_security_enabled ? 0x2000 : current_number);
        // Notify command completed
        sam_ba_putdata( ptr_monitor_if, "X\n\r", 3);
      }
      else if (command == 'Y') // Write buffer to flash
      {
        // This command writes the content of a buffer in SRAM into flash memory.

        // Syntax: Y[ADDR],0#
        // Set the starting address of the SRAM buffer.

        // Syntax: Y[ROM_ADDR],[SIZE]#
        // Write the first SIZE bytes from the SRAM buffer (previously set) into
        // flash memory starting from address ROM_ADDR

        static uint32_t *src_buff_addr = NULL;

        if (current_number == 0)
        {
          // Set buffer address
          src_buff_addr = (uint32_t*)ptr_data;
        }
        else
        {
          if (b_security_enabled && erased_from != 0x2000)
          {
            // To mitigate that an attacker might not use the ordinary BOSSA method of erasing flash before programming,
            // always erase flash, if it hasn't been done already.
            eraseFlash(0x2000);
          }

          // Write to flash
          uint32_t size = current_number/4;
          uint32_t *src_addr = src_buff_addr;
          uint32_t *dst_addr = (uint32_t*)ptr_data;

#ifdef ENABLE_JTAG_LOAD

          if ((uint32_t)dst_addr == 0x40000) {
              if (jtagInit() != 0) {
                // fail!
                sam_ba_putdata( ptr_monitor_if, "y\n\r", 3);
                return;
              }

              // content of the first flash page:
              // offset (32) : length(32) : sha256sum(256) : type (32) : force (32) = 48 bytes
              // for every section; check last sector of the flash to understand if reflash is needed
              externalFlashSignatures data[3];
              jtagFlashReadBlock(LAST_FLASH_PAGE, sizeof(data), (uint8_t*)data);
              externalFlashSignatures* newData = (externalFlashSignatures*)src_addr;
              for (int k=0; k<3; k++) {
                if (newData[k].force != 0) {
                  offset = newData[k].offset;
                  flashNeeded = true;
                  break;
                }
                if ((data[k].type == newData[k].type) || (data[k].type == 0xFFFFFFFF)) {
                  if (newData[k].offset < offset) {
                    offset = newData[k].offset;
                  }
                  if (memcmp(data[k].sha256sum, newData[k].sha256sum, 32) != 0) {
                    flashNeeded = true;
                    break;
                  }
                }
              }

              // merge old page and new page
              for (int k=0; k<3; k++) {
                if (newData[k].type != 0xFFFFFFFF) {
                  memcpy(&data[k], &newData[k], sizeof(newData[k]));
                }
              }

              jtagFlashEraseBlock(SCRATCHPAD_FLASH_PAGE);
              // write first page to SCRATCHPAD_FLASH_PAGE (to allow correct verification)
              for (int j =0; j<size; ) {
                jtagFlashWriteBlock(SCRATCHPAD_FLASH_PAGE + j*4, 512, (uint32_t*)&src_addr[j]);
                j += 128;
              }

              // write real structure with checksums to LAST_FLASH_PAGE
              jtagFlashWriteBlock(LAST_FLASH_PAGE, sizeof(data),  (uint32_t*)data);
              goto end;
          }


          if ((uint32_t)dst_addr >= 0x41000) {

            if (flashNeeded == false) {
              goto end;
            }

            uint32_t rebasedAddress = (uint32_t)dst_addr - 0x41000 + offset;
            if (rebasedAddress % 0x10000 == 0) {
              jtagFlashEraseBlock(rebasedAddress);
            }

            for (int j =0; j<size; ) {
              jtagFlashWriteBlock(rebasedAddress + j*4, 512, (uint32_t*)&src_addr[j]);
              j += 128;
            }
            goto end;
          }
#endif
          // Set automatic page write
          NVMCTRL->CTRLB.bit.MANW = 0;

          // Do writes in pages
          while (size)
          {
            // Execute "PBC" Page Buffer Clear
            NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
            while (NVMCTRL->INTFLAG.bit.READY == 0)
              ;

            // Fill page buffer
            uint32_t i;
            for (i=0; i<(PAGE_SIZE/4) && i<size; i++)
            {
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

end:

        // Notify command completed
        sam_ba_putdata( ptr_monitor_if, "Y\n\r", 3);
      }
      else if (command == 'Z') // Calculate CRC16
      {
        // This command calculate CRC for a given area of memory.
        // It's useful to quickly check if a transfer has been done
        // successfully.

        // Syntax: Z[START_ADDR],[SIZE]#
        // Returns: Z[CRC]#

        uint8_t *data;
        uint32_t size = current_number;
        uint16_t crc = 0;
        uint32_t i = 0;

#ifdef ENABLE_JTAG_LOAD
        uint8_t buf[4096];
#endif

#ifdef ENABLE_JTAG_LOAD
        if ((uint32_t)ptr_data == 0x40000) {
          data = (uint8_t*)buf;
          for (int j =0; j<size; ) {
            jtagFlashReadBlock(SCRATCHPAD_FLASH_PAGE + j, 256, &data[j]);
            j += 256;
          }
        } else if ((uint32_t)ptr_data >= 0x41000) {
          data = (uint8_t*)buf;
          for (int j =0; j<size; ) {
            jtagFlashReadBlock((uint32_t)ptr_data + offset - 0x41000 + j, 256, &data[j]);
            j += 256;
          }
        } else {
          data = (uint8_t *)ptr_data;
        }
#else
        data = (uint8_t *)ptr_data;
#endif

        for (i=0; i<size; i++)
          crc = serial_add_crc(*data++, crc);

        // Send response
        sam_ba_putdata( ptr_monitor_if, "Z", 1);
        put_uint32(crc);
        sam_ba_putdata( ptr_monitor_if, "#\n\r", 3);
      }

      command = 'z';
      current_number = 0;

      if (b_terminal_mode)
      {
        sam_ba_putdata( ptr_monitor_if, ">", 1);
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

void sam_ba_monitor_sys_tick(void)
{
	/* Check whether the TX or RX LED one-shot period has elapsed.  if so, turn off the LED */
	if (txLEDPulse && !(--txLEDPulse))
		LEDTX_off();
	if (rxLEDPulse && !(--rxLEDPulse))
		LEDRX_off();
}

/**
 * \brief This function starts the SAM-BA monitor.
 */
void sam_ba_monitor_run(void)
{
  uint32_t pageSizes[] = { 8, 16, 32, 64, 128, 256, 512, 1024 };
  PAGE_SIZE = pageSizes[NVMCTRL->PARAM.bit.PSZ];
  PAGES = NVMCTRL->PARAM.bit.NVMP;
  MAX_FLASH = PAGE_SIZE * PAGES;

#ifdef SECURE_BY_DEFAULT
  b_security_enabled = true;
#else
  b_security_enabled = NVMCTRL->STATUS.bit.SB != 0;
#endif

  ptr_data = NULL;
  command = 'z';
  while (1)
  {
    sam_ba_monitor_loop();
  }
}
