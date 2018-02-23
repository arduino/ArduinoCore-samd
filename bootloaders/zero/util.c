/*
 * Copyright (c) 2015 Arduino LLC.  All right reserved.
 * Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.
 * Copyright (c) 2017 MattairTech LLC. All right reserved.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "util.h"

extern uint32_t __sketch_vectors_ptr; // Exported value from linker script

/*
 * The SAMD21 has a default 1MHz clock @ reset.
 * SAML21 and SAMC21 have a default 4MHz clock @ reset.
 * SAMD51 has a default 48MHz clock @ reset.
 * It is switched to 48MHz (optionally to 120MHz for SAMD51) in board_init.c
 */
#if (SAMD21 || SAMD11)
  #define CLOCK_DEFAULT 1000000ul
#elif (SAML21 || SAMC21)
  #define CLOCK_DEFAULT 4000000ul
#elif (SAMD51)
  #define CLOCK_DEFAULT 48000000ul
#endif
uint32_t SystemCoreClock=CLOCK_DEFAULT;

void flashErase (uint32_t startAddress)
{
  // Syntax: X[ADDR]#
  // Erase the flash memory starting from ADDR to the end of flash.

  // Note: the flash memory is erased in 4 page ROWS (16 page blocks for D51).
  //       Even if the starting address is the last byte of a ROW/block the
  //       entire ROW/block is erased anyway.

  uint32_t dst_addr = startAddress; // starting address

  while (dst_addr < FLASH_SIZE)
  {
#if (SAMD51)
    // Execute "EB" Erase Block
    NVMCTRL->ADDR.reg = dst_addr;   // 8-bit hardware address
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_EB;
    while (NVMCTRL->STATUS.bit.READY == 0)
      ;
    dst_addr += FLASH_PAGE_SIZE * 16; // Skip a Block
#else
    // Execute "ER" Erase Row
    NVMCTRL->ADDR.reg = dst_addr / 2;   // 16-bit hardware address
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
    while (NVMCTRL->INTFLAG.bit.READY == 0)
      ;
    dst_addr += FLASH_PAGE_SIZE * 4; // Skip a ROW
#endif
  }
}

void flashWrite (uint32_t numBytes, uint32_t * buffer, uint32_t * ptr_data)
{
  // This command writes the content of a buffer in SRAM into flash memory.

  // Syntax: Y[ADDR],0#
  // Set the starting address of the SRAM buffer.

  // Syntax: Y[ROM_ADDR],[SIZE]#
  // Write the first SIZE bytes from the SRAM buffer (previously set) into
  // flash memory starting from address ROM_ADDR

  // Write to flash
  uint32_t size = numBytes/4;
  uint32_t *src_addr = buffer;
  uint32_t *dst_addr = ptr_data;

  // Do writes in pages
  while (size)
  {
    // Execute "PBC" Page Buffer Clear
#if (SAMD51)
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_PBC;
    while (NVMCTRL->STATUS.bit.READY == 0)
      ;
#else
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
    while (NVMCTRL->INTFLAG.bit.READY == 0)
      ;
#endif

    // Fill page buffer
    uint32_t i;
    for (i=0; i<(FLASH_PAGE_SIZE/4) && i<size; i++)
    {
      dst_addr[i] = src_addr[i];
    }

    // Execute "WP" Write Page
#if (SAMD51)
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_WP;
    while (NVMCTRL->STATUS.bit.READY == 0)
      ;
#else
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
    while (NVMCTRL->INTFLAG.bit.READY == 0)
      ;
#endif

    // Advance to next page
    dst_addr += i;
    src_addr += i;
    size     -= i;
  }
}

void pinMux (uint32_t pinmux)
{
  uint32_t port;
  uint32_t pin;

  if (pinmux != PINMUX_UNUSED)
  {
    /* Mask 6th bit in pin number to check whether it is greater than 32 i.e., PORTB pin */
    port = (pinmux & 0x200000) >> 21;
    pin = (pinmux >> 16);
    PORT->Group[port].PINCFG[(pin - (port*32))].bit.PMUXEN = 1;
    PORT->Group[port].PMUX[(pin - (port*32))/2].reg &= ~(0xF << (4 * (pin & 0x01u)));
    PORT->Group[port].PMUX[(pin - (port*32))/2].reg |= (pinmux & 0xFF) << (4 * (pin & 0x01u));
  }
}

void pinConfig (uint8_t port, uint8_t pin, uint8_t config)
{
  uint8_t pinCfg = PORT_PINCFG_INEN;

  if (config == INPUT_PULLUP || config == INPUT_PULLDOWN  || config == INPUT) {
    PORT->Group[port].DIRCLR.reg = (1 << pin);
    if (config == INPUT_PULLUP) {
      pinCfg |= PORT_PINCFG_PULLEN;
      PORT->Group[port].OUTSET.reg = (1 << pin);
    } else if (config == INPUT_PULLDOWN) {
      pinCfg |= PORT_PINCFG_PULLEN;
      PORT->Group[port].OUTCLR.reg = (1 << pin);
    }
  } else {
    if (config == OUTPUT_HIGH) {
      PORT->Group[port].OUTSET.reg = (1 << pin);
    } else if (config == OUTPUT_LOW) {
      PORT->Group[port].OUTCLR.reg = (1 << pin);
    }
    PORT->Group[port].DIRSET.reg = (1 << pin);
  }

  PORT->Group[port].PINCFG[pin].reg = (uint8_t)pinCfg;
}

bool isPinActive (uint8_t port, uint8_t pin, uint8_t config)
{
  uint32_t pinState = ((PORT->Group[port].IN.reg) & (1 << pin));
  if (config == PIN_POLARITY_ACTIVE_LOW) {
    if (!pinState) return(true);
  } else {
    if (pinState) return(true);
  }
  return(false);
}

void delayUs (uint32_t delay)
{
  /* The SAMD21 has a default 1MHz clock @ reset.
   * SAML21 and SAMC21 have a default 4MHz clock @ reset.
   * SAMD51 has a default 48MHz clock @ reset.
   * It is switched to 48MHz (optionally to 120MHz for SAMD51) in board_init.c.
   * Note that the D51 CMCC cache is not enabled, and the NVM caches are disabled
   * in the bootloader due to errata, so FLASH wait states apply.
   */
  uint32_t numLoops;

  if (SystemCoreClock == VARIANT_MCK) {
#if (SAMD51)
  #if (VARIANT_MCK == 120000000ul)
    numLoops = (12 * delay);
  #else
    numLoops = (9 * delay);
  #endif
#else
    numLoops = (12 * delay);
#endif
  } else {
#if (SAMD21 || SAMD11)
    numLoops = (delay >> 2);
#elif (SAML21 || SAMC21)
    numLoops = delay;
#elif (SAMD51)
    numLoops = (9 * delay);
#endif
  }

  for (uint32_t i=0; i < numLoops; i++)
    /* force compiler to not optimize this... */
    __asm__ __volatile__("");
}

void systemReset (void)
{
  /* Request a system reset */
  NVIC_SystemReset();
}

void waitForSync (void)
{
  #if (SAMD21 || SAMD11)
  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );
  #elif (SAML21 || SAMC21 || SAMD51)
  while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK );
  #endif
}
