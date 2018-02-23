/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

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

#include <Arduino.h>
#include "Reset.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NVM_MEMORY ((volatile uint16_t *)0x000000)

/* The definitions here need the MattairTech SAMD core >=1.6.8.
 * The format is different than the stock Arduino SAMD core,
 * which uses ARDUINO_SAMD_VARIANT_COMPLIANCE instead.
 */
#if (MATTAIRTECH_ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10608)

extern const uint32_t __text_start__;
#define APP_START ((volatile uint32_t)(&__text_start__) + 4)

#else
#if defined(__NO_BOOTLOADER__)
	#define APP_START 0x00000004UL
#elif defined(__4KB_BOOTLOADER__)
	#define APP_START 0x00001004UL
#elif defined(__8KB_BOOTLOADER__)
	#define APP_START 0x00002004UL
#elif defined(__16KB_BOOTLOADER__)
	#define APP_START 0x00004004UL
#else
	#error "Reset.cpp: You must define bootloader size in boards.txt build.extra_flags (ie: -D__8KB_BOOTLOADER__)"
#endif
#endif

static inline bool nvmReady(void) {
#if (SAMD51)
  return NVMCTRL->STATUS.reg & NVMCTRL_STATUS_READY;
#else
  return NVMCTRL->INTFLAG.reg & NVMCTRL_INTFLAG_READY;
#endif
}

__attribute__ ((long_call, section (".ramfunc")))
void banzai() {
	// Disable all interrupts
	__disable_irq();

	// Avoid erasing the application if APP_START is < than the minimum bootloader size
	// This could happen if without_bootloader linker script was chosen
	// Minimum bootloader size in SAMD21 family is 512bytes (RM section 22.6.5)
	if (APP_START < (0x200 + 4)) {
		goto reset;
        }

        // Disable NVM cache on D51 (errata). Will be re-enabled after reset.
        #if (SAMD51)
          NVMCTRL->CTRLA.reg = (NVMCTRL_CTRLA_CACHEDIS0 | NVMCTRL_CTRLA_CACHEDIS1);
        #endif

	// Erase application
	while (!nvmReady())
          ;

        // Note: the flash memory is erased in 4 page ROWS (16 page blocks for D51).
        //       Even if the starting address is the last byte of a ROW/block the
        //       entire ROW/block is erased anyway.
        #if (SAMD51)
          NVMCTRL->ADDR.reg  = (uint32_t)APP_START;   // 8-bit hardware address
          NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMD_EB | NVMCTRL_CTRLB_CMDEX_KEY;
        #else
          NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;
          NVMCTRL->ADDR.reg  = (uintptr_t)&NVM_MEMORY[APP_START / 4];   // 16-bit hardware address
          NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_ER | NVMCTRL_CTRLA_CMDEX_KEY;
        #endif

	while (!nvmReady())
		;

reset:
	// Reset the device
	NVIC_SystemReset() ;

	while (true);
}

static int ticks = -1;

void initiateReset(int _ticks) {
	ticks = _ticks;
}

void cancelReset() {
	ticks = -1;
}

void tickReset() {
	if (ticks == -1)
		return;
	ticks--;
	if (ticks == 0)
		banzai();
}

#ifdef __cplusplus
}
#endif
