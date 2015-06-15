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
#define APP_START 0x00002004

static inline bool nvmReady(void) {
        return NVMCTRL->INTFLAG.reg & NVMCTRL_INTFLAG_READY;
}

__attribute__ ((long_call, section (".ramfunc")))
static void banzai() {
	// Disable all interrupts
	__disable_irq();

	// Erase application
	while (!nvmReady())
		;
	NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;
	NVMCTRL->ADDR.reg  = (uintptr_t)&NVM_MEMORY[APP_START / 4];
	NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_ER | NVMCTRL_CTRLA_CMDEX_KEY;
	while (!nvmReady())
		;

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
