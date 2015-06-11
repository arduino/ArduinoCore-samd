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

/**
 * --------------------
 * SAM-BA Implementation on SAMD21
 * --------------------
 * Requirements to use SAM-BA :
 *
 * Supported communication interfaces :
 * --------------------
 *
 * SERCOM5 : RX:PB23 TX:PB22
 * Baudrate : 115200 8N1
 *
 * USB : D-:PA24 D+:PA25
 *
 * Pins Usage
 * --------------------
 * The following pins are used by the program :
 * PA25 : input/output
 * PA24 : input/output
 * PB23 : input
 * PB22 : output
 * PA15 : input
 *
 * The application board shall avoid driving the PA25,PA24,PB23,PB22 and PA15 signals
 * while the boot program is running (after a POR for example)
 *
 * Clock system
 * --------------------
 * CPU clock source (GCLK_GEN_0) - 8MHz internal oscillator (OSC8M)
 * SERCOM5 core GCLK source (GCLK_ID_SERCOM5_CORE) - GCLK_GEN_0 (i.e., OSC8M)
 * GCLK Generator 1 source (GCLK_GEN_1) - 48MHz DFLL in Clock Recovery mode (DFLL48M)
 * USB GCLK source (GCLK_ID_USB) - GCLK_GEN_1 (i.e., DFLL in CRM mode)
 *
 * Memory Mapping
 * --------------------
 * SAM-BA code will be located at 0x0 and executed before any applicative code.
 *
 * Applications compiled to be executed along with the bootloader will start at
 * 0x2000
 * Before jumping to the application, the bootloader changes the VTOR register
 * to use the interrupt vectors of the application @0x2000.<- not required as
 * application code is taking care of this
 *
*/

#include <stdio.h>
#include <sam.h>
#include "compiler.h"
#include "sam_ba_monitor.h"
#include "usart_sam_ba.h"
#include "main.h"
#include "cdc_enumerate.h"

#define NVM_SW_CALIB_DFLL48M_COARSE_VAL   58
#define NVM_SW_CALIB_DFLL48M_FINE_VAL     64

static void check_start_application(void);

static volatile bool main_b_cdc_enable = false;

/**
 * \brief Check the application startup condition
 *
 */
static void check_start_application(void)
{
	volatile PortGroup *led_port = (volatile PortGroup *)&PORT->Group[1];
	led_port->DIRSET.reg = (1<<30);
	led_port->OUTCLR.reg = (1<<30);

#if defined(BOOT_DOUBLE_TAP_ADDRESS)
	#define DOUBLE_TAP_MAGIC 0x07738135
	if (PM->RCAUSE.bit.POR)
	{
		/* On power-on initialize double-tap */
		BOOT_DOUBLE_TAP_DATA = 0;
	}
	else
	{
		if (BOOT_DOUBLE_TAP_DATA == DOUBLE_TAP_MAGIC) {
			/* Second tap, stay in bootloader */
			BOOT_DOUBLE_TAP_DATA = 0;
			return;
		}

		/* First tap */
		BOOT_DOUBLE_TAP_DATA = DOUBLE_TAP_MAGIC;

		/* Wait 0.5sec to see if the user tap reset again */
		for (uint32_t i=0; i<125000; i++) /* 500ms */
			/* force compiler to not optimize this... */
			__asm__ __volatile__("");

		/* Timeout happened, continue boot... */
		BOOT_DOUBLE_TAP_DATA = 0;
	}
#endif

	uint32_t app_start_address;

	/* Load the Reset Handler address of the application */
	app_start_address = *(uint32_t *)(APP_START_ADDRESS + 4);

	/**
	 * Test reset vector of application @APP_START_ADDRESS+4
	 * Stay in SAM-BA if *(APP_START+0x4) == 0xFFFFFFFF
	 * Application erased condition
	 */
	if (app_start_address == 0xFFFFFFFF) {
		/* Stay in bootloader */
		return;
	}

#if defined(BOOT_LOAD_PIN)
	volatile PortGroup *boot_port = (volatile PortGroup *)(&(PORT->Group[BOOT_LOAD_PIN / 32]));
	volatile bool boot_en;

	/* Enable the input mode in Boot GPIO Pin */
	boot_port->DIRCLR.reg = BOOT_PIN_MASK;
	boot_port->PINCFG[BOOT_LOAD_PIN & 0x1F].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	boot_port->OUTSET.reg = BOOT_PIN_MASK;
	/* Read the BOOT_LOAD_PIN status */
	boot_en = (boot_port->IN.reg) & BOOT_PIN_MASK;

	/* Check the bootloader enable condition */
	if (!boot_en) {
		/* Stay in bootloader */
		return;
	}
#endif

	led_port->OUTSET.reg = (1<<30);

	/* Rebase the Stack Pointer */
	__set_MSP(*(uint32_t *) APP_START_ADDRESS);

	/* Rebase the vector table base address */
	SCB->VTOR = ((uint32_t) APP_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);

	/* Jump to application Reset Handler in the application */
	asm("bx %0"::"r"(app_start_address));
}

void system_init()
{
	/* Configure flash wait states */
	NVMCTRL->CTRLB.bit.RWS = FLASH_WAIT_STATES;

	/* Set OSC8M prescalar to divide by 1 */
	SYSCTRL->OSC8M.bit.PRESC = 0;

	/* Configure OSC8M as source for GCLK_GEN0 */
	GCLK_GENCTRL_Type genctrl={0};
	uint32_t temp_genctrl;
	GCLK->GENCTRL.bit.ID = 0; /* GENERATOR_ID - GCLK_GEN_0 */
	while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
	temp_genctrl = GCLK->GENCTRL.reg;
	genctrl.bit.SRC = GCLK_GENCTRL_SRC_OSC8M_Val;
	genctrl.bit.GENEN = true;
	genctrl.bit.RUNSTDBY = false;
	GCLK->GENCTRL.reg = (genctrl.reg | temp_genctrl);
	while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

#if SAM_BA_INTERFACE == SAM_BA_USBCDC_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
	SYSCTRL_DFLLCTRL_Type dfllctrl_conf = {0};
	SYSCTRL_DFLLVAL_Type dfllval_conf = {0};
	uint32_t coarse =( *((uint32_t *)(NVMCTRL_OTP4)
                    + (NVM_SW_CALIB_DFLL48M_COARSE_VAL / 32))
                    >> (NVM_SW_CALIB_DFLL48M_COARSE_VAL % 32))
                    & ((1 << 6) - 1);
	if (coarse == 0x3f) {
		coarse = 0x1f;
	}
	uint32_t fine =( *((uint32_t *)(NVMCTRL_OTP4)
                  + (NVM_SW_CALIB_DFLL48M_FINE_VAL / 32))
                  >> (NVM_SW_CALIB_DFLL48M_FINE_VAL % 32))
                  & ((1 << 10) - 1);
	if (fine == 0x3ff) {
		fine = 0x1ff;
	}
	dfllval_conf.bit.COARSE  = coarse;
	dfllval_conf.bit.FINE    = fine;
	dfllctrl_conf.bit.USBCRM = true;
	dfllctrl_conf.bit.BPLCKC = false;
	dfllctrl_conf.bit.QLDIS  = false;
	dfllctrl_conf.bit.CCDIS  = true;
	dfllctrl_conf.bit.ENABLE = true;

	SYSCTRL->DFLLCTRL.bit.ONDEMAND = false;
	while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));
	SYSCTRL->DFLLMUL.reg = 48000;
	SYSCTRL->DFLLVAL.reg = dfllval_conf.reg;
	SYSCTRL->DFLLCTRL.reg = dfllctrl_conf.reg;

	GCLK_CLKCTRL_Type clkctrl={0};
	uint16_t temp;
	GCLK->CLKCTRL.bit.ID = 0; /* GCLK_ID - DFLL48M Reference */
	temp = GCLK->CLKCTRL.reg;
	clkctrl.bit.CLKEN = true;
	clkctrl.bit.WRTLOCK = false;
	clkctrl.bit.GEN = GCLK_CLKCTRL_GEN_GCLK0_Val;
	GCLK->CLKCTRL.reg = (clkctrl.reg | temp);

	/* Configure DFLL48M as source for GCLK_GEN1 */
	GCLK->GENCTRL.bit.ID = 1; /* GENERATOR_ID - GCLK_GEN_1 */
	while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
	temp_genctrl = GCLK->GENCTRL.reg;
	genctrl.bit.SRC = GCLK_GENCTRL_SRC_DFLL48M_Val;
	genctrl.bit.GENEN = true;
	genctrl.bit.RUNSTDBY = false;
	GCLK->GENCTRL.reg = (genctrl.reg | temp_genctrl);
	while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
#endif
}


#if DEBUG_ENABLE
#	define DEBUG_PIN_HIGH 	port_pin_set_output_level(BOOT_LED, 1)
#	define DEBUG_PIN_LOW 	port_pin_set_output_level(BOOT_LED, 0)
#else
#	define DEBUG_PIN_HIGH 	do{}while(0)
#	define DEBUG_PIN_LOW 	do{}while(0)
#endif


/**
 *  \brief SAMD21 SAM-BA Main loop.
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
#if SAM_BA_INTERFACE == SAM_BA_USBCDC_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES 
	P_USB_CDC pCdc;
#endif
	DEBUG_PIN_HIGH;

	/* Jump in application if condition is satisfied */
	check_start_application();

	/* We have determined we should stay in the monitor. */
	/* System initialization */
	system_init();
	cpu_irq_enable();

	#if SAM_BA_INTERFACE == SAM_BA_UART_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
	/* UART is enabled in all cases */
	usart_open();
#endif

#if SAM_BA_INTERFACE == SAM_BA_USBCDC_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
	pCdc = (P_USB_CDC)usb_init();
#endif
	DEBUG_PIN_LOW;
	/* Wait for a complete enum on usb or a '#' char on serial line */
	while (1) {
#if SAM_BA_INTERFACE == SAM_BA_USBCDC_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
		if (pCdc->IsConfigured(pCdc) != 0) {
			main_b_cdc_enable = true;
		}

		//Check if a USB enumeration has succeeded
		//And com port was opened
		if (main_b_cdc_enable) {
			sam_ba_monitor_init(SAM_BA_INTERFACE_USBCDC);
			//SAM-BA on USB loop
			while(1) {
				sam_ba_monitor_run();
			}
		}
#endif
#if SAM_BA_INTERFACE == SAM_BA_UART_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
		/* Check if a '#' has been received */
		if (!main_b_cdc_enable && usart_sharp_received()) {
			sam_ba_monitor_init(SAM_BA_INTERFACE_USART);
			/* SAM-BA on UART loop */
			while(1) {
				sam_ba_monitor_run();
			}
		}
#endif
	}
}
