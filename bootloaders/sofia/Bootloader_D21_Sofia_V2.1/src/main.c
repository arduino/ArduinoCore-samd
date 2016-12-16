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

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */

#include <asf.h>
#include <usart_setup.h>

static inline bool nvm_is_ready(void)
{
	/* Get a pointer to the module hardware instance */
	Nvmctrl *const nvm_module = NVMCTRL;

	return nvm_module->INTFLAG.reg & NVMCTRL_INTFLAG_READY;
}
#include <stk500v2.h>
enum nvm_cache_readmode {
	/** The NVM Controller (cache system) does not insert wait states on
	 *  a cache miss. Gives the best system performance.
	 */
	NVM_CACHE_READMODE_NO_MISS_PENALTY,
	/** Reduces power consumption of the cache system, but inserts a
	 *  wait state each time there is a cache miss
	 */
	NVM_CACHE_READMODE_LOW_POWER,
	/** The cache system ensures that a cache hit or miss takes the same
	 *  amount of time, determined by the number of programmed flash
	 *  wait states.
	 */
	NVM_CACHE_READMODE_DETERMINISTIC,
};

enum nvm_sleep_power_mode {
	/** NVM controller exits low power mode on first access after sleep. */
	NVM_SLEEP_POWER_MODE_WAKEONACCESS  = NVMCTRL_CTRLB_SLEEPPRM_WAKEONACCESS_Val,
	/** NVM controller exits low power mode when the device exits sleep mode. */
	NVM_SLEEP_POWER_MODE_WAKEUPINSTANT = NVMCTRL_CTRLB_SLEEPPRM_WAKEUPINSTANT_Val,
	/** Power reduction mode in the NVM controller disabled. */
	NVM_SLEEP_POWER_MODE_ALWAYS_AWAKE  = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val,
};


void configure_nvm(void)
{
	/* Turn on the digital interface clock */
	system_apb_clock_set_mask(SYSTEM_CLOCK_APB_APBB, PM_APBBMASK_NVMCTRL);

	/* Clear error flags */
	NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;

	/* Check if the module is busy */
	if (!nvm_is_ready()) {
		return STATUS_BUSY;
	}

	/* Writing configuration to the CTRLB register */
	NVMCTRL->CTRLB.reg =
		NVMCTRL_CTRLB_SLEEPPRM(NVM_SLEEP_POWER_MODE_WAKEONACCESS) |
		((false & 0x01) << NVMCTRL_CTRLB_MANW_Pos) |
		NVMCTRL_CTRLB_RWS(NVMCTRL->CTRLB.bit.RWS) |
		((false & 0x01) << NVMCTRL_CTRLB_CACHEDIS_Pos) |
		NVMCTRL_CTRLB_READMODE(NVM_CACHE_READMODE_NO_MISS_PENALTY);
}

/**
 * \brief Function to start the application.
 */
static void start_application(void)
{
	/* Pointer to the Application Section */
	void (*application_code_entry)(void);

	/* Rebase the Stack Pointer */
	__set_MSP(*(uint32_t *)APP_START_ADDR);

	/* Rebase the vector table base address */
	SCB->VTOR = ((uint32_t)APP_START_ADDR & SCB_VTOR_TBLOFF_Msk);

	/* Load the Reset Handler address of the application */
	application_code_entry = (void (*)(void))(unsigned *)(*(unsigned *)
			(APP_START_ADDR + 4));

	/* Jump to user Reset Handler in the application */
	application_code_entry();
}

struct rtc_module rtc_instance;

void rtc_overflow_callback(void)
{
	timeout+=2;//callback is called every 2ms
	if(timeout > TIMOUT_PERIOD)
	{
		rtc_count_disable(&rtc_instance);
		rtc_count_clear_overflow(&rtc_instance);
		NVIC_ClearPendingIRQ(RTC_IRQn);
		udc_stop();
		cpu_irq_disable();
		//SCB->AIRCR = ((0x05FA<<16)|0b100); //reset cpu
		NVIC_SystemReset() ;
		while(1)
		{
			//wait for WDT
		}
	}
	if((timeout % 50) == 0)
	{
		port_pin_toggle_output_level(LED_L_PIN);		// when the board is in bootloader mode the L Led blink
	}
}
void configure_timeout()
{
	struct rtc_count_config config_rtc_count;
	rtc_count_get_config_defaults(&config_rtc_count);
	config_rtc_count.prescaler = RTC_COUNT_PRESCALER_DIV_1;
	config_rtc_count.mode = RTC_COUNT_MODE_16BIT;
	config_rtc_count.continuously_update = true;
	rtc_count_init(&rtc_instance, RTC, &config_rtc_count);
	rtc_count_enable(&rtc_instance);	rtc_count_register_callback(
	&rtc_instance, rtc_overflow_callback, RTC_COUNT_CALLBACK_OVERFLOW);
	rtc_count_enable_callback(&rtc_instance, RTC_COUNT_CALLBACK_OVERFLOW);	rtc_count_set_period(&rtc_instance, 1);//1 ms per callback	NVIC_SetPriority(RTC_IRQn, 0);//RTC Top priority
}


void configure_port_pins(void)
{
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction  = PORT_PIN_DIR_INPUT;
	config_port_pin.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(BUTTON_0_PIN, &config_port_pin);
}


//1 =16 kb
#define BOOTLOADER_SIZE 1

void protect_boot_section()
{
	system_interrupt_enter_critical_section();
	volatile uint32_t raw_fusebits[2];

	/* Make sure the module is ready */
	while (!nvm_is_ready()) {
	};

	/* Read the fuse settings in the user row, 64 bit */
	((uint16_t*)&raw_fusebits)[0] = (uint16_t)NVM_MEMORY[NVMCTRL_USER / 2];
	((uint16_t*)&raw_fusebits)[1] = (uint16_t)NVM_MEMORY[(NVMCTRL_USER / 2) + 1];
	((uint16_t*)&raw_fusebits)[2] = (uint16_t)NVM_MEMORY[(NVMCTRL_USER / 2) + 2];
	((uint16_t*)&raw_fusebits)[3] = (uint16_t)NVM_MEMORY[(NVMCTRL_USER / 2) + 3];
	if(((raw_fusebits[0] & NVMCTRL_FUSES_BOOTPROT_Msk)>> NVMCTRL_FUSES_BOOTPROT_Pos) != BOOTLOADER_SIZE)
	{
		/* Auxiliary space cannot be accessed if the security bit is set */
		if (NVMCTRL->STATUS.reg & NVMCTRL_STATUS_SB) {
			return;
		}	
		 /* Disable Cache */
		 uint32_t temp = NVMCTRL->CTRLB.reg;
	 
		 NVMCTRL->CTRLB.reg = temp | NVMCTRL_CTRLB_CACHEDIS;
	 
		 /* Clear error flags */
		 NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;

		 /* Set address, command will be issued elsewhere */
		 NVMCTRL->ADDR.reg = NVMCTRL_AUX0_ADDRESS/2;
	 
		 /* Erase the user page */
		 NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_EAR | NVMCTRL_CTRLA_CMDEX_KEY;
	 
		 /* Wait for NVM command to complete */
		 while (!(NVMCTRL->INTFLAG.reg & NVMCTRL_INTFLAG_READY));
	 
		 /* Clear error flags */
		 NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;
	 
		 /* Set address, command will be issued elsewhere */
		 NVMCTRL->ADDR.reg = NVMCTRL_AUX0_ADDRESS/2;
	 
		 /* Erase the page buffer before buffering new data */
		 NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_PBC | NVMCTRL_CTRLA_CMDEX_KEY;

		 /* Wait for NVM command to complete */
		 while (!(NVMCTRL->INTFLAG.reg & NVMCTRL_INTFLAG_READY));
	 
		 /* Clear error flags */
		 NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;
	 
		 /* Set address, command will be issued elsewhere */
		 NVMCTRL->ADDR.reg = NVMCTRL_AUX0_ADDRESS/2;
		raw_fusebits[0] = ((raw_fusebits[0] ^ NVMCTRL_FUSES_BOOTPROT_Msk) | NVMCTRL_FUSES_BOOTPROT(BOOTLOADER_SIZE));
		 *((uint32_t *)NVMCTRL_AUX0_ADDRESS) = raw_fusebits[0];
		 *(((uint32_t *)NVMCTRL_AUX0_ADDRESS) + 1) = raw_fusebits[1];
	 
		 /* Write the user page */
		 NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_WAP | NVMCTRL_CTRLA_CMDEX_KEY;
	 
		 /* Restore the settings */
		 NVMCTRL->CTRLB.reg = temp;


		//nvm_execute_command(NVM_COMMAND_WRITE_AUX_ROW, NVMCTRL_USER, 0);
	}
	system_interrupt_leave_critical_section();
}

int main (void)
{
	
	/* Check if WDT is locked */
	
	
	if (!(WDT->CTRL.reg & WDT_CTRL_ALWAYSON)) {
		/* Disable the Watchdog module */
		WDT->CTRL.reg &= ~WDT_CTRL_ENABLE;
	}
	if(!(PM->RCAUSE.reg & PM_RCAUSE_EXT) && !(PM->RCAUSE.reg & PM_RCAUSE_WDT) && (NVM_MEMORY[APP_START_ADDR / 2] != 0xFFFF))		//Power on reset or systemResetReq -> run main app
	{
		start_application();
	}
	//if reset by wdt or reset button, run bootloader
	irq_initialize_vectors();
	cpu_irq_enable();

	system_init();

	configure_timeout();
	configure_nvm();
	
	// start usart
	configure_usart();
	// Start USB stack to authorize VBus monitoring
	udc_start();
	NVIC_SetPriority(USB_IRQn, 1);		//USB Should have lower priority than rtc

	//check if boot-protection is on
	//(edbg does not write to boot-section if this is protected
	//(bootprot can be manually changed to 0x07 in the "fuses" tab of Atmel tudio to reprogram)
	
	protect_boot_section();												//uncomment for release
	
	// set board LED initial status
	
	port_pin_set_output_level(LED_L_PIN,LED_L_INACTIVE);
	port_pin_set_output_level(LED_TX_PIN,LED_TX_INACTIVE);
	port_pin_set_output_level(LED_RX_PIN,LED_RX_INACTIVE);

	while (1) {
		get_message();//STK500v2
	}
}

void main_cdc_config(uint8_t port,usb_cdc_line_coding_t *cfg)
{
	if(cfg[port].dwDTERate == 1200)
	{
		//do nothing
	}
}