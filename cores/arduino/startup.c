/*
  Arduino.h - Main include file for the Arduino SDK
  Copyright (c) 2014 Arduino Team.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "sam.h"

/* Initialize segments */
extern uint32_t __etext ;
extern uint32_t __data_start__ ;
extern uint32_t __data_end__ ;
extern uint32_t __bss_start__ ;
extern uint32_t __bss_end__ ;
extern uint32_t __StackTop;

extern int main( void ) ;

//void __libc_init_array(void);

/* Default empty handler */
void Dummy_Handler(void);

/* Cortex-M0+ core handlers */
void NMI_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void HardFault_Handler       ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SVC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PendSV_Handler          ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SysTick_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));

/* Peripherals handlers */
void PM_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SYSCTRL_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void WDT_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RTC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void NVMCTRL_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DMAC_Handler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void USB_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EVSYS_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM0_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM1_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM2_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM3_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM4_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM5_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC0_Handler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC1_Handler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC2_Handler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC3_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC4_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC5_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC6_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC7_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void ADC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void AC_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DAC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PTC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void I2S_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));

/* Exception Table */
__attribute__ ((section(".vectors")))
const DeviceVectors exception_table=
{
  /* Configure Initial Stack Pointer, using linker-generated symbols */
  (void*) (&__StackTop),

  (void*) Reset_Handler,
  (void*) NMI_Handler,
  (void*) HardFault_Handler,
  (void*) (0UL), /* Reserved */
  (void*) (0UL), /* Reserved */
  (void*) (0UL), /* Reserved */
  (void*) (0UL), /* Reserved */
  (void*) (0UL), /* Reserved */
  (void*) (0UL), /* Reserved */
  (void*) (0UL), /* Reserved */
  (void*) SVC_Handler,
  (void*) (0UL), /* Reserved */
  (void*) (0UL), /* Reserved */
  (void*) PendSV_Handler,
  (void*) SysTick_Handler,

  /* Configurable interrupts */
  (void*) PM_Handler,             /*  0 Power Manager */
  (void*) SYSCTRL_Handler,        /*  1 System Control */
  (void*) WDT_Handler,            /*  2 Watchdog Timer */
  (void*) RTC_Handler,            /*  3 Real-Time Counter */
  (void*) EIC_Handler,            /*  4 External Interrupt Controller */
  (void*) NVMCTRL_Handler,        /*  5 Non-Volatile Memory Controller */
  (void*) DMAC_Handler,           /*  6 Direct Memory Access Controller */
  (void*) USB_Handler,            /*  7 Universal Serial Bus */
  (void*) EVSYS_Handler,          /*  8 Event System Interface */
  (void*) SERCOM0_Handler,        /*  9 Serial Communication Interface 0 */
  (void*) SERCOM1_Handler,        /* 10 Serial Communication Interface 1 */
  (void*) SERCOM2_Handler,        /* 11 Serial Communication Interface 2 */
  (void*) SERCOM3_Handler,        /* 12 Serial Communication Interface 3 */
  (void*) SERCOM4_Handler,        /* 13 Serial Communication Interface 4 */
  (void*) SERCOM5_Handler,        /* 14 Serial Communication Interface 5 */
  (void*) TCC0_Handler,           /* 15 Timer Counter Control 0 */
  (void*) TCC1_Handler,           /* 16 Timer Counter Control 1 */
  (void*) TCC2_Handler,           /* 17 Timer Counter Control 2 */
  (void*) TC3_Handler,            /* 18 Basic Timer Counter 0 */
  (void*) TC4_Handler,            /* 19 Basic Timer Counter 1 */
  (void*) TC5_Handler,            /* 20 Basic Timer Counter 2 */
  (void*) TC6_Handler,            /* 21 Basic Timer Counter 3 */
  (void*) TC7_Handler,            /* 22 Basic Timer Counter 4 */
  (void*) ADC_Handler,            /* 23 Analog Digital Converter */
  (void*) AC_Handler,             /* 24 Analog Comparators */
  (void*) DAC_Handler,            /* 25 Digital Analog Converter */
  (void*) PTC_Handler,            /* 26 Peripheral Touch Controller */
  (void*) I2S_Handler             /* 27 Inter-IC Sound Interface */
} ;

/**
 * \brief This is the code that gets called on processor reset.
 * It configures the needed clocks and according Flash Read Wait States.
 * At reset:
 * - OSC8M clock source is enabled with a divider by 8 (1MHz).
 * - Generic Clock Generator 0 (GCLKMAIN) is using OSC8M as source.
 * We need to:
 * 1) Enable XOSC32K clock (External on-board 32.768Hz oscillator), will be used as DFLL48M reference.
 * 2) Put XOSC32K as source of Generic Clock Generator 1
 * 3) Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference)
 * 4) Enable DFLL48M clock
 * 5) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
 * 6) Put OSC8M as source for Generic Clock Generator 3
 */
void SystemInit( void )
{
  uint32_t ul_Coarse_calibration_value ;
  uint32_t ul_Fine_calibration_value ;
#define DFLL_CALIB_COARSE       ((uint32_t*)(NVMCTRL_OTP4+4))
#define DFLL_CALIB_COARSE_MSK   (0x3FUL)
#define DFLL_CALIB_COARSE_POS   (26UL)
#define DFLL_CALIB_FINE         ((uint32_t*)(NVMCTRL_OTP4+8))
#define DFLL_CALIB_FINE_MSK     (0x3FFUL)

  /* Set 1 Flash Wait State for 48MHz, cf tables 20.9 and 35.27 in SAMD21 Datasheet */
  NVMCTRL->CTRLB.bit.RWS = NVMCTRL_CTRLB_RWS_HALF_Val ;

  /* Turn on the digital interface clock */
  PM->APBAMASK.reg |= PM_APBAMASK_GCLK ;

  /*
   * 1) Enable XOSC32K clock (External on-board 32.768Hz oscillator)
   */
  SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_STARTUP( 0x6u ) | /* cf table 15.10 of product datasheet */
                         SYSCTRL_XOSC32K_XTALEN | SYSCTRL_XOSC32K_EN32K | SYSCTRL_XOSC32K_ENABLE ;
  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_XOSC32KRDY) == 0 )
  {
    /* Wait for oscillator stabilization */
  }

  /* Software reset the module to ensure it is re-initialized correctly */
  GCLK->CTRL.reg = GCLK_CTRL_SWRST ;
  while ( GCLK->CTRL.reg & GCLK_CTRL_SWRST )
  {
    /* Wait for reset to complete */
  }

  /*
   * 2) Put XOSC32K as source of Generic Clock Generator 1
   */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID( 1 ) | // Generic Clock Generator 1
                     GCLK_GENDIV_DIV( 1 ) | // Divider set to 1
                     GCLK_GENCTRL_DIVSEL ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* Write Generic Clock Generator 1 configuration */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 1 ) | // Generic Clock Generator 1
                      GCLK_GENCTRL_SRC_XOSC32K | // Selected source is External 32KHz Oscillator
                      GCLK_GENCTRL_GENEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /*
   * 3) Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference)
   */
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( 0 ) | // Generic Clock 0 (GCLKMAIN)
                      GCLK_CLKCTRL_GEN_GCLK1_Val | // Generic Clock Generator 1 is source
                      GCLK_CLKCTRL_CLKEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

 /*
  * 4) Enable DFLL48M clock
  */

  /* DFLL Configuration in Closed Loop mode, cf product datasheet chapter 15.6.7.1 - Closed-Loop Operation */

	/* Enable the generic clock */
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( SYSCTRL_GCLK_ID_DFLL48 ) | GCLK_CLKCTRL_CLKEN ;

  /* DFLL Enable (Closed Loop) */
  ul_Coarse_calibration_value = (DFLL_CALIB_COARSE >> DFLL_CALIB_COARSE_POS) & DFLL_CALIB_COARSE_MSK ;
  ul_Fine_calibration_value = DFLL_CALIB_FINE & DFLL_CALIB_FINE_MSK;

  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP( 4 ) | // Coarse step is 4
                         SYSCTRL_DFLLMUL_FSTEP( 1 ) | // Fine step is 1
                         SYSCTRL_DFLLMUL_MUL( (48000000ul/32768ul) ) ; // External 32KHz is the reference
  SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE( ul_Coarse_calibration_value ) | SYSCTRL_DFLLVAL_FINE( ul_Fine_calibration_value ) ;

  /* Write full configuration to DFLL control register */
  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_MODE | /* Enable the closed loop mode */
                          SYSCTRL_DFLLCTRL_QLDIS | /* Disable Quick lock */
                          SYSCTRL_DFLLCTRL_ENABLE ;

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
  {
    /* Wait for synchronization */
  }

  /*
   * 5) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
   */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID( 0 ) | // Generic Clock Generator 0
                     GCLK_GENDIV_DIV( 1 ) | // Divider set to 1
                     GCLK_GENCTRL_DIVSEL ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* Write Generic Clock Generator 0 configuration */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 0 ) | // Generic Clock Generator 0
                      GCLK_GENCTRL_SRC_DFLL48M | // Selected source is DFLL 48MHz
                      GCLK_GENCTRL_GENEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /*
   * 6) Put OSC8M as source for Generic Clock Generator 3
   */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID( 3 ) | // Generic Clock Generator 3
                     GCLK_GENDIV_DIV( 1 ) | // Divider set to 1 (8 MHz)
                     GCLK_GENCTRL_DIVSEL ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* Write Generic Clock Generator 0 configuration */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 3 ) | // Generic Clock Generator 3
                      GCLK_GENCTRL_SRC_OSC8M | // Selected source is DFLL 48MHz
                      GCLK_GENCTRL_GENEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

************************************************************
  /* CPU and BUS clocks */
  system_cpu_clock_set_divider(CONF_CLOCK_CPU_DIVIDER);
  system_main_clock_set_failure_detect(CONF_CLOCK_CPU_CLOCK_FAILURE_DETECT);
  system_apb_clock_set_divider(SYSTEM_CLOCK_APB_APBA, CONF_CLOCK_APBA_DIVIDER);
  system_apb_clock_set_divider(SYSTEM_CLOCK_APB_APBB, CONF_CLOCK_APBB_DIVIDER);
}

/**
 * \brief This is the code that gets called on processor reset.
 * To initialize the device, and call the main() routine.
 */
void Reset_Handler( void )
{
  uint32_t *pSrc, *pDest;

  /* Initialize the initialized data section */
  pSrc = &__etext;
  pDest = &__data_start__;

  if ( (&__data_start__ != &__data_end__) && (pSrc != pDest) )
  {
    for (; pDest < &__data_end__ ; pDest++, pSrc++ )
    {
      *pDest = *pSrc ;
    }
  }

  /* Clear the zero section */
  if ( (&__data_start__ != &__data_end__) && (pSrc != pDest) )
  {
    for ( pDest = &__bss_start__ ; pDest < &__bss_end__ ; pDest++ )
    {
      *pDest = 0 ;
    }
  }

  /* Initialize the C library */
//	__libc_init_array();

  SystemInit() ;

  /* Branch to main function */
  main() ;

  /* Infinite loop */
  while ( 1 )
  {
  }
}

/**
 * \brief Default interrupt handler for unused IRQs.
 */
void Dummy_Handler( void )
{
  while ( 1 )
  {
  }
}
