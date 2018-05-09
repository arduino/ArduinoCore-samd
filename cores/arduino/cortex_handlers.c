/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  SAMD51 support added by Adafruit - Copyright (c) 2018 Dean Miller for Adafruit Industries

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

#include <sam.h>
#include <variant.h>
#include <stdio.h>

/* RTOS Hooks */
extern void svcHook(void);
extern void pendSVHook(void);
extern int sysTickHook(void);

/* Default empty handler */
void Dummy_Handler(void)
{
#if defined DEBUG
  __BKPT(3);
#endif
  for (;;) { }
}

#if defined(__SAMD51__)

/* Cortex-M4 processor handlers */
void Reset_Handler               ( void );
void NMI_Handler                 ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void HardFault_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void MemManage_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void BusFault_Handler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void UsageFault_Handler          ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SVC_Handler                 ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DebugMon_Handler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PendSV_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SysTick_Handler			 ( void );

/* Peripherals handlers */
void PM_Handler                  ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void MCLK_Handler                ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void OSCCTRL_0_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void OSCCTRL_1_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void OSCCTRL_2_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void OSCCTRL_3_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void OSCCTRL_4_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void OSC32KCTRL_Handler          ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SUPC_0_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SUPC_1_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void WDT_Handler                 ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RTC_Handler                 ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_0_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_1_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_2_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_3_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_4_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_5_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_6_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_7_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_8_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_9_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_10_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_11_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_12_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_13_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_14_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_15_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void FREQM_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void NVMCTRL_0_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void NVMCTRL_1_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DMAC_0_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DMAC_1_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DMAC_2_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DMAC_3_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DMAC_4_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EVSYS_0_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EVSYS_1_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EVSYS_2_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EVSYS_3_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EVSYS_4_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PAC_Handler                 ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TAL_0_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TAL_1_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RAMECC_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM0_0_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM0_1_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM0_2_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM0_3_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM1_0_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM1_1_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM1_2_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM1_3_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM2_0_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM2_1_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM2_2_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM2_3_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM3_0_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM3_1_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM3_2_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM3_3_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM4_0_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM4_1_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM4_2_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM4_3_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM5_0_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM5_1_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM5_2_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM5_3_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM6_0_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM6_1_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM6_2_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM6_3_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM7_0_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM7_1_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM7_2_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM7_3_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void USB_0_Handler               ( void ) __attribute__ ((weak));
void USB_1_Handler               ( void ) __attribute__ ((weak));
void USB_2_Handler               ( void ) __attribute__ ((weak));
void USB_3_Handler               ( void ) __attribute__ ((weak));
void TCC0_0_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC0_1_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC0_2_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC0_3_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC0_4_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC0_5_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC0_6_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC1_0_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC1_1_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC1_2_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC1_3_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC1_4_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC2_0_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC2_1_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC2_2_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC2_3_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC3_0_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC3_1_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC3_2_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC4_0_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC4_1_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC4_2_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC0_Handler                 ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC1_Handler                 ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC2_Handler                 ( void ) __attribute__ ((weak)); //used in Tone.cpp
void TC3_Handler                 ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC4_Handler                 ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC5_Handler                 ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC6_Handler                 ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC7_Handler                 ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PDEC_0_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PDEC_1_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PDEC_2_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void ADC0_0_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void ADC0_1_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void ADC1_0_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void ADC1_1_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void AC_Handler                  ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DAC_0_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DAC_1_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DAC_2_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DAC_3_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DAC_4_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void I2S_Handler                 ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PCC_Handler                 ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void AES_Handler                 ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TRNG_Handler                ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void ICM_Handler                 ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PUKCC_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void QSPI_Handler                ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SDHC0_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SDHC1_Handler               ( void ) __attribute__ ((weak, alias("Dummy_Handler")));

/* Initialize segments */
extern uint32_t __etext;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __StackTop;

/* Exception Table */
__attribute__ ((section(".isr_vector"))) const DeviceVectors exception_table =
{
	/* Configure Initial Stack Pointer, using linker-generated symbols */
	(void*) (&__StackTop),
	
	/* Cortex-M handlers */
	(void*) Reset_Handler,
	(void*) NMI_Handler,
	(void*) HardFault_Handler,
	(void*) MemManage_Handler,
	(void*) BusFault_Handler,
	(void*) UsageFault_Handler,
	(void*) (0UL), /* Reserved */
	(void*) (0UL), /* Reserved */
	(void*) (0UL), /* Reserved */
	(void*) (0UL), /* Reserved */
	(void*) SVC_Handler,
	(void*) DebugMon_Handler,
	(void*) (0UL), /* Reserved */
	(void*) PendSV_Handler,
	(void*) SysTick_Handler,

	/* Peripheral handlers */
	  (void*) PM_Handler,                    /*  0 Power Manager */
	  (void*) MCLK_Handler,                  /*  1 Main Clock */
	  (void*) OSCCTRL_0_Handler,             /*  2 Oscillators Control IRQ 0 */
	  (void*) OSCCTRL_1_Handler,             /*  3 Oscillators Control IRQ 1 */
	  (void*) OSCCTRL_2_Handler,             /*  4 Oscillators Control IRQ 2 */
	  (void*) OSCCTRL_3_Handler,             /*  5 Oscillators Control IRQ 3 */
	  (void*) OSCCTRL_4_Handler,             /*  6 Oscillators Control IRQ 4 */
	  (void*) OSC32KCTRL_Handler,            /*  7 32kHz Oscillators Control */
	  (void*) SUPC_0_Handler,                /*  8 Supply Controller IRQ 0 */
	  (void*) SUPC_1_Handler,                /*  9 Supply Controller IRQ 1 */
	  (void*) WDT_Handler,                   /* 10 Watchdog Timer */
	  (void*) RTC_Handler,                   /* 11 Real-Time Counter */
	  (void*) EIC_0_Handler,                 /* 12 External Interrupt Controller IRQ 0 */
	  (void*) EIC_1_Handler,                 /* 13 External Interrupt Controller IRQ 1 */
	  (void*) EIC_2_Handler,                 /* 14 External Interrupt Controller IRQ 2 */
	  (void*) EIC_3_Handler,                 /* 15 External Interrupt Controller IRQ 3 */
	  (void*) EIC_4_Handler,                 /* 16 External Interrupt Controller IRQ 4 */
	  (void*) EIC_5_Handler,                 /* 17 External Interrupt Controller IRQ 5 */
	  (void*) EIC_6_Handler,                 /* 18 External Interrupt Controller IRQ 6 */
	  (void*) EIC_7_Handler,                 /* 19 External Interrupt Controller IRQ 7 */
	  (void*) EIC_8_Handler,                 /* 20 External Interrupt Controller IRQ 8 */
	  (void*) EIC_9_Handler,                 /* 21 External Interrupt Controller IRQ 9 */
	  (void*) EIC_10_Handler,                /* 22 External Interrupt Controller IRQ 10 */
	  (void*) EIC_11_Handler,                /* 23 External Interrupt Controller IRQ 11 */
	  (void*) EIC_12_Handler,                /* 24 External Interrupt Controller IRQ 12 */
	  (void*) EIC_13_Handler,                /* 25 External Interrupt Controller IRQ 13 */
	  (void*) EIC_14_Handler,                /* 26 External Interrupt Controller IRQ 14 */
	  (void*) EIC_15_Handler,                /* 27 External Interrupt Controller IRQ 15 */
	  (void*) FREQM_Handler,                 /* 28 Frequency Meter */
	  (void*) NVMCTRL_0_Handler,             /* 29 Non-Volatile Memory Controller IRQ 0 */
	  (void*) NVMCTRL_1_Handler,             /* 30 Non-Volatile Memory Controller IRQ 1 */
	  (void*) DMAC_0_Handler,                /* 31 Direct Memory Access Controller IRQ 0 */
	  (void*) DMAC_1_Handler,                /* 32 Direct Memory Access Controller IRQ 1 */
	  (void*) DMAC_2_Handler,                /* 33 Direct Memory Access Controller IRQ 2 */
	  (void*) DMAC_3_Handler,                /* 34 Direct Memory Access Controller IRQ 3 */
	  (void*) DMAC_4_Handler,                /* 35 Direct Memory Access Controller IRQ 4 */
	  (void*) EVSYS_0_Handler,               /* 36 Event System Interface IRQ 0 */
	  (void*) EVSYS_1_Handler,               /* 37 Event System Interface IRQ 1 */
	  (void*) EVSYS_2_Handler,               /* 38 Event System Interface IRQ 2 */
	  (void*) EVSYS_3_Handler,               /* 39 Event System Interface IRQ 3 */
	  (void*) EVSYS_4_Handler,               /* 40 Event System Interface IRQ 4 */
	  (void*) PAC_Handler,                   /* 41 Peripheral Access Controller */
	  (void*) TAL_0_Handler,                 /* 42 Trigger Allocator IRQ 0 */
	  (void*) TAL_1_Handler,                 /* 43 Trigger Allocator IRQ 1 */
	  (void*) (0UL),
	  (void*) RAMECC_Handler,                /* 45 RAM ECC */
	  (void*) SERCOM0_0_Handler,             /* 46 Serial Communication Interface 0 IRQ 0 */
	  (void*) SERCOM0_1_Handler,             /* 47 Serial Communication Interface 0 IRQ 1 */
	  (void*) SERCOM0_2_Handler,             /* 48 Serial Communication Interface 0 IRQ 2 */
	  (void*) SERCOM0_3_Handler,             /* 49 Serial Communication Interface 0 IRQ 3 */
	  (void*) SERCOM1_0_Handler,             /* 50 Serial Communication Interface 1 IRQ 0 */
	  (void*) SERCOM1_1_Handler,             /* 51 Serial Communication Interface 1 IRQ 1 */
	  (void*) SERCOM1_2_Handler,             /* 52 Serial Communication Interface 1 IRQ 2 */
	  (void*) SERCOM1_3_Handler,             /* 53 Serial Communication Interface 1 IRQ 3 */
	  (void*) SERCOM2_0_Handler,             /* 54 Serial Communication Interface 2 IRQ 0 */
	  (void*) SERCOM2_1_Handler,             /* 55 Serial Communication Interface 2 IRQ 1 */
	  (void*) SERCOM2_2_Handler,             /* 56 Serial Communication Interface 2 IRQ 2 */
	  (void*) SERCOM2_3_Handler,             /* 57 Serial Communication Interface 2 IRQ 3 */
	  (void*) SERCOM3_0_Handler,             /* 58 Serial Communication Interface 3 IRQ 0 */
	  (void*) SERCOM3_1_Handler,             /* 59 Serial Communication Interface 3 IRQ 1 */
	  (void*) SERCOM3_2_Handler,             /* 60 Serial Communication Interface 3 IRQ 2 */
	  (void*) SERCOM3_3_Handler,             /* 61 Serial Communication Interface 3 IRQ 3 */
	  (void*) SERCOM4_0_Handler,             /* 62 Serial Communication Interface 4 IRQ 0 */
	  (void*) SERCOM4_1_Handler,             /* 63 Serial Communication Interface 4 IRQ 1 */
	  (void*) SERCOM4_2_Handler,             /* 64 Serial Communication Interface 4 IRQ 2 */
	  (void*) SERCOM4_3_Handler,             /* 65 Serial Communication Interface 4 IRQ 3 */
	  (void*) SERCOM5_0_Handler,             /* 66 Serial Communication Interface 5 IRQ 0 */
	  (void*) SERCOM5_1_Handler,             /* 67 Serial Communication Interface 5 IRQ 1 */
	  (void*) SERCOM5_2_Handler,             /* 68 Serial Communication Interface 5 IRQ 2 */
	  (void*) SERCOM5_3_Handler,             /* 69 Serial Communication Interface 5 IRQ 3 */
	  (void*) SERCOM6_0_Handler,             /* 70 Serial Communication Interface 6 IRQ 0 */
	  (void*) SERCOM6_1_Handler,             /* 71 Serial Communication Interface 6 IRQ 1 */
	  (void*) SERCOM6_2_Handler,             /* 72 Serial Communication Interface 6 IRQ 2 */
	  (void*) SERCOM6_3_Handler,             /* 73 Serial Communication Interface 6 IRQ 3 */
	  (void*) SERCOM7_0_Handler,             /* 74 Serial Communication Interface 7 IRQ 0 */
	  (void*) SERCOM7_1_Handler,             /* 75 Serial Communication Interface 7 IRQ 1 */
	  (void*) SERCOM7_2_Handler,             /* 76 Serial Communication Interface 7 IRQ 2 */
	  (void*) SERCOM7_3_Handler,             /* 77 Serial Communication Interface 7 IRQ 3 */
	  (void*) (0UL),
	  (void*) (0UL),
	  (void*) USB_0_Handler,                 /* 80 Universal Serial Bus IRQ 0 */
	  (void*) USB_1_Handler,                 /* 81 Universal Serial Bus IRQ 1 */
	  (void*) USB_2_Handler,                 /* 82 Universal Serial Bus IRQ 2 */
	  (void*) USB_3_Handler,                 /* 83 Universal Serial Bus IRQ 3 */
	  (void*) (0UL),
	  (void*) TCC0_0_Handler,                /* 85 Timer Counter Control 0 IRQ 0 */
	  (void*) TCC0_1_Handler,                /* 86 Timer Counter Control 0 IRQ 1 */
	  (void*) TCC0_2_Handler,                /* 87 Timer Counter Control 0 IRQ 2 */
	  (void*) TCC0_3_Handler,                /* 88 Timer Counter Control 0 IRQ 3 */
	  (void*) TCC0_4_Handler,                /* 89 Timer Counter Control 0 IRQ 4 */
	  (void*) TCC0_5_Handler,                /* 90 Timer Counter Control 0 IRQ 5 */
	  (void*) TCC0_6_Handler,                /* 91 Timer Counter Control 0 IRQ 6 */
	  (void*) TCC1_0_Handler,                /* 92 Timer Counter Control 1 IRQ 0 */
	  (void*) TCC1_1_Handler,                /* 93 Timer Counter Control 1 IRQ 1 */
	  (void*) TCC1_2_Handler,                /* 94 Timer Counter Control 1 IRQ 2 */
	  (void*) TCC1_3_Handler,                /* 95 Timer Counter Control 1 IRQ 3 */
	  (void*) TCC1_4_Handler,                /* 96 Timer Counter Control 1 IRQ 4 */
	  (void*) TCC2_0_Handler,                /* 97 Timer Counter Control 2 IRQ 0 */
	  (void*) TCC2_1_Handler,                /* 98 Timer Counter Control 2 IRQ 1 */
	  (void*) TCC2_2_Handler,                /* 99 Timer Counter Control 2 IRQ 2 */
	  (void*) TCC2_3_Handler,                /* 100 Timer Counter Control 2 IRQ 3 */
	  (void*) TCC3_0_Handler,                /* 101 Timer Counter Control 3 IRQ 0 */
	  (void*) TCC3_1_Handler,                /* 102 Timer Counter Control 3 IRQ 1 */
	  (void*) TCC3_2_Handler,                /* 103 Timer Counter Control 3 IRQ 2 */
	  (void*) TCC4_0_Handler,                /* 104 Timer Counter Control 4 IRQ 0 */
	  (void*) TCC4_1_Handler,                /* 105 Timer Counter Control 4 IRQ 1 */
	  (void*) TCC4_2_Handler,                /* 106 Timer Counter Control 4 IRQ 2 */
	  (void*) TC0_Handler,                   /* 107 Basic Timer Counter 0 */
	  (void*) TC1_Handler,                   /* 108 Basic Timer Counter 1 */
	  (void*) TC2_Handler,                   /* 109 Basic Timer Counter 2 */
	  (void*) TC3_Handler,                   /* 110 Basic Timer Counter 3 */
	  (void*) TC4_Handler,                   /* 111 Basic Timer Counter 4 */
	  (void*) TC5_Handler,                   /* 112 Basic Timer Counter 5 */
	  (void*) TC6_Handler,                   /* 113 Basic Timer Counter 6 */
	  (void*) TC7_Handler,                   /* 114 Basic Timer Counter 7 */
	  (void*) PDEC_0_Handler,                /* 115 Quadrature Decodeur IRQ 0 */
	  (void*) PDEC_1_Handler,                /* 116 Quadrature Decodeur IRQ 1 */
	  (void*) PDEC_2_Handler,                /* 117 Quadrature Decodeur IRQ 2 */
	  (void*) ADC0_0_Handler,                /* 118 Analog Digital Converter 0 IRQ 0 */
	  (void*) ADC0_1_Handler,                /* 119 Analog Digital Converter 0 IRQ 1 */
	  (void*) ADC1_0_Handler,                /* 120 Analog Digital Converter 1 IRQ 0 */
	  (void*) ADC1_1_Handler,                /* 121 Analog Digital Converter 1 IRQ 1 */
	  (void*) AC_Handler,                    /* 122 Analog Comparators */
	  (void*) DAC_0_Handler,                 /* 123 Digital-to-Analog Converter IRQ 0 */
	  (void*) DAC_1_Handler,                 /* 124 Digital-to-Analog Converter IRQ 1 */
	  (void*) DAC_2_Handler,                 /* 125 Digital-to-Analog Converter IRQ 2 */
	  (void*) DAC_3_Handler,                 /* 126 Digital-to-Analog Converter IRQ 3 */
	  (void*) DAC_4_Handler,                 /* 127 Digital-to-Analog Converter IRQ 4 */
	  (void*) I2S_Handler,                   /* 128 Inter-IC Sound Interface */
	  (void*) PCC_Handler,                   /* 129 Parallel Capture Controller */
	  (void*) AES_Handler,                   /* 130 Advanced Encryption Standard */
	  (void*) TRNG_Handler,                  /* 131 True Random Generator */
	  (void*) ICM_Handler,                   /* 132 Integrity Check Monitor */
	  (void*) PUKCC_Handler,                 /* 133 PUblic-Key Cryptography Controller */
	  (void*) QSPI_Handler,                  /* 134 Quad SPI interface */
	  (void*) SDHC0_Handler,                 /* 135 SD/MMC Host Controller 0 */
	  (void*) SDHC1_Handler,                 /* 136 SD/MMC Host Controller 1 */
};

#else

/* Cortex-M0+ core handlers */
void HardFault_Handler(void) __attribute__ ((weak, alias("Dummy_Handler")));
void Reset_Handler    (void);
void NMI_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void SVC_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void PendSV_Handler   (void) __attribute__ ((weak, alias("Dummy_Handler")));
void SysTick_Handler  (void);

/* Peripherals handlers */
void PM_Handler       (void) __attribute__ ((weak, alias("Dummy_Handler")));
void SYSCTRL_Handler  (void) __attribute__ ((weak, alias("Dummy_Handler")));
void WDT_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void RTC_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void NVMCTRL_Handler  (void) __attribute__ ((weak, alias("Dummy_Handler")));
void DMAC_Handler     (void) __attribute__ ((weak, alias("Dummy_Handler")));
void USB_Handler      (void) __attribute__ ((weak));
void EVSYS_Handler    (void) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM0_Handler  (void) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM1_Handler  (void) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM2_Handler  (void) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM3_Handler  (void) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM4_Handler  (void) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM5_Handler  (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC0_Handler     (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC1_Handler     (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC2_Handler     (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TC3_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TC4_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TC5_Handler      (void) __attribute__ ((weak)); // Used in Tone.cpp
void TC6_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TC7_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void ADC_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void AC_Handler       (void) __attribute__ ((weak, alias("Dummy_Handler")));
void DAC_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void PTC_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void I2S_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));

/* Initialize segments */
extern uint32_t __etext;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __StackTop;

/* Exception Table */
__attribute__ ((section(".isr_vector"))) const DeviceVectors exception_table =
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
  (void*) I2S_Handler,            /* 27 Inter-IC Sound Interface */
  (void*) (0UL),                  /* Reserved */
};

#endif

extern int main(void);

/* This is called on processor reset to initialize the device and call main() */
void Reset_Handler(void)
{
  uint32_t *pSrc, *pDest;

  /* Initialize the initialized data section */
  pSrc = &__etext;
  pDest = &__data_start__;

  if ((&__data_start__ != &__data_end__) && (pSrc != pDest)) {
    for (; pDest < &__data_end__; pDest++, pSrc++)
      *pDest = *pSrc;
  }

  /* Clear the zero section */
  if ((&__data_start__ != &__data_end__) && (pSrc != pDest)) {
    for (pDest = &__bss_start__; pDest < &__bss_end__; pDest++)
      *pDest = 0;
  }

#if defined(__FPU_USED) && defined(__SAMD51__)
	/* Enable FPU */
	SCB->CPACR |= (0xFu << 20);
	__DSB();
	__ISB();
#endif

  SystemInit();

  main();

  while (1)
    ;
}

/* Default Arduino systick handler */
extern void SysTick_DefaultHandler(void);

void SysTick_Handler(void)
{
  if (sysTickHook())
    return;
  SysTick_DefaultHandler();
}

static void (*usb_isr)(void) = NULL;

#if defined(__SAMD51__)
void USB_0_Handler(void)
{
	if (usb_isr)
	usb_isr();
}
void USB_1_Handler(void)
{
	if (usb_isr)
	usb_isr();
}
void USB_2_Handler(void)
{
	if (usb_isr)
	usb_isr();
}
void USB_3_Handler(void)
{
	if (usb_isr)
	usb_isr();
}
#else
void USB_Handler(void)
{
  if (usb_isr)
    usb_isr();
}
#endif

void USB_SetHandler(void (*new_usb_isr)(void))
{
  usb_isr = new_usb_isr;
}
