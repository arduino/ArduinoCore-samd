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

/* Cortex-M0+ core handlers */
void HardFault_Handler(void) __attribute__ ((weak, alias("Dummy_Handler")));
void Reset_Handler    (void);
void NMI_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void SVC_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void PendSV_Handler   (void) __attribute__ ((weak, alias("Dummy_Handler")));
void SysTick_Handler  (void);

/* Peripherals handlers */
void SYSTEM_Handler   (void) __attribute__ ((weak, alias("Dummy_Handler")));
void PM_Handler       (void) __attribute__ ((weak, alias("Dummy_Handler")));
void SYSCTRL_Handler  (void) __attribute__ ((weak, alias("Dummy_Handler")));
void WDT_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void RTC_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void FREQM_Handler    (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TSENS_Handler    (void) __attribute__ ((weak, alias("Dummy_Handler")));
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
void CAN0_Handler     (void) __attribute__ ((weak, alias("Dummy_Handler")));
void CAN1_Handler     (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC0_Handler     (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC1_Handler     (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC2_Handler     (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TC0_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TC1_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TC2_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TC3_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TC4_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TC5_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TC6_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TC7_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void ADC_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void ADC0_Handler     (void) __attribute__ ((weak, alias("Dummy_Handler")));
void ADC1_Handler     (void) __attribute__ ((weak, alias("Dummy_Handler")));
void AC_Handler       (void) __attribute__ ((weak, alias("Dummy_Handler")));
void DAC_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void SDADC_Handler    (void) __attribute__ ((weak, alias("Dummy_Handler")));
void PTC_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void I2S_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void AES_Handler      (void) __attribute__ ((weak, alias("Dummy_Handler")));
void TRNG_Handler     (void) __attribute__ ((weak, alias("Dummy_Handler")));

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
#if (SAMD21 || SAMD11)
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
#if (SAMD21)
  (void*) SERCOM3_Handler,        /* 12 Serial Communication Interface 3 */
  (void*) SERCOM4_Handler,        /* 13 Serial Communication Interface 4 */
  (void*) SERCOM5_Handler,        /* 14 Serial Communication Interface 5 */
#endif
  (void*) TCC0_Handler,           /* 15 / 12 Timer Counter Control 0 */
#if (SAMD21)
  (void*) TCC1_Handler,           /* 16 Timer Counter Control 1 */
  (void*) TCC2_Handler,           /* 17 Timer Counter Control 2 */
  (void*) TC3_Handler,            /* 18 Basic Timer Counter 3 */
  (void*) TC4_Handler,            /* 19 Basic Timer Counter 4 */
  (void*) TC5_Handler,            /* 20 Basic Timer Counter 5 */
  (void*) TC6_Handler,            /* 21 Basic Timer Counter 6 */
  (void*) TC7_Handler,            /* 22 Basic Timer Counter 7 */
#else
  (void*) TC1_Handler,            /* 13 Basic Timer Counter 1 */
  (void*) TC2_Handler,            /* 14 Basic Timer Counter 2 */
#endif
  (void*) ADC_Handler,            /* 23 / 15 Analog Digital Converter */
  (void*) AC_Handler,             /* 24 / 16 Analog Comparators */
  (void*) DAC_Handler,            /* 25 / 17 Digital Analog Converter */
  (void*) PTC_Handler,            /* 26 / 18 Peripheral Touch Controller */
#if (SAMD21)
  (void*) I2S_Handler             /* 27 Inter-IC Sound Interface */
  (void*) (0UL),                  /* Reserved */
#endif

#elif (SAML21 || SAMC21)
  (void*) SYSTEM_Handler,         /*  0 SYSTEM handler (includes SYSTEM, MCLK, OSCCTRL, OSC32KCTRL, PAC, PM, SUPC, and TAL) */
  (void*) WDT_Handler,            /*  1 Watchdog Timer */
  (void*) RTC_Handler,            /*  2 Real-Time Counter */
  (void*) EIC_Handler,            /*  3 External Interrupt Controller */
#if (SAMC21)
  (void*) FREQM_Handler,          /*  4 FREQM */
  (void*) TSENS_Handler,          /*  5 TSENS */
#endif
  (void*) NVMCTRL_Handler,        /*  4 / 6 Non-Volatile Memory Controller */
  (void*) DMAC_Handler,           /*  5 / 7 Direct Memory Access Controller */
#if (SAML21)
  (void*) USB_Handler,            /*  6 Universal Serial Bus */
#endif
  (void*) EVSYS_Handler,          /*  7 / 8 Event System Interface */
  (void*) SERCOM0_Handler,        /*  8 / 9 Serial Communication Interface 0 */
  (void*) SERCOM1_Handler,        /*  9 / 10 Serial Communication Interface 1 */
  (void*) SERCOM2_Handler,        /* 10 / 11 Serial Communication Interface 2 */
  (void*) SERCOM3_Handler,        /* 11 / 12 Serial Communication Interface 3 */
  (void*) SERCOM4_Handler,        /* 12 / 13 Serial Communication Interface 4 */
  (void*) SERCOM5_Handler,        /* 13 / 14 Serial Communication Interface 5 */
#if (SAMC21)
  (void*) CAN0_Handler,           /* 15 CAN0 */
  (void*) CAN1_Handler,           /* 16 CAN1 */
#endif
  (void*) TCC0_Handler,           /* 14 / 17 Timer Counter Control 0 */
  (void*) TCC1_Handler,           /* 15 / 18 Timer Counter Control 1 */
  (void*) TCC2_Handler,           /* 16 / 19 Timer Counter Control 2 */
  (void*) TC0_Handler,            /* 17 / 20 Basic Timer Counter 0 */
  (void*) TC1_Handler,            /* 18 / 21 Basic Timer Counter 1 */
  (void*) TC2_Handler,            /* 19 / 22 Basic Timer Counter 2 */
  (void*) TC3_Handler,            /* 20 / 23 Basic Timer Counter 3 */
  (void*) TC4_Handler,            /* 21 / 24 Basic Timer Counter 4 */
#if (SAMC21)
  (void*) ADC0_Handler,           /* 25 ADC0 */
  (void*) ADC1_Handler,           /* 26 ADC1 */
#else
  (void*) ADC_Handler,            /* 22 Analog Digital Converter */
#endif
  (void*) AC_Handler,             /* 23 / 27 Analog Comparators */
  (void*) DAC_Handler,            /* 24 / 28 Digital Analog Converter */
#if (SAMC21)
  (void*) SDADC_Handler,          /* 29 SDADC */
#endif
  (void*) PTC_Handler,            /* 25 / 30 Peripheral Touch Controller */
#if (SAML21)
  (void*) AES_Handler,            /* 26 AES */
  (void*) TRNG_Handler,           /* 27 TRNG */
#endif

#else
#error "cortex_handlers.c: Unsupported chip"
#endif
};

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

  /* Change default QOS values to have the best performance and correct USB behaviour (applies to D21/D11). From startup_samd21.c from ASF 3.32. */
#if (SAMD21_SERIES || SAMD11_SERIES)
  SBMATRIX->SFR[SBMATRIX_SLAVE_HMCRAMC0].reg = 2;
#endif

#if defined(ID_USB)
  USB->DEVICE.QOSCTRL.bit.CQOS = 2;
  USB->DEVICE.QOSCTRL.bit.DQOS = 2;
#endif

  DMAC->QOSCTRL.bit.DQOS = 2;
  DMAC->QOSCTRL.bit.FQOS = 2;
  DMAC->QOSCTRL.bit.WRBQOS = 2;

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

#if !defined(USB_DISABLED)
static void (*usb_isr)(void) = NULL;

void USB_Handler(void)
{
  if (usb_isr)
    usb_isr();
}

void USB_SetHandler(void (*new_usb_isr)(void))
{
  usb_isr = new_usb_isr;
}
#endif
