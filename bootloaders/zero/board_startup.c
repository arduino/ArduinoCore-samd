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

#include <sam.h>

struct ConstVectors
{
  /* Stack pointer */
  void* pvStack;

  /* Cortex-M handlers */
  void* pfnReset_Handler;
  void* pfnNMI_Handler;
  void* pfnHardFault_Handler;
#if (SAMD51)
  void* pfnMemManage_Handler;
  void* pfnBusFault_Handler;
  void* pfnUsageFault_Handler;
#else
  void* pfnReservedM12;
  void* pfnReservedM11;
  void* pfnReservedM10;
#endif
  void* pfnReservedM9;
  void* pfnReservedM8;
  void* pfnReservedM7;
  void* pfnReservedM6;
  void* pfnSVC_Handler;
#if (SAMD51)
  void* pfnDebugMon_Handler;
#else
  void* pfnReservedM4;
#endif
  void* pfnReservedM3;
  void* pfnPendSV_Handler;
  void* pfnSysTick_Handler;
};

/* Symbols exported from linker script */
extern uint32_t __etext ;
extern uint32_t __data_start__ ;
extern uint32_t __data_end__ ;
extern uint32_t __bss_start__ ;
extern uint32_t __bss_end__ ;
extern uint32_t __StackTop;

extern int main(void);
extern void __libc_init_array(void);

/* Exception Table */
__attribute__ ((section(".isr_vector")))
const struct ConstVectors exception_table =
{
  /* Configure Initial Stack Pointer, using linker-generated symbols */
  .pvStack = (void*) (&__StackTop),

  .pfnReset_Handler      = (void*) Reset_Handler,
  .pfnNMI_Handler        = (void*) NMI_Handler,
  .pfnHardFault_Handler  = (void*) HardFault_Handler,
#if (SAMD51)
  .pfnMemManage_Handler  = (void*) MemManage_Handler,
  .pfnBusFault_Handler   = (void*) BusFault_Handler,
  .pfnUsageFault_Handler = (void*) UsageFault_Handler,
#else
  .pfnReservedM12        = (void*) (0UL), /* Reserved */
  .pfnReservedM11        = (void*) (0UL), /* Reserved */
  .pfnReservedM10        = (void*) (0UL), /* Reserved */
#endif
  .pfnReservedM9         = (void*) (0UL), /* Reserved */
  .pfnReservedM8         = (void*) (0UL), /* Reserved */
  .pfnReservedM7         = (void*) (0UL), /* Reserved */
  .pfnReservedM6         = (void*) (0UL), /* Reserved */
  .pfnSVC_Handler        = (void*) SVC_Handler,
#if (SAMD51)
  .pfnDebugMon_Handler  = (void*) DebugMon_Handler,
#else
  .pfnReservedM4         = (void*) (0UL), /* Reserved */
#endif
  .pfnReservedM3         = (void*) (0UL), /* Reserved */
  .pfnPendSV_Handler     = (void*) PendSV_Handler,
  .pfnSysTick_Handler    = (void*) SysTick_Handler,
};

/**
 * \brief This is the code that gets called on processor reset.
 * Initializes the device and call the main() routine.
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
  if ( &__bss_start__ != &__bss_end__ )
  {
    for ( pDest = &__bss_start__ ; pDest < &__bss_end__ ; pDest++ )
    {
      *pDest = 0ul ;
    }
  }

  /* Change default QOS values to have the best performance and correct USB behavior (applies to D21/D11). From startup_samd21.c from ASF 3.32. */
#if (SAMD21_SERIES || SAMD11_SERIES)
  SBMATRIX->SFR[SBMATRIX_SLAVE_HMCRAMC0].reg = 2;

  USB->DEVICE.QOSCTRL.reg = (USB_QOSCTRL_CQOS(2) | USB_QOSCTRL_DQOS(2));

  DMAC->QOSCTRL.reg = (DMAC_QOSCTRL_WRBQOS(2) | DMAC_QOSCTRL_FQOS(2) | DMAC_QOSCTRL_DQOS(2));
#endif

#if (SAMD51)
  #if __FPU_USED
    /* Enable FPU */
    SCB->CPACR |=  (0xFu << 20);
    __DSB();
    __ISB();
  #endif
#endif
//  board_init(); // will be done in main() after app check

  /* Initialize the C library */
//  __libc_init_array();

  main();

  while (1);
}

void NMI_Handler(void)
{
  __BKPT(14);
  while (1);
}

void HardFault_Handler(void)
{
  __BKPT(13);
  while (1);
}

void SVC_Handler(void)
{
  __BKPT(5);
  while (1);
}

void PendSV_Handler(void)
{
  __BKPT(2);
  while (1);
}

#if (SAMD51)
void MemManage_Handler(void)
{
  __BKPT(3);
  while (1);
}

void BusFault_Handler(void)
{
  __BKPT(4);
  while (1);
}

void UsageFault_Handler(void)
{
  __BKPT(6);
  while (1);
}

void DebugMon_Handler(void)
{
  __BKPT(7);
  while (1);
}
#endif
