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
  void* pfnReservedM12;
  void* pfnReservedM11;
  void* pfnReservedM10;
  void* pfnReservedM9;
  void* pfnReservedM8;
  void* pfnReservedM7;
  void* pfnReservedM6;
  void* pfnSVC_Handler;
  void* pfnReservedM4;
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
  .pfnReservedM12        = (void*) (0UL), /* Reserved */
  .pfnReservedM11        = (void*) (0UL), /* Reserved */
  .pfnReservedM10        = (void*) (0UL), /* Reserved */
  .pfnReservedM9         = (void*) (0UL), /* Reserved */
  .pfnReservedM8         = (void*) (0UL), /* Reserved */
  .pfnReservedM7         = (void*) (0UL), /* Reserved */
  .pfnReservedM6         = (void*) (0UL), /* Reserved */
  .pfnSVC_Handler        = (void*) SVC_Handler,
  .pfnReservedM4         = (void*) (0UL), /* Reserved */
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
