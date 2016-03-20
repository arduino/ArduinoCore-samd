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

#include <stdio.h>
#include <sam.h>
#include "sam_ba_monitor.h"
#include "sam_ba_serial.h"
#include "board_definitions.h"
#include "board_driver_led.h"
#include "sam_ba_usb.h"
#include "sam_ba_cdc.h"

extern uint32_t __sketch_vectors_ptr; // Exported value from linker script
extern void board_init(void);

#if (defined DEBUG) && (DEBUG == 1)
volatile uint32_t* pulSketch_Start_Address;
#endif

static volatile bool main_b_cdc_enable = false;

/**
 * \brief Check the application startup condition
 *
 */
static void check_start_application(void)
{
  LED_init();
  LED_off();

#if defined(BOOT_DOUBLE_TAP_ADDRESS)
  #define DOUBLE_TAP_MAGIC 0x07738135
  if (PM->RCAUSE.bit.POR)
  {
    /* On power-on initialize double-tap */
    BOOT_DOUBLE_TAP_DATA = 0;
  }
  else
  {
    if (BOOT_DOUBLE_TAP_DATA == DOUBLE_TAP_MAGIC)
    {
      /* Second tap, stay in bootloader */
      BOOT_DOUBLE_TAP_DATA = 0;
      return;
    }

    /* First tap */
    BOOT_DOUBLE_TAP_DATA = DOUBLE_TAP_MAGIC;

    /* Wait 0.5sec to see if the user tap reset again.
     * The loop value is based on SAMD21 default 1MHz clock @ reset.
     */
    for (uint32_t i=0; i<125000; i++) /* 500ms */
      /* force compiler to not optimize this... */
      __asm__ __volatile__("");

    /* Timeout happened, continue boot... */
    BOOT_DOUBLE_TAP_DATA = 0;
  }
#endif

#if (!defined DEBUG) || ((defined DEBUG) && (DEBUG == 0))
uint32_t* pulSketch_Start_Address;
#endif

  /*
   * Test sketch stack pointer @ &__sketch_vectors_ptr
   * Stay in SAM-BA if value @ (&__sketch_vectors_ptr) == 0xFFFFFFFF (Erased flash cell value)
   */
  if (__sketch_vectors_ptr == 0xFFFFFFFF)
  {
    /* Stay in bootloader */
    return;
  }

  /*
   * Load the sketch Reset Handler address
   * __sketch_vectors_ptr is exported from linker script and point on first 32b word of sketch vector table
   * First 32b word is sketch stack
   * Second 32b word is sketch entry point: Reset_Handler()
   */
  pulSketch_Start_Address = &__sketch_vectors_ptr ;
  pulSketch_Start_Address++ ;

  /*
   * Test vector table address of sketch @ &__sketch_vectors_ptr
   * Stay in SAM-BA if this function is not aligned enough, ie not valid
   */
  if ( ((uint32_t)(&__sketch_vectors_ptr) & ~SCB_VTOR_TBLOFF_Msk) != 0x00)
  {
    /* Stay in bootloader */
    return;
  }

/*
#if defined(BOOT_LOAD_PIN)
  volatile PortGroup *boot_port = (volatile PortGroup *)(&(PORT->Group[BOOT_LOAD_PIN / 32]));
  volatile bool boot_en;

  // Enable the input mode in Boot GPIO Pin
  boot_port->DIRCLR.reg = BOOT_PIN_MASK;
  boot_port->PINCFG[BOOT_LOAD_PIN & 0x1F].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
  boot_port->OUTSET.reg = BOOT_PIN_MASK;
  // Read the BOOT_LOAD_PIN status
  boot_en = (boot_port->IN.reg) & BOOT_PIN_MASK;

  // Check the bootloader enable condition
  if (!boot_en)
  {
    // Stay in bootloader
    return;
  }
#endif
*/

  LED_on();

  /* Rebase the Stack Pointer */
  __set_MSP( (uint32_t)(__sketch_vectors_ptr) );

  /* Rebase the vector table base address */
  SCB->VTOR = ((uint32_t)(&__sketch_vectors_ptr) & SCB_VTOR_TBLOFF_Msk);

  /* Jump to application Reset Handler in the application */
  asm("bx %0"::"r"(*pulSketch_Start_Address));
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
  board_init();
  __enable_irq();

#if SAM_BA_INTERFACE == SAM_BA_UART_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
  /* UART is enabled in all cases */
  serial_open();
#endif

#if SAM_BA_INTERFACE == SAM_BA_USBCDC_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
  pCdc = usb_init();
#endif

  DEBUG_PIN_LOW;

  LED_on();

  /* Wait for a complete enum on usb or a '#' char on serial line */
  while (1)
  {
    pulse_led(3); // while we're waiting, blink the D13 

#if SAM_BA_INTERFACE == SAM_BA_USBCDC_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
    if (pCdc->IsConfigured(pCdc) != 0)
    {
      main_b_cdc_enable = true;
    }

    /* Check if a USB enumeration has succeeded and if comm port has been opened */
    if (main_b_cdc_enable)
    {
      sam_ba_monitor_init(SAM_BA_INTERFACE_USBCDC);
      /* SAM-BA on USB loop */
      while( 1 )
      {
        sam_ba_monitor_run();
      }
    }
#endif

#if SAM_BA_INTERFACE == SAM_BA_UART_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
    /* Check if a '#' has been received */
    if (!main_b_cdc_enable && serial_sharp_received())
    {
      sam_ba_monitor_init(SAM_BA_INTERFACE_USART);
      /* SAM-BA on Serial loop */
      while(1)
      {
        sam_ba_monitor_run();
      }
    }
#endif
  }
}


// We'll have the D13 LED slowly pulse on and off with bitbang PWM
// for a nice 'hey we're in bootload mode' indication! -ada
static uint16_t pulse_tick=0;
#define BOOT_PULSE_MAX 1000
static int8_t  pulse_dir=1;
static int16_t pulse_pwm;
void pulse_led(int8_t speed) {
  // blink D13
  pulse_tick++;
  if (pulse_tick==BOOT_PULSE_MAX) {
    pulse_tick = 0;
    pulse_pwm += pulse_dir * speed;
    if (pulse_pwm > 255) {
      pulse_pwm = 255;
      pulse_dir = -1;
    }
    if (pulse_pwm < 0) {
      pulse_pwm = 0;
      pulse_dir = +1;
    }
    LED_on();
  }
  if (pulse_tick==pulse_pwm) 
    LED_off();
}
