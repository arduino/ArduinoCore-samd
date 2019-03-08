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

#ifndef _BOARD_DEFINITIONS_H_
#define _BOARD_DEFINITIONS_H_

#if defined(BOARD_ID_arduino_zero)
  #include "board_definitions_arduino_zero.h"
#elif defined(BOARD_ID_genuino_zero)
  #include "board_definitions_genuino_zero.h"
#elif defined(BOARD_ID_arduino_mkr1000)
  #include "board_definitions_arduino_mkr1000.h"
#elif defined(BOARD_ID_genuino_mkr1000)
  #include "board_definitions_genuino_mkr1000.h"
#elif defined(BOARD_ID_arduino_mkrzero)
  #include "board_definitions_arduino_mkrzero.h"
#elif defined(BOARD_ID_arduino_mkrfox1200)
  #include "board_definitions_arduino_mkrfox1200.h"
#elif defined(BOARD_ID_arduino_mkrgsm1400)
  #include "board_definitions_arduino_mkrgsm1400.h"
#elif defined(BOARD_ID_arduino_mkrwan1300)
  #include "board_definitions_arduino_mkrwan1300.h"
#elif defined(BOARD_ID_arduino_mkrwifi1010)
  #include "board_definitions_arduino_mkrwifi1010.h"
#elif defined(BOARD_ID_arduino_mkrnb1500)
  #include "board_definitions_arduino_mkrnb1500.h"
#else
  #error You must define a BOARD_ID and add the corresponding definitions in board_definitions.h
#endif

// Common definitions
// ------------------

#define BOOT_PIN_MASK (1U << (BOOT_LOAD_PIN & 0x1f))

/*
 * If BOOT_DOUBLE_TAP_ADDRESS is defined the bootloader is started by
 * quickly tapping two times on the reset button.
 * BOOT_DOUBLE_TAP_ADDRESS must point to a free SRAM cell that must not
 * be touched from the loaded application.
 */
#define BOOT_DOUBLE_TAP_ADDRESS           (0x20007FFCul)
#define BOOT_DOUBLE_TAP_DATA              (*((volatile uint32_t *) BOOT_DOUBLE_TAP_ADDRESS))

/*
 * If BOOT_LOAD_PIN is defined the bootloader is started if the selected
 * pin is tied LOW.
 */
//#define BOOT_LOAD_PIN                     PIN_PA21 // Pin 7
//#define BOOT_LOAD_PIN                     PIN_PA15 // Pin 5
#define BOOT_PIN_MASK                     (1U << (BOOT_LOAD_PIN & 0x1f))

#define CPU_FREQUENCY                     (48000000ul)

#define BOOT_USART_MODULE                 SERCOM0
#define BOOT_USART_BUS_CLOCK_INDEX        PM_APBCMASK_SERCOM0
#define BOOT_USART_PER_CLOCK_INDEX        GCLK_ID_SERCOM0_CORE
#define BOOT_USART_PAD_SETTINGS           UART_RX_PAD3_TX_PAD2
#define BOOT_USART_PAD3                   PINMUX_PA11C_SERCOM0_PAD3
#define BOOT_USART_PAD2                   PINMUX_PA10C_SERCOM0_PAD2
#define BOOT_USART_PAD1                   PINMUX_UNUSED
#define BOOT_USART_PAD0                   PINMUX_UNUSED

/* Frequency of the board main oscillator */
#define VARIANT_MAINOSC	                  (32768ul)

/* Master clock frequency */
#define VARIANT_MCK			                  CPU_FREQUENCY

#define NVM_SW_CALIB_DFLL48M_COARSE_VAL   (58)
#define NVM_SW_CALIB_DFLL48M_FINE_VAL     (64)

/*
 * LEDs definitions
 */
#define BOARD_LED_PORT                    (0)
#define BOARD_LED_PIN                     (17)

#define BOARD_LEDRX_PORT                  (1)
#define BOARD_LEDRX_PIN                   (3)

#define BOARD_LEDTX_PORT                  (0)
#define BOARD_LEDTX_PIN                   (27)

#endif // _BOARD_DEFINITIONS_H_
