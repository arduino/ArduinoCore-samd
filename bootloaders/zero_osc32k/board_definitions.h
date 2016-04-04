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

#define CPU_FREQUENCY                     (48000000ul) // SAM R21E XOSC max is 32MHz, not 48MHz like the SAM D21E.
//#define CPU_FREQUENCY                     (32000000ul) // SAM R21E XOSC max is 32MHz, not 48MHz like the SAM D21E.

//#define BOOT_USART_MODULE                 SERCOM1
//#define BOOT_USART_BUS_CLOCK_INDEX        PM_APBCMASK_SERCOM1
//#define BOOT_USART_PER_CLOCK_INDEX        GCLK_ID_SERCOM1_CORE
//#define BOOT_USART_PAD_SETTINGS           UART_RX_PAD1_TX_PAD0
//#define BOOT_USART_PAD3                   PINMUX_PA17C_SERCOM1_PAD1
//#define BOOT_USART_PAD2                   PINMUX_PA16C_SERCOM1_PAD0
//#define BOOT_USART_PAD1                   PINMUX_UNUSED
//#define BOOT_USART_PAD0                   PINMUX_UNUSED

#define BOOT_USART_MODULE                 SERCOM0
#define BOOT_USART_BUS_CLOCK_INDEX        PM_APBCMASK_SERCOM0
#define BOOT_USART_PER_CLOCK_INDEX        GCLK_ID_SERCOM0_CORE
#define BOOT_USART_PAD_SETTINGS           UART_RX_PAD3_TX_PAD2
#define BOOT_USART_PAD3                   PINMUX_PA11C_SERCOM0_PAD3
#define BOOT_USART_PAD2                   PINMUX_PA10C_SERCOM0_PAD2
#define BOOT_USART_PAD1                   PINMUX_UNUSED
#define BOOT_USART_PAD0                   PINMUX_UNUSED

/* Frequency of the board main oscillator */
//#define VARIANT_MAINOSC                      (32768ul)
//#define VARIANT_MAINOSC                    (8000000ul)
#define VARIANT_MAINOSC			  (12000000ul)

/* Master clock frequency */
#define VARIANT_MCK                       CPU_FREQUENCY

#define NVM_SW_CALIB_DFLL48M_COARSE_VAL   (58)
#define NVM_SW_CALIB_DFLL48M_FINE_VAL     (512)//(64)

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
