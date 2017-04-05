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

/* Set SAM_BA_INTERFACE to SAM_BA_USBCDC_ONLY, SAM_BA_UART_ONLY, or
 * SAM_BA_BOTH_INTERFACES. Select only one interface with 4KB bootloaders.
 * The C21 lacks USB, so set to SAM_BA_UART_ONLY in this case.
 */
#define SAM_BA_INTERFACE		SAM_BA_BOTH_INTERFACES

/* If ARDUINO_EXTENDED_CAPABILITIES is defined, 3 additional commands will
 * become available which will speed up programming when using the Arduino
 * IDE or the bossac tool standalone. Set to 0 with 4KB bootloaders.
 */
#define ARDUINO_EXTENDED_CAPABILITIES	1

/* The clock source must be chosen by setting CLOCKCONFIG_CLOCK_SOURCE to
 * CLOCKCONFIG_32768HZ_CRYSTAL, CLOCKCONFIG_HS_CRYSTAL, CLOCKCONFIG_INTERNAL,
 * or CLOCKCONFIG_INTERNAL_USB. If CLOCKCONFIG_32768HZ_CRYSTAL or
 * CLOCKCONFIG_HS_CRYSTAL is defined, then the PLL will be used. If
 * CLOCKCONFIG_HS_CRYSTAL is defined, then HS_CRYSTAL_FREQUENCY_HERTZ must
 * also be defined with the crystal frequency in Hertz. CLOCKCONFIG_INTERNAL
 * uses the DFLL in open-loop mode, except with the C21 which lacks a DFLL, so
 * the internal 48MHz RC oscillator is used instead. CLOCKCONFIG_INTERNAL_USB
 * can be defined for the D21, D11, or L21. It will also use the DFLL in
 * open-loop mode, except when connected to a USB port with data lines (and
 * not suspended), where it will calibrate against the USB SOF signal.
 */
#ifndef CLOCKCONFIG_CLOCK_SOURCE
#define CLOCKCONFIG_CLOCK_SOURCE	CLOCKCONFIG_INTERNAL_USB
#endif

/* If CLOCKCONFIG_HS_CRYSTAL is defined, then HS_CRYSTAL_FREQUENCY_HERTZ
 * must also be defined with the external crystal frequency in Hertz.
 */
//#define HS_CRYSTAL_FREQUENCY_HERTZ	16000000UL

/* If the PLL is used (CLOCKCONFIG_32768HZ_CRYSTAL, or CLOCKCONFIG_HS_CRYSTAL
 * defined), then PLL_FRACTIONAL_ENABLED can be defined, which will result in
 * a more accurate 48MHz output frequency at the expense of increased jitter.
 */
//#define PLL_FRACTIONAL_ENABLED

/* If both PLL_FAST_STARTUP and CLOCKCONFIG_HS_CRYSTAL are defined, the crystal
 * will be divided down to 1MHz - 2MHz, rather than 32KHz - 64KHz, before being
 * multiplied by the PLL. This will result in a faster lock time for the PLL,
 * however, it will also result in a less accurate PLL output frequency if the
 * crystal is not divisible (without remainder) by 1MHz. In this case, define
 * PLL_FRACTIONAL_ENABLED as well.
 */
//#define PLL_FAST_STARTUP

/* Master clock frequency (also Fcpu frequency) */
#define VARIANT_MCK                       (48000000ul)

/* The fine calibration value for DFLL open-loop mode is defined here.
 * The coarse calibration value is loaded from NVM OTP (factory calibration values).
 */
#define NVM_SW_CALIB_DFLL48M_FINE_VAL     (512)

/* If USB_VENDOR_STRINGS_ENABLED is defined, then STRING_MANUFACTURER and
 * STRING_PRODUCT will be sent to the host. Do not enable with 4KB bootloaders.
 */
#define USB_VENDOR_STRINGS_ENABLED
#define STRING_PRODUCT "Arduino m0 pro"

/* USB VID and PID */
#define USB_VID_HIGH   0x2A
#define USB_VID_LOW    0x03
#define USB_PID_HIGH   0x00
#define USB_PID_LOW    0x4D

/* USART configuration */
#define BOOT_USART_SERCOM_INSTANCE        5
#define BOOT_USART_PAD_SETTINGS           UART_RX_PAD3_TX_PAD2
#define BOOT_USART_PAD3                   PINMUX_PB23D_SERCOM5_PAD3
#define BOOT_USART_PAD2                   PINMUX_PB22D_SERCOM5_PAD2
#define BOOT_USART_PAD1                   PINMUX_UNUSED
#define BOOT_USART_PAD0                   PINMUX_UNUSED

/*
 * If BOOT_DOUBLE_TAP_ENABLED is defined the bootloader is started by
 * quickly tapping two times on the reset button.
 */
#define BOOT_DOUBLE_TAP_ENABLED

/*
 * If BOOT_LOAD_PIN is defined the bootloader is started if the selected
 * pin is tied LOW.
 */
//#define BOOT_LOAD_PIN                     PIN_PA21 // Pin 7
//#define BOOT_LOAD_PIN                     PIN_PA15 // Pin 5

/*
 * LEDs definitions
 * If the PORT is defined, then the associated LED is enabled
 * If BOARD_LED_FADE_ENABLED is defined, then the main LED produces
 * a PWM pulse or heartbeat, otherwise, it simply turns on if enabled.
 */
#define BOARD_LED_PORT                    (0)
#define BOARD_LED_PIN                     (17)
#define BOARD_LED_POLARITY	LED_POLARITY_HIGH_ON
#define BOARD_LED_FADE_ENABLED

#define BOARD_LEDRX_PORT                  (1)
#define BOARD_LEDRX_PIN                   (3)
#define BOARD_LEDRX_POLARITY	LED_POLARITY_LOW_ON

#define BOARD_LEDTX_PORT                  (0)
#define BOARD_LEDTX_PIN                   (27)
#define BOARD_LEDTX_POLARITY	LED_POLARITY_LOW_ON

#endif // _BOARD_DEFINITIONS_H_
