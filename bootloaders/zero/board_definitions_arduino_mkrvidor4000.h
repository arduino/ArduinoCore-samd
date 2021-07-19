/*
  Copyright (c) 2016 Arduino LLC.  All right reserved.

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
 * USB device definitions
 */
#define STRING_PRODUCT "Arduino MKR Vidor 4000"
#define USB_VID_HIGH   0x23
#define USB_VID_LOW    0x41
#define USB_PID_HIGH   0x00
#define USB_PID_LOW    0x56
#define USB_CURRENT_MA 500

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
//#define BOOT_LOAD_PIN                     PIN_PA21
//#define BOOT_LOAD_PIN                     PIN_PA15

/* Master clock frequency */
#define CPU_FREQUENCY                     (48000000ul)
#define VARIANT_MCK                       CPU_FREQUENCY

/* Frequency of the board main oscillator */
#define VARIANT_MAINOSC                   (32768ul)

/* Calibration values for DFLL48 pll */
#define NVM_SW_CALIB_DFLL48M_COARSE_VAL   (58)
#define NVM_SW_CALIB_DFLL48M_FINE_VAL     (64)

/*
 * LEDs definitions
 */
// PB08 (digital pin 32)
#define BOARD_LED_PORT                    (1)
#define BOARD_LED_PIN                     (8)

#define CONFIGURE_PMIC                    1
#define PMIC_PIN_SCL                      12
#define PMIC_PIN_SDA                      11
#define PMIC_SERCOM                       SERCOM0

#define HAS_EZ6301QI                      1

#define ENABLE_JTAG_LOAD                  1
#define TDI                               12
#define TDO                               15
#define TCK                               13
#define TMS                               14
#define MB_INT                            28

#define LAST_FLASH_PAGE                   (0x200000 - 0x1000)
#define SCRATCHPAD_FLASH_PAGE             (0x200000 - 0x2000)

typedef struct __attribute__((packed)) {
  uint32_t offset;
  uint32_t length;
  uint32_t sha256sum[8];
  uint32_t type;
  uint32_t force;
} externalFlashSignatures;

// No RX/TX led
//#define BOARD_LEDRX_PORT
//#define BOARD_LEDRX_PIN

//#define BOARD_LEDTX_PORT
//#define BOARD_LEDTX_PIN

#endif // _BOARD_DEFINITIONS_H_
