# Arduino Zero Bootloader Binaries

The bootloaders/zero/binaries directory contains the SAM-BA
bootloaders built by the build_all_bootloaders.sh script from
the 'MattairTech SAM M0+ Boards' Arduino core, which is available
at https://github.com/mattairtech/ArduinoCore-samd. Each board
and chip combination has two bootloaders available:

* SAM-BA interface only
  * USB CDC only for all MattairTech boards
  * Both USB CDC and UART for most Arduino boards
  * The Generic board variants minimize external pin usage
    * Only the SAM-BA interface pins are used (no crystal, LED, etc.)
  * Filename is: sam_ba_$(BOARD_ID)_$(MCU)

* SAM-BA interface and SD Card interface
  * USB CDC only for all Arduino and most MattairTech boards
  * No SAM-BA interface for the D11 chips
  * All board variants define SDCARD_USE_PIN1 (except D11)
  * The Generic board variants use the LED
  * SDCARD_AUTORUN_DISABLED is defined
  * Filename is: sam_ba_sdcard_$(BOARD_ID)_$(MCU)

Please see the appropriate board_definitions file to see which pins
are used for the SD card. Note that the D51 uses different pins.


## MattairTech Boards

MattairTech boards are all configured with only one interface:
SAM_BA_USBCDC_ONLY (except C21, which uses SAM_BA_UART_ONLY).
CLOCKCONFIG_CLOCK_SOURCE is set to CLOCKCONFIG_INTERNAL_USB
(CLOCKCONFIG_INTERNAL for the C21). Only the main LED is defined.
BOOT_LOAD_PIN is not defined, but BOOT_DOUBLE_TAP_ENABLED is.
When the SD Card interface is enabled, SDCARD_AUTORUN_DISABLED and
SDCARD_USE_PIN1 are defined.

## MattairTech/Generic D11 Boards

All boards are configured with only the USB CDC interface, except
when SDCARD_ENABLED is defined, then only the SD Card interface is
enabled. ARDUINO_EXTENDED_CAPABILITIES is set to 0 (disabled).
TERMINAL_MODE_ENABLED is not defined. As of 1.6.8-beta-b2,
USB_VENDOR_STRINGS_ENABLED is now defined. BOOT_LOAD_PIN is not
defined, but BOOT_DOUBLE_TAP_ENABLED is. When the SD Card interface is
enabled, SDCARD_AUTORUN_DISABLED is defined (but not SDCARD_USE_PIN1).

## Arduino/Genuino Boards

Most Arduino/Genuino boards are configured with both interfaces,
except when SDCARD_ENABLED is defined, then only USB CDC is enabled.
CLOCKCONFIG_CLOCK_SOURCE is set to CLOCKCONFIG_32768HZ_CRYSTAL.
All LEDs that are installed for each board are defined (and some
have LED_POLARITY_LOW_ON set). BOOT_LOAD_PIN is not defined, but
BOOT_DOUBLE_TAP_ENABLED is. When the SD Card interface is enabled,
SDCARD_AUTORUN_DISABLED and SDCARD_USE_PIN1 are defined.

## Generic Boards

The generic boards are all configured to minimize external hardware
requirements. Only one interface is enabled: SAM_BA_USBCDC_ONLY
(except C21, which uses SAM_BA_UART_ONLY). CLOCKCONFIG_CLOCK_SOURCE
is set to CLOCKCONFIG_INTERNAL_USB (CLOCKCONFIG_INTERNAL for the C21),
so no crystal is required. No LEDs are defined. BOOT_LOAD_PIN is not
defined, but BOOT_DOUBLE_TAP_ENABLED is, since it uses the reset pin.
When the SD Card interface is enabled, SDCARD_AUTORUN_DISABLED and
SDCARD_USE_PIN1 are defined.


## License
Copyright (c) 2017 MattairTech LLC. All right reserved.
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

### Petit FatFS

Petit FatFs module is an open source software to implement FAT file system to
small embedded systems. This is a free software and is opened for education,
research and commercial developments under license policy of following trems.

Copyright (C) 2014, ChaN, all right reserved.

* The Petit FatFs module is a free software and there is NO WARRANTY.
* No restriction on use. You can use, modify and redistribute it for
  personal, non-profit or commercial use UNDER YOUR RESPONSIBILITY.
* Redistributions of source code must retain the above copyright notice.
