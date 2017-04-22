# Arduino Zero Bootloader Binaries

This directory contains the SAM-BA m0+ bootloaders built by the
build_all_bootloaders.sh script from the 'MattairTech SAM M0+
Boards' Arduino core, which is available at
https://github.com/mattairtech/ArduinoCore-samd.

## MattairTech Boards
MattairTech boards are all configured with only one interface:
SAM_BA_USBCDC_ONLY (except C21, which uses SAM_BA_UART_ONLY).
CLOCKCONFIG_CLOCK_SOURCE is set to CLOCKCONFIG_INTERNAL_USB
(CLOCKCONFIG_INTERNAL for the C21). Only the main LED is defined.
BOOT_LOAD_PIN is not defined, but BOOT_DOUBLE_TAP_ENABLED is.

## Arduino/Genuino Boards
Arduino/Genuino boards are all configured with both interfaces.
CLOCKCONFIG_CLOCK_SOURCE is set to CLOCKCONFIG_32768HZ_CRYSTAL.
All LEDs that are installed for each board are defined (and some
have LED_POLARITY_LOW_ON set). BOOT_LOAD_PIN is not defined, but
BOOT_DOUBLE_TAP_ENABLED is.

## Generic Boards
The generic boards are all configured to minimize external hardware
requirements. Only one interface is enabled: SAM_BA_USBCDC_ONLY
(except C21, which uses SAM_BA_UART_ONLY). CLOCKCONFIG_CLOCK_SOURCE
is set to CLOCKCONFIG_INTERNAL_USB (CLOCKCONFIG_INTERNAL for the C21),
so no crystal is required. No LEDs are defined. BOOT_LOAD_PIN is not
defined, but BOOT_DOUBLE_TAP_ENABLED is, since it uses the reset pin.


## License
Copyright (c) 2015 Arduino LLC.  All right reserved.
Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.
Copyright (c) 2017 MattairTech LLC. All right reserved.

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
