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

#ifndef _MONITOR_SAM_BA_H_
#define _MONITOR_SAM_BA_H_

#define SAM_BA_VERSION              "2.0"

/* Enable the interfaces to save code size */
#define SAM_BA_BOTH_INTERFACES      0
#define SAM_BA_UART_ONLY            1
#define SAM_BA_USBCDC_ONLY          2

#ifndef SAM_BA_INTERFACE
#define SAM_BA_INTERFACE    SAM_BA_BOTH_INTERFACES
#endif

/* Selects USB as the communication interface of the monitor */
#define SAM_BA_INTERFACE_USBCDC     0
/* Selects USART as the communication interface of the monitor */
#define SAM_BA_INTERFACE_USART      1

/* Selects USB as the communication interface of the monitor */
#define SIZEBUFMAX                  64

/**
 * \brief Initialize the monitor
 *
 */
void sam_ba_monitor_init(uint8_t com_interface);

/**
 * \brief System tick function of the SAM-BA Monitor
 *
 */
void sam_ba_monitor_sys_tick(void);

/**
 * \brief Main function of the SAM-BA Monitor
 *
 */
void sam_ba_monitor_run(void);

/**
 * \brief
 *
 */
void sam_ba_putdata_term(uint8_t* data, uint32_t length);

/**
 * \brief
 *
 */
void call_applet(uint32_t address);

#endif // _MONITOR_SAM_BA_H_
