/*
 * Copyright (c) 2017 MattairTech LLC. All right reserved.
 * Copyright (C) 2014, ChaN, all right reserved.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef _SD_BOOTLOADER_H_
#define _SD_BOOTLOADER_H_

#include "sam.h"
#include <string.h>
#include <stdbool.h>
#include "pff.h"
#include "../board_definitions.h"
#include "../board_driver_led.h"
#include "../util.h"

#define SD_BOOTLOADER_MODE_NO_UPDATE    0
#define SD_BOOTLOADER_MODE_UPDATE       1
#define SD_BOOTLOADER_MODE_UPDATE2      2

#define SD_BOOTLOADER_NOT_CALLED                0
#define SD_BOOTLOADER_SUCCESS                   1
#define SD_BOOTLOADER_FILE_ALREADY_MATCHES      2
#define SD_BOOTLOADER_FILE_NOT_FOUND            3
#define SD_BOOTLOADER_FILE_TOO_LARGE            4
#define SD_BOOTLOADER_VERIFICATION_FAILURE      5

uint8_t sdBootloader (uint8_t mode);
bool flashVerify (void);

#endif
