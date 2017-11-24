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

#include "sdBootloader.h"

FATFS Fatfs;                    /* Petit-FatFs work area */
BYTE Buff[FLASH_PAGE_SIZE];        /* Page data buffer */

#ifndef SDCARD_FILENAME_PRIMARY
#define SDCARD_FILENAME_PRIMARY "UPDATE.BIN"
#endif
#ifndef SDCARD_FILENAME_SECONDARY
#define SDCARD_FILENAME_SECONDARY "UPDATE2.BIN"
#endif

char updateBin[] = SDCARD_FILENAME_PRIMARY;
char update2Bin[] = SDCARD_FILENAME_SECONDARY;

uint8_t sdBootloader (uint8_t mode)
{
  DWORD fa = APP_START;       /* Flash address */
  UINT br;        /* Bytes read */
  char * fileName;

  if (mode == SD_BOOTLOADER_MODE_UPDATE) {
    fileName = updateBin;
  } else {
    fileName = update2Bin;
  }

  if (pf_mount(&Fatfs) == FR_OK) {       /* Initialize file system */
    if (pf_open(fileName) == FR_OK) {      /* Open application file */
#if !defined(SDCARD_VERIFICATION_DISABLED)
      // Check if installed binary already matches file
      if (flashVerify()) {
        #if defined(SDCARD_AUTORUN_DISABLED)
        LED_status(LED_STATUS_FILE_ALREADY_MATCHES);
        #else
        systemReset();       // software reset, then execute flash
        #endif
      }
      pf_lseek(0);
#endif

      flashErase(fa); // erase entire FLASH beyond bootloader

      do {
        memset(Buff, 0xFF, FLASH_PAGE_SIZE);       /* Clear buffer */
        pf_read(Buff, FLASH_PAGE_SIZE, &br);       /* Load a page data */
        if (br) {
          if (fa >= FLASH_SIZE) {
            flashErase(APP_START); // erase entire FLASH beyond bootloader
            LED_status(LED_STATUS_FILE_TOO_LARGE);
          } else {
            flashWrite(br, (uint32_t *)Buff, (uint32_t *)fa);
          }
        }
        fa += FLASH_PAGE_SIZE;
      } while (br);

#if !defined(SDCARD_VERIFICATION_DISABLED)
      // Verification
      if (!flashVerify()) {
        flashErase(APP_START); // erase entire FLASH beyond bootloader
        LED_status(LED_STATUS_VERIFICATION_FAILURE);
      }
#endif

#if defined(SDCARD_AUTORUN_DISABLED)
      LED_status(LED_STATUS_SUCCESS);
#else
      systemReset();       // software reset, then execute flash
#endif
    }
  }

  return(SD_BOOTLOADER_FILE_NOT_FOUND);
}

bool flashVerify (void)
{
  DWORD fa = APP_START;
  UINT br;

  pf_lseek(0);

  do {
    pf_read(Buff, FLASH_PAGE_SIZE, &br);       /* Load a page data */

    if (br) {
      for (uint16_t i=0; i<br; i++) {
        if (Buff[i] != *((BYTE *)(fa + i))) {
          return (false);
        }
      }
    }

    fa += FLASH_PAGE_SIZE;
  } while (br);

  return (true);
}
