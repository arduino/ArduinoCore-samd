/*
  This file is part of the SAMD Bootloader Updater library.
  Copyright (c) 2018 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "SAMD_BootloaderUpdater.h"

#define PAGE_SIZE                   (64)
#define PAGES                       (4096)
#define MAX_FLASH                   (PAGE_SIZE * PAGES)
#define ROW_SIZE                    (PAGE_SIZE * 4)

__attribute__((aligned (4)))
const uint8_t booloaderData[8192] = {
#if defined(ARDUINO_SAMD_MKRVIDOR4000)
  #include "bootloaders/mkrvidor4000.h"
#else
  #error "Unsupported board!"
#endif
};

#define BOOTLOADER_START 0x00000000
#define USER_ROW_START   0x00804000

extern "C" {
  // these functions must be in RAM (.data) and NOT inlined
  // as they erase and copy the sketch data in flash

  __attribute__ ((long_call, noinline, section (".data#")))
  static void eraseFlash(uint32_t address, int length)
  {
    for (int i = 0; i < length; i += ROW_SIZE) {
      NVMCTRL->ADDR.reg = ((uint32_t)((uint32_t)address + i)) / 2;
      NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
      
      while (!NVMCTRL->INTFLAG.bit.READY);
    }
  }

  __attribute__ ((long_call, noinline, section (".data#")))
  static void readFlash(uint32_t src, void* dest, int length)
  {
    memcpy(dest, (void*)src, length);
  }

  __attribute__ ((long_call, noinline, section (".data#")))
  static void writeFlash(uint32_t dest, const void* src, int length)
  {
    volatile uint32_t* d = (uint32_t*)dest;
    const volatile uint32_t* s = (const uint32_t*)src;

    for (int i = 0; i < length; i += 4) {
      *d++ = *s++;

      while (!NVMCTRL->INTFLAG.bit.READY);
    }
  }
}

bool SAMD_BootloaderUpdaterClass::needsUpdate()
{
  return (memcmp(BOOTLOADER_START, booloaderData, sizeof(booloaderData)) == 0);
}

int SAMD_BootloaderUpdaterClass::update(void(*progressCallback)(float))
{
  // enable auto page writes
  NVMCTRL->CTRLB.bit.MANW = 0;

  // read the user row
  uint32_t userRow[PAGE_SIZE / sizeof(uint32_t)];
  readFlash(USER_ROW_START, userRow, sizeof(userRow));

  if ((userRow[0] & 0x00000007) != 0x00000007) {
    // bootloader is protected, unprotect it
    userRow[0] |= 0x00000007;

    // erase the user row and flash the updated value
    eraseFlash(USER_ROW_START, sizeof(userRow));
    writeFlash(USER_ROW_START, userRow, sizeof(userRow));
  }

  if (progressCallback) {
    progressCallback(0.0);
  }

  #define CHUNK_SIZE (ROW_SIZE * 2)

  // erase and copy the flash row by row
  for (size_t i = 0; i < sizeof(booloaderData); i += CHUNK_SIZE) {
    eraseFlash(BOOTLOADER_START + i, CHUNK_SIZE);
    writeFlash(BOOTLOADER_START + i, &booloaderData[i], CHUNK_SIZE);

    if (progressCallback) {
      progressCallback((i + CHUNK_SIZE) * 100.0 / sizeof(booloaderData));
    }
  }

  return needsUpdate() ? 1 : 0;
}

SAMD_BootloaderUpdaterClass SAMD_BootloaderUpdater;
