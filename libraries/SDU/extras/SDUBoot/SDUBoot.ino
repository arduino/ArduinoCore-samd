/*
  Copyright (c) 2017 Arduino LLC.  All right reserved.

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

#include <SD.h>
#include <FlashStorage.h>

#ifdef ARDUINO_SAM_ZERO
#define SDU_START    0x4000
#else
#define SDU_START    0x2000
#endif
#define SDU_SIZE     0x4000

#define SKETCH_START (uint32_t*)(SDU_START + SDU_SIZE)

#ifndef SDCARD_SS_PIN
#define SDCARD_SS_PIN 4
#endif

#define UPDATE_FILE "UPDATE.BIN"

FlashClass flash;

// Initialize C library
extern "C" void __libc_init_array(void);

int main() {
  init();

  __libc_init_array();

  delay(1);

  if (SD.begin(SDCARD_SS_PIN) && SD.exists(UPDATE_FILE)) {
    File updateFile = SD.open(UPDATE_FILE);
    uint32_t updateSize = updateFile.size();
    bool updateFlashed = false;

    if (updateSize > SDU_SIZE) {
      // skip the SDU section
      updateFile.seek(SDU_SIZE);
      updateSize -= SDU_SIZE;

      uint32_t flashAddress = (uint32_t)SKETCH_START;

      // erase the pages
      flash.erase((void*)flashAddress, updateSize);

      uint8_t buffer[512];

      // write the pages
      for (uint32_t i = 0; i < updateSize; i += sizeof(buffer)) {
        updateFile.read(buffer, sizeof(buffer));

        flash.write((void*)flashAddress, buffer, sizeof(buffer));

        flashAddress += sizeof(buffer);
      }

      updateFlashed = true;
    }

    updateFile.close();

    if (updateFlashed) {
      SD.remove(UPDATE_FILE);
    }
  }

  // jump to the sketch
  __set_MSP(*SKETCH_START);

  //Reset vector table address
  SCB->VTOR = ((uint32_t)(SKETCH_START) & SCB_VTOR_TBLOFF_Msk);

  // address of Reset_Handler is written by the linker at the beginning of the .text section (see linker script)
  uint32_t resetHandlerAddress = (uint32_t) * (SKETCH_START + 1);
  // jump to reset handler
  asm("bx %0"::"r"(resetHandlerAddress));
}


