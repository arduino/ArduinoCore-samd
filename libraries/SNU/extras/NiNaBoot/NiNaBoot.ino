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

#include <WiFiNINA.h>
#include <FlashStorage.h>

#include "lzss.h"

#ifdef ARDUINO_SAMD_MKRVIDOR4000
#include <VidorPeripherals.h>
#endif /* ARDUINO_SAMD_MKRVIDOR4000 */

#ifdef ARDUINO_SAMD_MKRVIDOR4000
#define NINA_GPIO0  FPGA_NINA_GPIO0
#define NINA_RESETN FPGA_SPIWIFI_RESET
#endif /* ARDUINO_SAMD_MKRVIDOR4000 */

#define SDU_START    0x2000
#define SDU_SIZE     0x4000

#define SKETCH_START (uint32_t*)(SDU_START + SDU_SIZE)

const char * UPDATE_FILE_NAME      = "/fs/UPDATE.BIN";
const char * UPDATE_FILE_NAME_LZSS = "/fs/UPDATE.BIN.LZSS";

FlashClass flash;

// Initialize C library
extern "C" void __libc_init_array(void);

int main() {
  init();

  __libc_init_array();

  delay(1);

#if defined(ARDUINO_SAMD_MKRVIDOR4000)
  FPGA.begin();
  /* NINA select SPI mode and enable (by setting RESETN = '1') */
  FPGA.pinMode     (NINA_GPIO0,  OUTPUT);
  FPGA.digitalWrite(NINA_GPIO0,  HIGH);
  FPGA.pinMode     (NINA_RESETN, OUTPUT);
  FPGA.digitalWrite(NINA_RESETN, HIGH);
#elif defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT)
  /* NINA select SPI mode and enable (by setting RESETN = '1') */
  pinMode     (NINA_GPIO0,  OUTPUT);
  digitalWrite(NINA_GPIO0,  HIGH);
  pinMode     (NINA_RESETN, OUTPUT);
  digitalWrite(NINA_RESETN, HIGH);
#endif

  if (WiFi.status() == WL_NO_SHIELD) {
    goto boot;
  }

  /* For UPDATE.BIN.LZSS - LZSS compressed binary files. */
  if (WiFiStorage.exists(UPDATE_FILE_NAME_LZSS))
  {
    WiFiStorageFile update_file = WiFiStorage.open(UPDATE_FILE_NAME_LZSS);
    /* Erase the complete flash starting from the SSU forward
     * because we've got no possibility of knowing how large
     * the decompressed binary will finally be.
     */
    flash.erase((void*)SKETCH_START, 0x40000 - (uint32_t)SKETCH_START);
    /* Initialize the lzss module with the data which
     * it requires.
     */
    lzss_init(&update_file, (uint32_t)SKETCH_START);
    /* During the process of decoding UPDATE.BIN.LZSS
     * is decompressed and stored as UPDATE.BIN.
     */
    lzss_decode();
    /* Write the data remaining in the write buffer to
     * the file.
     */
    lzss_flush();
    /* Delete UPDATE.BIN.LZSS because this update is complete. */
    update_file.close();
    update_file.erase();
  }
  /* For UPDATE.BIN - uncompressed binary files. */
  else if (WiFiStorage.exists(UPDATE_FILE_NAME)) {

    WiFiStorageFile updateFile = WiFiStorage.open(UPDATE_FILE_NAME);
    uint32_t updateSize = updateFile.size();
    bool updateFlashed = false;

    if (updateSize > SDU_SIZE) {
      // skip the SDU section
      updateFile.seek(SDU_SIZE);
      updateSize -= SDU_SIZE;

      uint32_t flashAddress = (uint32_t)SKETCH_START;

      // erase the pages
      flash.erase((void*)flashAddress, updateSize);

      uint8_t buffer[128];

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
      updateFile.erase();
    }
  }

boot:
  // jump to the sketch
  __set_MSP(*SKETCH_START);

  //Reset vector table address
  SCB->VTOR = ((uint32_t)(SKETCH_START) & SCB_VTOR_TBLOFF_Msk);

  // address of Reset_Handler is written by the linker at the beginning of the .text section (see linker script)
  uint32_t resetHandlerAddress = (uint32_t) * (SKETCH_START + 1);
  // jump to reset handler
  asm("bx %0"::"r"(resetHandlerAddress));
}
