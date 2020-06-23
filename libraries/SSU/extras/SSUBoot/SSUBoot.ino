/*
  Copyright (c) 2020 Arduino LLC.  All right reserved.

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

/**************************************************************************************
   INCLUDE
 **************************************************************************************/

#include <FlashStorage.h>
#include "MKRGSM.h"

/**************************************************************************************
   DEFINE
 **************************************************************************************/

#define SSU_START    0x2000
#define SSU_SIZE     0x8000

#define SKETCH_START (uint32_t*)(SSU_START + SSU_SIZE)

/**************************************************************************************
   GLOBAL CONSTANTS
 **************************************************************************************/

static char const UPDATE_FILE_NAME[] = "UPDATE.BIN";

/**************************************************************************************
   GLOBAL VARIABLES
 **************************************************************************************/

FlashClass mcu_flash;
GSMFileUtils  fileUtils(false);
/**************************************************************************************
   FUNCTION DECLARATION
 **************************************************************************************/

extern "C" void __libc_init_array(void);

/**************************************************************************************
   MAIN
 **************************************************************************************/

int main()
{
  init();

  __libc_init_array();

  delay(1);

  String filename = UPDATE_FILE_NAME;

  const size_t  blockSize = 512;

  MODEM.begin();
  fileUtils.begin();
  bool update_success = false;
  if (fileUtils.listFile("UPDATE.OK") == 1) {
    auto size = fileUtils.listFile(filename);
    auto cycles = (size / blockSize) + 1;
    if (size > SSU_SIZE) {
      size -= SSU_SIZE;

      /* Erase the MCU flash */
      uint32_t flash_address = (uint32_t)SKETCH_START;
      mcu_flash.erase((void*)flash_address, size);

      for (auto i = 0; i < cycles; i++) {
        uint8_t block[blockSize] { 0 };
        digitalWrite(LED_BUILTIN,LOW);
        auto read = fileUtils.readBlock(filename, (i * blockSize) + SSU_SIZE, blockSize, block);
        digitalWrite(LED_BUILTIN,HIGH);
        mcu_flash.write((void*)flash_address, block, read);
        flash_address += read;
      }
      update_success = true;
    }
    if (update_success) {
      fileUtils.deleteFile(filename);
      fileUtils.deleteFile("UPDATE.OK");
    }
  }
  /* Jump to the sketch */
  __set_MSP(*SKETCH_START);

  /* Reset vector table address */
  SCB->VTOR = ((uint32_t)(SKETCH_START) & SCB_VTOR_TBLOFF_Msk);

  /* Address of Reset_Handler is written by the linker at the beginning of the .text section (see linker script) */
  uint32_t resetHandlerAddress = (uint32_t) * (SKETCH_START + 1);
  /* Jump to reset handler */
  asm("bx %0"::"r"(resetHandlerAddress));
}
