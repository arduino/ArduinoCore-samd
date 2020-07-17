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

#include <FlashStorage.h>
#include <MKRNB.h>

#define SBU_START    0x2000
#define SBU_SIZE     0x8000

#define SKETCH_START (uint32_t*)(SBU_START + SBU_SIZE)

static constexpr char UPDATE_FILE_NAME[] = "UPDATE.BIN";
static constexpr char CHECK_FILE_NAME[] = "UPDATE.OK";

FlashClass mcu_flash;

NBFileUtils  fileUtils(true);

extern "C" void __libc_init_array(void);

int main()
{
  init();

  __libc_init_array();

  delay(1);

  constexpr size_t blockSize = 512;
  fileUtils.begin();

  bool update_success = false;

  // Try to update only if update file
  // has been download successfully.

  if (fileUtils.existFile(CHECK_FILE_NAME)) {
    uint32_t updateSize = fileUtils.listFile(UPDATE_FILE_NAME);
    uint32_t tot_bytes = 0;
    uint32_t read_bytes = 0;

    if (updateSize > SBU_SIZE) {
      updateSize = updateSize - SBU_SIZE;
      size_t cycles = (updateSize / blockSize);
      size_t spare_bytes = (updateSize % blockSize);
      /* Erase the MCU flash */
      uint32_t flash_address = (uint32_t)SKETCH_START;
      mcu_flash.erase((void*)flash_address, updateSize);

      for (auto i = 0; i < cycles; i++) {
        uint8_t block[blockSize] { 0 };
        digitalWrite(LED_BUILTIN, LOW);
        read_bytes = fileUtils.readBlock(UPDATE_FILE_NAME, (i * blockSize) + SBU_SIZE, blockSize, block);
        digitalWrite(LED_BUILTIN, HIGH);
        mcu_flash.write((void*)flash_address, block, read_bytes);
        flash_address += read_bytes;
        tot_bytes += read_bytes;
      }

      if (spare_bytes){
        uint8_t block[spare_bytes] { 0 };
        digitalWrite(LED_BUILTIN, LOW);
        read_bytes = fileUtils.readBlock(UPDATE_FILE_NAME, tot_bytes + SBU_SIZE, spare_bytes, block);
        digitalWrite(LED_BUILTIN, HIGH);
        mcu_flash.write((void*)flash_address, block, read_bytes);
        flash_address += read_bytes;
      }
      update_success = true;
    }
    if (update_success) {
      fileUtils.deleteFile(UPDATE_FILE_NAME);
      fileUtils.deleteFile(CHECK_FILE_NAME);
    }
  }

boot:
  /* Jump to the sketch */
  __set_MSP(*SKETCH_START);

  /* Reset vector table address */
  SCB->VTOR = ((uint32_t)(SKETCH_START) & SCB_VTOR_TBLOFF_Msk);

  /* Address of Reset_Handler is written by the linker at the beginning of the .text section (see linker script) */
  uint32_t resetHandlerAddress = (uint32_t) * (SKETCH_START + 1);
  /* Jump to reset handler */
  asm("bx %0"::"r"(resetHandlerAddress));
}
