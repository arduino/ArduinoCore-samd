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
 * INCLUDE
 **************************************************************************************/

#include <FlashStorage.h>
#include <Arduino_MKRMEM.h>

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#define SFU_START    0x2000
#define SFU_SIZE     0x8000

#define SKETCH_START (uint32_t*)(SFU_START + SFU_SIZE)

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static char const UPDATE_FILE_NAME[] = "UPDATE.BIN";

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

FlashClass mcu_flash;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

extern "C" void __libc_init_array(void);

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main()
{
  init();

  __libc_init_array();

  delay(1);

  /* Initialize W25Q16DV flash chip found on Arduino MKRMEM Shield */
  flash.begin();

  /* Mount and verify filesystem */
  if ((SPIFFS_OK == filesystem.mount()) && (SPIFFS_OK == filesystem.check()))
  {
    /* Open update file */
    filesystem.clearerr();
    File file = filesystem.open(UPDATE_FILE_NAME, READ_ONLY);
    if(SPIFFS_OK == filesystem.err())
    { 
      bool update_success = false;
      /* Determine the size of the update file */
      int file_size = file.lseek(0, END);
      if (file_size > SFU_SIZE)
      {
        /* Skip the SFU section */
        file.lseek(SFU_SIZE, START);
        file_size -= SFU_SIZE;

        /* Erase the MCU flash */
        uint32_t flash_address = (uint32_t)SKETCH_START;
        mcu_flash.erase((void*)flash_address, file_size);

        /* Write the MCU flash */
        uint8_t buffer[512];
        for (int b = 0; b < file_size; b += sizeof(buffer))
        {
          file.read(buffer, sizeof(buffer));
          mcu_flash.write((void*)flash_address, buffer, sizeof(buffer));
          flash_address += sizeof(buffer);
        }
        update_success = true;
      }
      file.close();
      if (update_success) { filesystem.remove(UPDATE_FILE_NAME); }
      filesystem.unmount();
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
