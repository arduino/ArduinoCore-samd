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
#include <Adafruit_SleepyDog.h>

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

#define UPDATE_FILE "/fs/UPDATE.BIN"
#define COUNTER_FILE "/fs/bootcounter.bin"

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

  Watchdog.enable(4000);

  if (WiFiStorage.exists(COUNTER_FILE)) {
    WiFiStorageFile counterFile = WiFiStorage.open(COUNTER_FILE);
    uint8_t counter;
    int status = -1;
    counterFile.read(&counter, 1);
    if (counter >= 3) {
      Watchdog.disable();
      Watchdog.enable(8000);
      int timeout = 10000;
      int start = millis();
      //it only works if it remembers the last working SSID
      while (status != WL_CONNECTED && (millis() - start) < timeout) {
        //fake credentials -> the last working credentials are retrieved
        status = WiFi.begin("foo", "bar");
        Watchdog.reset();
      }

      //set the URL to one where a valid binary is available
      WiFiStorage.download("http://192.168.1.100/Usage_mod.ino.bin", "UPDATE.BIN");
    }
  }

  if (WiFiStorage.exists(UPDATE_FILE)) {

    WiFiStorageFile updateFile = WiFiStorage.open(UPDATE_FILE);
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

        Watchdog.reset();
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
