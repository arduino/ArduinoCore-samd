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

/**************************************************************************************
   INCLUDE
 **************************************************************************************/

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

/**************************************************************************************
   DEFINE
 **************************************************************************************/

#define SDU_START    0x2000
#define SDU_SIZE     0x4000

#define SKETCH_START (uint32_t*)(SDU_START + SDU_SIZE)

/**************************************************************************************
   CONST
 **************************************************************************************/

const char * UPDATE_FILE_NAME      = "/fs/UPDATE.BIN";
const char * UPDATE_FILE_NAME_LZSS = "/fs/UPDATE.BIN.LZSS";

static const uint32_t crc_table[256] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
    0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
    0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
    0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
    0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
    0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
    0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
    0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
    0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
    0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
    0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
    0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
    0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
    0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
    0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
    0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
    0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
    0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
    0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
    0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
    0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
    0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

/**************************************************************************************
   FUNCTIONS
 **************************************************************************************/

uint32_t crc_update(uint32_t crc, const void * data, size_t data_len)
{
  const unsigned char *d = (const unsigned char *)data;
  unsigned int tbl_idx;

  while (data_len--) {
    tbl_idx = (crc ^ *d) & 0xff;
    crc = (crc_table[tbl_idx] ^ (crc >> 8)) & 0xffffffff;
    d++;
  }

  return crc & 0xffffffff;
}

/**************************************************************************************
   GLOBAL VARIABLES
 **************************************************************************************/

FlashClass flash;

// Initialize C library
extern "C" void __libc_init_array(void);

/**************************************************************************************
   MAIN
 **************************************************************************************/

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

  String nina_fw_version;

  if (WiFi.status() == WL_NO_SHIELD) {
    goto boot;
  }

  nina_fw_version = WiFi.firmwareVersion();
  if (nina_fw_version < "1.4.1") {
    goto boot;
  }

  /* For UPDATE.BIN.LZSS - LZSS compressed binary files. */
  if (WiFiStorage.exists(UPDATE_FILE_NAME_LZSS))
  {
    WiFiStorageFile update_file = WiFiStorage.open(UPDATE_FILE_NAME_LZSS);

    union HeaderVersion
    {
      struct __attribute__((packed))
      {
        uint32_t header_version    :  6;
        uint32_t compression       :  1;
        uint32_t signature         :  1;
        uint32_t spare             :  4;
        uint32_t payload_target    :  4;
        uint32_t payload_major     :  8;
        uint32_t payload_minor     :  8;
        uint32_t payload_patch     :  8;
        uint32_t payload_build_num : 24;
      } field;
      uint8_t buf[sizeof(field)];
      static_assert(sizeof(buf) == 8, "Error: sizeof(HEADER.VERSION) != 8");
    };

    union
    {
      struct __attribute__((packed))
      {
        uint32_t len;
        uint32_t crc32;
        uint32_t magic_number;
        HeaderVersion hdr_version;
      } header;
      uint8_t buf[sizeof(header)];
      static_assert(sizeof(buf) == 20, "Error: sizeof(HEADER) != 20");
    } ota_header;
    uint32_t crc32, bytes_read;
    uint8_t crc_buf[128];

    /* Read the OTA header ... */
    update_file.read(ota_header.buf, sizeof(ota_header.buf));

    /* ... and check first length ... */
    if (ota_header.header.len != (update_file.size() - sizeof(ota_header.header.len) - sizeof(ota_header.header.crc32))) {
      update_file.close();
      update_file.erase();
      goto boot;
    }

    /* ... and the CRC second ... rewind to start of CRC verified header ... */
    update_file.seek(sizeof(ota_header.header.len) + sizeof(ota_header.header.crc32));
    /* ... initialize CRC ... */
    crc32 = 0xFFFFFFFF;
    /* ... and calculate over file ... */
    for(bytes_read = 0;
        bytes_read < (ota_header.header.len - sizeof(crc_buf));
        bytes_read += sizeof(crc_buf))
    {
      update_file.read(crc_buf, sizeof(crc_buf));
      crc32 = crc_update(crc32, crc_buf, sizeof(crc_buf));
    }
    update_file.read(crc_buf, ota_header.header.len - bytes_read);
    crc32 = crc_update(crc32, crc_buf, ota_header.header.len - bytes_read);
    /* ... then finalise ... */
    crc32 ^= 0xFFFFFFFF;
    /* ... and compare. */
    if (ota_header.header.crc32 != crc32) {
      update_file.close();
      update_file.erase();
      goto boot;
    }

    /* Thirdly verify via magic number if this application is intented for
     * MKR WIFI 1010 or NANO 33 IOT.
     */
#if defined(ARDUINO_SAMD_MKRWIFI1010)
    if (ota_header.header.magic_number != 0x23418054) /* 2341:8054 = VID/PID MKR WIFI 1010 */
    {
      update_file.close();
      update_file.erase();
      goto boot;
    }
#elif defined(ARDUINO_SAMD_NANO_33_IOT)
    if (ota_header.header.magic_number != 0x23418057) /* 2341:8057 = VID/PID NANO 33 IOT */
    {
      update_file.close();
      update_file.erase();
      goto boot;
    }
#else
    update_file.close();
    update_file.erase();
    goto boot;
#endif

    /* Rewind to start of LZSS compressed binary. */
    update_file.seek(sizeof(ota_header.buf));

    /* Erase the complete flash starting from the SSU forward
     * because we've got no possibility of knowing how large
     * the decompressed binary will finally be.
     */
    flash.erase((void*)SKETCH_START, 0x40000 - (uint32_t)SKETCH_START);
    /* Initialize the lzss module with the data which
     * it requires.
     */
    uint32_t const LZSS_FILE_SIZE = update_file.size() - sizeof(ota_header.buf);
    lzss_init(&update_file, (uint32_t)SKETCH_START, LZSS_FILE_SIZE);
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
