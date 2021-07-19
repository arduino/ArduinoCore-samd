#ifndef SSU_LZSS_H_
#define SSU_LZSS_H_

/**************************************************************************************
   INCLUDE
 **************************************************************************************/

#include <stdint.h>

#include <WiFiNINA.h>

/**************************************************************************************
   FUNCTION DEFINITION
 **************************************************************************************/

void lzss_init(WiFiStorageFile * update_file_ptr, uint32_t const sketch_start, uint32_t const lzss_file_size);
void lzss_decode();
void lzss_flush();

#endif /* SSU_LZSS_H_ */
