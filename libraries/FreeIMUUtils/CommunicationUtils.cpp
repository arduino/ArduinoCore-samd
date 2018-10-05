#include "CommunicationUtils.h"
void serialPrintFloatArr(float * arr, int length) {
  for(int i=0; i<length; i++) {
    serialFloatPrint(arr[i]);
    Serial.print(",");
  }
}


void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  for(int i=0; i<4; i++) {
    
    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);
    
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    Serial.print(c1);
    Serial.print(c2);
  }
}


void writeArr(void * varr, uint8_t arr_length, uint8_t type_bytes) {
  byte * arr = (byte*) varr;
  for(uint8_t i=0; i<arr_length; i++) {
    writeVar(&arr[i * type_bytes], type_bytes);
  }
}


// thanks to Francesco Ferrara and the Simplo project for the following code!
void writeVar(void * val, uint8_t type_bytes) {
  byte * addr=(byte *)(val);
  for(uint8_t i=0; i<type_bytes; i++) { 
    Serial.write(addr[i]);
  }
}


// Same as serialPrintFloatArr(), but appends to a 'result' char array instead of printing to serial output.
void toPrintableFloatArr(float* arr, int length, char* result) {
  char* buffer;

  for(int i=0; i<length; i++) {
    toPrintableFloat(arr[i], &result[i*8]); // 8 bytes per hex float outpit
  }
}

// Same as serialFloatPrint(), but appends to a 'result' char array instead of printing to serial output.
void toPrintableFloat(float f, char* result) {
  byte * b = (byte *) &f;

  for(int i=0; i<4; i++) {
    
    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);
    
    *result++ = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    *result++ = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
  }
  *result = '\0';
}

// Same as writeArr(), but appends to a 'result' char array instead of printing to serial output.
// returns number of bytes written to array
int toPrintableArr(void* varr, uint8_t arr_length, uint8_t type_bytes, char* result) {
  byte * arr = (byte*) varr;
  result = &result[strlen(result)]; // append to end of result
  for(uint8_t i=0; i<arr_length; i++) {
    toPrintableVar(&arr[i * type_bytes], type_bytes, &result[i*type_bytes]);
  }
  return arr_length * type_bytes;
}

// Same as writeVar(), but appends to a 'result' char array instead of printing to serial output.
void toPrintableVar(void* val, uint8_t type_bytes, char* result) {
    memcpy((void *)result, (void *)val, type_bytes);
}
