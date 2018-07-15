#ifndef CommunitationUtils_h
#define CommunitationUtils_h

#include "Arduino.h"

#if defined(__SAMD21G18A__) || defined(__SAMR21G18A__) || defined(_VARIANT_ATSAMR21E18A_)
    // Output over native USB port instead (Arduino Zero, SAM D21/R21 chips)
    #define Serial SERIAL_PORT_USBVIRTUAL
#endif

void serialPrintFloatArr(float * arr, int length);
void serialFloatPrint(float f);
void writeArr(void * arr, uint8_t arr_length, uint8_t type_bytes);
void writeVar(void * val, uint8_t type_bytes);


#endif // CommunitationUtils_h