/*
  Arduino.h - Main include file for the Arduino SDK
  Copyright (c) 2014 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef Arduino_h
#define Arduino_h

#include "api/ArduinoAPI.h"

#define RAMSTART (HMCRAMC0_ADDR)
#define RAMSIZE  (HMCRAMC0_SIZE)
#define RAMEND   (RAMSTART + RAMSIZE - 1)

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

// Include Atmel headers
#undef LITTLE_ENDIAN
#include <samd.h>

#define clockCyclesPerMicrosecond() ( SystemCoreClock / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (SystemCoreClock / 1000L) )
#define microsecondsToClockCycles(a) ( (a) * (SystemCoreClock / 1000000L) )

#include "WVariant.h"

#ifdef __cplusplus
} // extern "C"
#endif

// Include board variant
#include "variant.h"

#define interrupts()    __enable_irq()
#define noInterrupts()  __disable_irq()

#if (ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10606)
// Interrupts
#define digitalPinToInterrupt(P)   ( P )
#endif

// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif // abs

#define abs(x) ((x)>0?(x):-(x))

// Allows publishing the Beta core under samd-beta / arduino organization
#ifndef ARDUINO_ARCH_SAMD
#define ARDUINO_ARCH_SAMD
#endif

#ifdef __cplusplus
extern "C" {
#endif
/*
 * \brief SAMD products have only one reference for ADC
 */
typedef enum _eAnalogReference
{
  AR_DEFAULT,
  AR_INTERNAL,
  AR_EXTERNAL,
  AR_INTERNAL1V0,
  AR_INTERNAL1V65,
  AR_INTERNAL2V23
} eAnalogReference ;

/*
 * \brief Set the resolution of analogRead return values. Default is 10 bits (range from 0 to 1023).
 *
 * \param res
 */
extern void analogReadResolution(int res);

/*
 * \brief Set the resolution of analogWrite parameters. Default is 8 bits (range from 0 to 255).
 *
 * \param res
 */
extern void analogWriteResolution(int res);

extern void analogOutputInit( void ) ;

#ifdef __cplusplus
}
#endif

// USB Device
#include "USB/USBDesc.h"
#include "USB/USBCore.h"
#include "USB/USBAPI.h"
#include "USB/USB_host.h"

// ARM toolchain doesn't provide itoa etc, provide them
#include "api/itoa.h"

#endif // Arduino_h
