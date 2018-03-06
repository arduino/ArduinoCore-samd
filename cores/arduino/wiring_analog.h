/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

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

#pragma once

#include <stdint.h>
#include "sam.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * \brief Analog reference selection.
 * For eAnalogReference values <= 5, the value is written to the REFSEL register.
 * For values > 5 (SAML and SAMC only), 0 is written into the REFSEL register, and
 * the SUPC_VREF_SEL (supply controller) register value is: (eAnalogReference - 6).
 */
typedef enum _eAnalogReference
{
#if (SAMD)
  AR_INTERNAL1V0 = 0,
  AR_INTERNAL_INTVCC0 = 1,
  AR_INTERNAL_INTVCC1 = 2,
  AR_EXTERNAL_REFA = 3,
  AR_EXTERNAL_REFB = 4,
  AR_DEFAULT = 5,	// On the SAMD, this also uses 1/2 gain on each input
#elif (SAML21)
  AR_INTREF = 0,                // This has the same effect as AR_INTREF_1V0
  AR_INTERNAL_INTVCC0 = 1,
  AR_INTERNAL_INTVCC1 = 2,
  AR_EXTERNAL_REFA = 3,
  AR_EXTERNAL_REFB = 4,
  AR_INTERNAL_INTVCC2 = 5,
  AR_INTREF_1V0 = 6,
  AR_INTREF_1V1 = 7,
  AR_INTREF_1V2 = 8,
  AR_INTREF_1V25 = 9,
  AR_INTREF_2V0 = 10,
  AR_INTREF_2V2 = 11,
  AR_INTREF_2V4 = 12,
  AR_INTREF_2V5 = 13,
  AR_DEFAULT = AR_INTERNAL_INTVCC2,
  AR_INTERNAL1V0 = AR_INTREF,
#elif (SAMC21)
  AR_INTREF = 0,                // This has the same effect as AR_INTREF_1V024
  AR_INTERNAL_INTVCC0 = 1,
  AR_INTERNAL_INTVCC1 = 2,
  AR_EXTERNAL_REFA = 3,
  AR_EXTERNAL_DAC = 4,
  AR_INTERNAL_INTVCC2 = 5,
  AR_INTREF_1V024 = 6,
  AR_INTREF_2V048 = 7,
  AR_INTREF_4V096 = 8,
  AR_DEFAULT = AR_INTERNAL_INTVCC2,
  AR_INTERNAL1V0 = AR_INTREF,
#else
  #error "wiring_analog.c: Unsupported chip"
#endif
  AR_INTERNAL = AR_INTERNAL_INTVCC0,
  AR_INTERNAL2V23 = AR_INTERNAL_INTVCC0,	// 2.23V only when Vcc = 3.3V
  AR_INTERNAL1V65 = AR_INTERNAL_INTVCC1,	// 1.65V only when Vcc = 3.3V
  AR_EXTERNAL = AR_EXTERNAL_REFA,
} eAnalogReference ;


/*
 * \brief Configures the reference voltage used for analog input (i.e. the value used as the top of the input range).
 * This function is kept only for compatibility with existing AVR based API.
 *
 * \param ulMmode Should be set to AR_DEFAULT.
 */
extern void analogReference( eAnalogReference ulMode ) ;

/*
 * \brief Writes an analog value (PWM wave) to a pin.
 *
 * \param ulPin
 * \param ulValue
 */
extern void analogWrite( uint32_t ulPin, uint32_t ulValue ) ;

extern void analogWritePeriod(uint32_t pin, uint32_t value, uint32_t period) ;

/*
 * \brief Reads the value from the specified analog pin.
 *
 * \param ulPin
 *
 * \return Read value from selected pin, if no error.
 */
extern uint32_t analogRead( uint32_t ulPin ) ;

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
