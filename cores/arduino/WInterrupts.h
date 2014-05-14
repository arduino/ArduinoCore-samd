/*
  Copyright (c) 2014 Arduino.  All right reserved.

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

#ifndef _WIRING_INTERRUPTS_
#define _WIRING_INTERRUPTS_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//      LOW 0
//      HIGH 1
#define CHANGE 2
#define FALLING 3
#define RISING 4

///*
 //* Interrupt modes
 //*   The two first values are conflicting with the ones used by Digital API, so we use another name for each.
 //*/
//typedef enum _EExt_IntMode
//{
  //IM_LOW = 0,
  //IM_HIGH = 1,
  //CHANGE = 2,
  //FALLING = 3,
  //RISING = 4,
  //IM_CHANGE = 2,
  //IM_FALLING = 3,
  //IM_RISING = 4,
//} EExt_IntMode ;

#define DEFAULT 1
#define EXTERNAL 0

typedef void (*voidFuncPtr)( void ) ;

/*
 * \brief Specifies a named Interrupt Service Routine (ISR) to call when an interrupt occurs.
 *        Replaces any previous function that was attached to the interrupt.
 */
//void attachInterrupt( uint32_t ulPin, void (*callback)(void), EExt_IntMode mode ) ;
//void attachInterrupt( uint32_t ulPin, voidFuncPtr callback, EExt_IntMode mode ) ;
void attachInterrupt( uint32_t ulPin, voidFuncPtr callback, uint32_t mode ) ;

/*
 * \brief Turns off the given interrupt.
 */
void detachInterrupt( uint32_t ulPin ) ;

#ifdef __cplusplus
}
#endif

//#ifdef __cplusplus
//inline operator ::EExt_IntMode( uint32_t ul )
//{
  //return (EExt_IntMode)ul ;
//}
//#endif

#endif /* _WIRING_INTERRUPTS_ */
