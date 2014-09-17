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

#ifndef _DELAY_
#define _DELAY_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "variant.h"

/**
 * \brief Returns the number of milliseconds since the Arduino board began running the current program.
 *
 * This number will overflow (go back to zero), after approximately 50 days.
 *
 * \return Number of milliseconds since the program started (uint32_t)
 */
extern uint32_t millis( void ) ;

/**
 * \brief Returns the number of microseconds since the Arduino board began running the current program.
 *
 * This number will overflow (go back to zero), after approximately 70 minutes. On 16 MHz Arduino boards
 * (e.g. Duemilanove and Nano), this function has a resolution of four microseconds (i.e. the value returned is
 * always a multiple of four). On 8 MHz Arduino boards (e.g. the LilyPad), this function has a resolution
 * of eight microseconds.
 *
 * \note There are 1,000 microseconds in a millisecond and 1,000,000 microseconds in a second.
 */
extern uint32_t micros( void ) ;

/**
 * \brief Pauses the program for the amount of time (in miliseconds) specified as parameter.
 * (There are 1000 milliseconds in a second.)
 *
 * \param dwMs the number of milliseconds to pause (uint32_t)
 */
extern void delay( uint32_t dwMs ) ;

/**
 * \brief Pauses the program for the amount of time (in microseconds) specified as parameter.
 *
 * \param dwUs the number of microseconds to pause (uint32_t)
 */
static __inline__ void delayMicroseconds( uint32_t ) __attribute__((always_inline, unused)) ;
static __inline__ void delayMicroseconds( uint32_t ul_usec )
{
  if ( ul_usec == 0 )
  {
    return ;
  }

  uint32_t ul = ul_usec * (VARIANT_MCK / 3000000); // = ul_usec * 16 with VARIANT_MCK @ 48MHz

/* following 'for' loop will generate this:
 * loop:
 * subs r3, #1 // 1 Core cycle
 * bne.n loop // 1 or 2 Core cycles, depends on pipeline break (50/50?)

  for ( ; ul != 0 ; ul-- )
  {
    __asm__ volatile("");
  }
*/

/*
 * __asm__ [__volatile__] ( AssemblerTemplate : [OutputOperands] [ : [InputOperands] [ : [Clobbers] ] ] )
 *
 * __asm__ documentation for GCC: https://gcc.gnu.org/onlinedocs/gcc/Extended-Asm.html
 * __volatile__ documentation for GCC: https://gcc.gnu.org/onlinedocs/gcc/Extended-Asm.html#Volatile
 *
 */
  __asm__ __volatile__( "1:      \n"
                        "sub %0, #1              \n" /* substract 1 from %0 (ul) */
                        "bne 1b   \n"
                        : /* no outputs */
                        : "r" (ul) /* 'ul' variable is '%0', '+' means R/W constraint on this variable/register, 'r' means 'register' (versus 'm' for 'memory' */
                        : /* no clobber, ie nothing should be damaged by this asm code */
                      ) ;
}

#ifdef __cplusplus
}
#endif

#endif /* _DELAY_ */
