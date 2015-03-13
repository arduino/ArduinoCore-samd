/*
  Copyright (c) 2015 Arduino.  All right reserved.

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

#ifndef _WIRING_TONE_
#define _WIRING_TONE_

#ifdef __cplusplus

#include "Arduino.h"

void tone(uint32_t _pin, uint32_t frequency, uint32_t duration = 0);
void noTone(uint32_t _pin);

void toneAccurateClock (uint32_t);

#endif /*  __cplusplus */

#endif /* _WIRING_TONE_ */
