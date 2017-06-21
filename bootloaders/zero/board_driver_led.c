/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.
  Copyright (c) 2017 MattairTech LLC. All right reserved.

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

#include "board_driver_led.h"

#define LED_TARGET_VALUE_MIN_1	5
#define LED_TARGET_VALUE_MIN_2	70
#define LED_TARGET_VALUE_MAX	240

volatile uint8_t ledKeepValue = 0;
volatile uint8_t ledTargetValue = 20;
volatile int8_t ledDirection = 8;
volatile uint8_t ledTargetValueMin = LED_TARGET_VALUE_MIN_1;

inline void LED_pulse(void)
{
  if (ledKeepValue == 0) {
    ledTargetValue += ledDirection;
    LED_toggle();
  }
  ledKeepValue ++;

  if (ledTargetValue > LED_TARGET_VALUE_MAX) {
    ledDirection = -ledDirection;
    ledTargetValue += ledDirection;
  } else if ((ledTargetValue < ledTargetValueMin) && ledDirection < 0) {
    if (ledTargetValueMin == LED_TARGET_VALUE_MIN_1) {
      ledTargetValueMin = LED_TARGET_VALUE_MIN_2;
    } else {
      ledTargetValueMin = LED_TARGET_VALUE_MIN_1;
    }
    ledDirection = -ledDirection;
    ledTargetValue += ledDirection;
  }

  if (ledKeepValue == ledTargetValue) {
    LED_toggle();
  }
}

void LED_status(uint32_t periodMS)
{
  __disable_irq();

  while (1) {
    LED_toggle();
    delayUs(periodMS * 500UL);
  }
}
