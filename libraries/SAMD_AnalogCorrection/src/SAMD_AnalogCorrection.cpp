/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  SAMD51 support added by Adafruit - Copyright (c) 2018 Dean Miller for Adafruit Industries

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

#include "SAMD_AnalogCorrection.h"

void analogReadCorrection (int offset, uint16_t gain)
{
  Adc *adc;
#if defined (__SAMD51__)
adc = ADC0;
#else
adc = ADC;
#endif
  // Set correction values
  adc->OFFSETCORR.reg = ADC_OFFSETCORR_OFFSETCORR(offset);
  adc->GAINCORR.reg = ADC_GAINCORR_GAINCORR(gain);

  // Enable digital correction logic
  adc->CTRLB.bit.CORREN = 1;

#if defined (__SAMD51__)
  while(adc->SYNCBUSY.bit.OFFSETCORR || adc->SYNCBUSY.bit.GAINCORR);
#else
  while(adc->STATUS.bit.SYNCBUSY);
#endif
}

