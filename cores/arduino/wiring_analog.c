/*
  Copyright (c) 2014 Arduino LLC.  All right reserved.
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

#include "Arduino.h"
#include "wiring_private.h"

#ifdef __cplusplus
extern "C" {
#endif

static int _readResolution = 10;
static int _ADCResolution = 10;

#if defined(__SAMD51__)
static int _writeResolution = 12;
static int _dacResolution = 12;
#else
static int _writeResolution = 8;
static int _dacResolution = 10;
#endif


#if !defined(__SAMD51__)
// Wait for synchronization of registers between the clock domains
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;
}

 // ATSAMR, for example, doesn't have a DAC
#ifdef DAC
// Wait for synchronization of registers between the clock domains
static __inline__ void syncDAC() __attribute__((always_inline, unused));
static void syncDAC() {
  while (DAC->STATUS.bit.SYNCBUSY == 1)
    ;
}
#endif

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTC_16(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_16(Tc* TCx) {
  while (TCx->COUNT16.STATUS.bit.SYNCBUSY);
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}

#else
static bool dacEnabled[2];
#endif

void analogReadResolution(int res)
{
  _readResolution = res;
#if defined(__SAMD51__)

	if (res > 10) {
		ADC0->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
		ADC1->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
		_ADCResolution = 12;
		} else if (res > 8) {
		ADC0->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;
		ADC1->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;
		_ADCResolution = 10;
		} else {
		ADC0->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_8BIT_Val;
		ADC1->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_8BIT_Val;
		_ADCResolution = 8;
	}


	while(ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB); //wait for sync
	while(ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB); //wait for sync
#else

	if (res > 10) {
		ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
		_ADCResolution = 12;
		} else if (res > 8) {
		ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;
		_ADCResolution = 10;
		} else {
		ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_8BIT_Val;
		_ADCResolution = 8;
	}

  syncADC();
#endif
}

void analogWriteResolution(int res)
{
  _writeResolution = res;
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
  if (from == to) {
    return value;
  }
  if (from > to) {
    return value >> (from-to);
  }
  return value << (to-from);
}

/*
 * Internal Reference is at 1.0v
 * External Reference should be between 1v and VDDANA-0.6v=2.7v
 *
 * Warning : On Arduino Zero board the input/output voltage for SAMD21G18 is 3.3 volts maximum
 */
void analogReference(eAnalogReference mode)
{
#if defined(__SAMD51__)
	while(ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_REFCTRL); //wait for sync
	while(ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_REFCTRL); //wait for sync
	
	//TODO: fix gains
	switch (mode)
	{
		case AR_INTERNAL:
		case AR_INTERNAL2V23:
		//ADC0->GAINCORR.reg = ADC_GAINCORR_GAINCORR();      // Gain Factor Selection
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val; // 1/1.48 VDDANA = 1/1.48* 3V3 = 2.2297
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val; // 1/1.48 VDDANA = 1/1.48* 3V3 = 2.2297
		break;

		case AR_EXTERNAL:
		//ADC0->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val;
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val;
		break;

/*		Don't think this works on SAMD51
		case AR_INTERNAL1V0:
		//ADC0->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INT1V_Val;   // 1.0V voltage reference
		break;
*/

		case AR_INTERNAL1V65:
		//ADC0->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // 1/2 VDDANA = 0.5* 3V3 = 1.65V
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // 1/2 VDDANA = 0.5* 3V3 = 1.65V
		break;

		case AR_DEFAULT:
		default:
		//ADC0->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // 1/2 VDDANA = 0.5* 3V3 = 1.65V
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // 1/2 VDDANA = 0.5* 3V3 = 1.65V
		
		break;
	}
	
#else
  syncADC();
  switch (mode)
  {
    case AR_INTERNAL:
    case AR_INTERNAL2V23:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val; // 1/1.48 VDDANA = 1/1.48* 3V3 = 2.2297
      break;

    case AR_EXTERNAL:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val;
      break;

    case AR_INTERNAL1V0:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INT1V_Val;   // 1.0V voltage reference
      break;

    case AR_INTERNAL1V65:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // 1/2 VDDANA = 0.5* 3V3 = 1.65V
      break;

    case AR_DEFAULT:
    default:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // 1/2 VDDANA = 0.5* 3V3 = 1.65V
      break;
  }
#endif
}

uint32_t analogRead(uint32_t pin)
{
  uint32_t valueRead = 0;

#if defined(PIN_A6)
  if (pin == 6) {
    pin = PIN_A6;
  } else
#endif
#if defined(PIN_A7)
  if (pin == 7) {
    pin = PIN_A7;
  } else 
#endif
  if (pin <= 5) {
    pin += A0;
  }

  pinPeripheral(pin, PIO_ANALOG);
 //ATSAMR, for example, doesn't have a DAC
#ifdef DAC

	#if defined(__SAMD51__)
	  if (pin == A0 || pin == A1) { // Disable DAC, if analogWrite(A0,dval) used previously the DAC is enabled
		uint8_t channel = (pin == PIN_A0 ? 0 : 1);
		
		if(dacEnabled[channel]){
			dacEnabled[channel] = false;
			
			while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
			DAC->CTRLA.bit.ENABLE = 0;     // disable DAC
			
			while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
			DAC->DACCTRL[channel].bit.ENABLE = 0;
			
			while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
			DAC->CTRLA.bit.ENABLE = 1;     // enable DAC
		}
		
		while (DAC->SYNCBUSY.bit.ENABLE);
	#else
	  if (pin == A0) { // Disable DAC, if analogWrite(A0,dval) used previously the DAC is enabled
	    syncDAC();
		
		DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC
		//DAC->CTRLB.bit.EOEN = 0x00; // The DAC output is turned off.
		syncDAC();
	#endif
	  }

#endif

#if defined(__SAMD51__)
  Adc *adc;
  if(g_APinDescription[pin].ulPinAttribute & PIN_ATTR_ANALOG) adc = ADC0;
  else if(g_APinDescription[pin].ulPinAttribute & PIN_ATTR_ANALOG_ALT) adc = ADC1;
  else return 0;

  while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL ); //wait for sync
  adc->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber; // Selection for the positive ADC input
  
  // Control A
  /*
   * Bit 1 ENABLE: Enable
   *   0: The ADC is disabled.
   *   1: The ADC is enabled.
   * Due to synchronization, there is a delay from writing CTRLA.ENABLE until the peripheral is enabled/disabled. The
   * value written to CTRL.ENABLE will read back immediately and the Synchronization Busy bit in the Status register
   * (STATUS.SYNCBUSY) will be set. STATUS.SYNCBUSY will be cleared when the operation is complete.
   *
   * Before enabling the ADC, the asynchronous clock source must be selected and enabled, and the ADC reference must be
   * configured. The first conversion after the reference is changed must not be used.
   */
  while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
  adc->CTRLA.bit.ENABLE = 0x01;             // Enable ADC

  // Start conversion
  while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
  
  adc->SWTRIG.bit.START = 1;

  // Clear the Data Ready flag
  adc->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  // Start conversion again, since The first conversion after the reference is changed must not be used.
  adc->SWTRIG.bit.START = 1;

  // Store the value
  while (adc->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
  valueRead = adc->RESULT.reg;

  while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
  adc->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
  
#else
  syncADC();
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber; // Selection for the positive ADC input
  
  // Control A
  /*
   * Bit 1 ENABLE: Enable
   *   0: The ADC is disabled.
   *   1: The ADC is enabled.
   * Due to synchronization, there is a delay from writing CTRLA.ENABLE until the peripheral is enabled/disabled. The
   * value written to CTRL.ENABLE will read back immediately and the Synchronization Busy bit in the Status register
   * (STATUS.SYNCBUSY) will be set. STATUS.SYNCBUSY will be cleared when the operation is complete.
   *
   * Before enabling the ADC, the asynchronous clock source must be selected and enabled, and the ADC reference must be
   * configured. The first conversion after the reference is changed must not be used.
   */
  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC

  // Start conversion
  syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Clear the Data Ready flag
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  // Start conversion again, since The first conversion after the reference is changed must not be used.
  syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Store the value
  while (ADC->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
  valueRead = ADC->RESULT.reg;

  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  syncADC();
#endif

  return mapResolution(valueRead, _ADCResolution, _readResolution);
}


// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.
void analogWrite(uint32_t pin, uint32_t value)
{
  PinDescription pinDesc = g_APinDescription[pin];
  uint32_t attr = pinDesc.ulPinAttribute;

 // ATSAMR, for example, doesn't have a DAC
#ifdef DAC

	  if ((attr & PIN_ATTR_ANALOG) == PIN_ATTR_ANALOG)
	  {
	    // DAC handling code
#if defined(__SAMD51__)
		if (pin == PIN_A0 || pin == PIN_A1) { // 2 DACs on A0 (PA02) and A1 (PA05)
#else
	    if (pin == PIN_A0) { // Only 1 DAC on A0 (PA02)
#endif

#if defined(__SAMD51__)

	    value = mapResolution(value, _writeResolution, _dacResolution);


			uint8_t channel = (pin == PIN_A0 ? 0 : 1);

			pinPeripheral(pin, PIO_ANALOG);

			if(!dacEnabled[channel]){
				dacEnabled[channel] = true;

				while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
				DAC->CTRLA.bit.ENABLE = 0;     // disable DAC

				while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
				DAC->DACCTRL[channel].bit.ENABLE = 1;

				while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
				DAC->CTRLA.bit.ENABLE = 1;     // enable DAC

				if(channel == 0){

					while ( !DAC->STATUS.bit.READY0 );

					while (DAC->SYNCBUSY.bit.DATA0);
					DAC->DATA[0].reg = value;
				}
				else if(channel == 1){
					while ( !DAC->STATUS.bit.READY1 );

					while (DAC->SYNCBUSY.bit.DATA1);
					DAC->DATA[1].reg = value;
				}

				delayMicroseconds(10000);
			}

			//ERROR!
			while(!DAC->DACCTRL[channel].bit.ENABLE);

			if(channel == 0){

				while ( !DAC->STATUS.bit.READY0 );

				while (DAC->SYNCBUSY.bit.DATA0);
				DAC->DATA[0].reg = value;  // DAC on 10 bits.
			}
			else if(channel == 1){
				while ( !DAC->STATUS.bit.READY1 );

				while (DAC->SYNCBUSY.bit.DATA1);
				DAC->DATA[1].reg = value;  // DAC on 10 bits.
			}


#else
			syncDAC();
			DAC->DATA.reg = value & 0x3FF;  // DAC on 10 bits.
			syncDAC();
			DAC->CTRLA.bit.ENABLE = 0x01;     // Enable DAC
			syncDAC();
#endif // __SAMD51__
				return;
	  }
	}
#endif // DAC

#if defined(__SAMD51__)
	if(attr & (PIN_ATTR_PWM_E|PIN_ATTR_PWM_F|PIN_ATTR_PWM_G)){

		uint32_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);
		uint8_t tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);
		static bool tcEnabled[TCC_INST_NUM+TC_INST_NUM];

		if(attr & PIN_ATTR_PWM_E)
			pinPeripheral(pin, PIO_TIMER);
		else if(attr & PIN_ATTR_PWM_F)
			pinPeripheral(pin, PIO_TIMER_ALT);
		else if(attr & PIN_ATTR_PWM_G)
			pinPeripheral(pin, PIO_TCC_PDEC);

		if (!tcEnabled[tcNum]) {
		  tcEnabled[tcNum] = true;
	      GCLK->PCHCTRL[GCLK_CLKCTRL_IDs[tcNum]].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 0

	      // Set PORT
	      if (tcNum >= TCC_INST_NUM) {
				// -- Configure TC
				Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);

				//reset
				TCx->COUNT8.CTRLA.bit.SWRST = 1;
				while (TCx->COUNT8.SYNCBUSY.bit.SWRST);

				// Disable TCx
				TCx->COUNT8.CTRLA.bit.ENABLE = 0;
				while (TCx->COUNT8.SYNCBUSY.bit.ENABLE);
				// Set Timer counter Mode to 8 bits, normal PWM, prescaler 1/256
				TCx->COUNT8.CTRLA.reg = TC_CTRLA_MODE_COUNT8 | TC_CTRLA_PRESCALER_DIV256;
				TCx->COUNT8.WAVE.reg = TC_WAVE_WAVEGEN_NPWM;

				while (TCx->COUNT8.SYNCBUSY.bit.CC0);
				// Set the initial value
				TCx->COUNT8.CC[tcChannel].reg = (uint8_t) value;
				while (TCx->COUNT8.SYNCBUSY.bit.CC0);
				// Set PER to maximum counter value (resolution : 0xFF)
				TCx->COUNT8.PER.reg = 0xFF;
				while (TCx->COUNT8.SYNCBUSY.bit.PER);
				// Enable TCx
				TCx->COUNT8.CTRLA.bit.ENABLE = 1;
				while (TCx->COUNT8.SYNCBUSY.bit.ENABLE);
			} else {
				// -- Configure TCC
				Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);

				TCCx->CTRLA.bit.SWRST = 1;
				while (TCCx->SYNCBUSY.bit.SWRST);

				// Disable TCCx
				TCCx->CTRLA.bit.ENABLE = 0;
				while (TCCx->SYNCBUSY.bit.ENABLE);
				// Set prescaler to 1/256
				TCCx->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV256 | TCC_CTRLA_PRESCSYNC_GCLK;

				// Set TCx as normal PWM
				TCCx->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
				while ( TCCx->SYNCBUSY.bit.WAVE );

				while (TCCx->SYNCBUSY.bit.CC0 || TCCx->SYNCBUSY.bit.CC1);
				// Set the initial value
				TCCx->CC[tcChannel].reg = (uint32_t) value;
				while (TCCx->SYNCBUSY.bit.CC0 || TCCx->SYNCBUSY.bit.CC1);
				// Set PER to maximum counter value (resolution : 0xFF)
				TCCx->PER.reg = 0xFF;
				while (TCCx->SYNCBUSY.bit.PER);
				// Enable TCCx
				TCCx->CTRLA.bit.ENABLE = 1;
				while (TCCx->SYNCBUSY.bit.ENABLE);
			}
		}
		else {
			if (tcNum >= TCC_INST_NUM) {
				Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
				TCx->COUNT8.CC[tcChannel].reg = (uint8_t) value;
				while (TCx->COUNT8.SYNCBUSY.bit.CC0 || TCx->COUNT8.SYNCBUSY.bit.CC1);
				} else {
				Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
				while (TCCx->SYNCBUSY.bit.CTRLB);
				while (TCCx->SYNCBUSY.bit.CC0 || TCCx->SYNCBUSY.bit.CC1);
				TCCx->CCBUF[tcChannel].reg = (uint32_t) value;
				while (TCCx->SYNCBUSY.bit.CC0 || TCCx->SYNCBUSY.bit.CC1);
				TCCx->CTRLBCLR.bit.LUPD = 1;
				while (TCCx->SYNCBUSY.bit.CTRLB);
				}
		}

		return;
	}
	  
#else

  if ((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM)
	  {
	  value = mapResolution(value, _writeResolution, 16);

	  uint32_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);
	  uint8_t tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);
	  static bool tcEnabled[TCC_INST_NUM+TC_INST_NUM];

	    if (attr & PIN_ATTR_TIMER) {
#if !(ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10603)
	      // Compatibility for cores based on SAMD core <=1.6.2
	      if (pinDesc.ulPinType == PIO_TIMER_ALT) {
	        pinPeripheral(pin, PIO_TIMER_ALT);
	      } else
#endif
	      {
	        pinPeripheral(pin, PIO_TIMER);
	      }
	    } else if ((attr & PIN_ATTR_TIMER_ALT) == PIN_ATTR_TIMER_ALT){
	        //this is on an alt timer
	        pinPeripheral(pin, PIO_TIMER_ALT);
	    }
	    else{
	        return;
	    }

	    if (!tcEnabled[tcNum]) {
	      tcEnabled[tcNum] = true;
		  uint16_t GCLK_CLKCTRL_IDs[] = {
			GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC0
			GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC1
			GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TCC2
			GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TC3
			GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC4
			GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC5
			GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC6
			GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC7
		  };
		  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_IDs[tcNum]);
		  while (GCLK->STATUS.bit.SYNCBUSY == 1);

		  // Set PORT
		  if (tcNum >= TCC_INST_NUM) {
			// -- Configure TC
			Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
			// Disable TCx
			TCx->COUNT16.CTRLA.bit.ENABLE = 0;
			syncTC_16(TCx);
			// Set Timer counter Mode to 16 bits, normal PWM
			TCx->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_NPWM;
			syncTC_16(TCx);
			// Set the initial value
			TCx->COUNT16.CC[tcChannel].reg = (uint32_t) value;
			syncTC_16(TCx);
			// Enable TCx
			TCx->COUNT16.CTRLA.bit.ENABLE = 1;
			syncTC_16(TCx);
		  } else {
			// -- Configure TCC
			Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
			// Disable TCCx
			TCCx->CTRLA.bit.ENABLE = 0;
			syncTCC(TCCx);
			// Set TCCx as normal PWM
			TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
			syncTCC(TCCx);
			// Set the initial value
			TCCx->CC[tcChannel].reg = (uint32_t) value;
			syncTCC(TCCx);
			// Set PER to maximum counter value (resolution : 0xFFFF)
			TCCx->PER.reg = 0xFFFF;
			syncTCC(TCCx);
			// Enable TCCx
			TCCx->CTRLA.bit.ENABLE = 1;
			syncTCC(TCCx);
		  }
		} else {
		  if (tcNum >= TCC_INST_NUM) {
			Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
			TCx->COUNT16.CC[tcChannel].reg = (uint32_t) value;
			syncTC_16(TCx);
		  } else {
			Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
			TCCx->CTRLBSET.bit.LUPD = 1;
			syncTCC(TCCx);
			TCCx->CCB[tcChannel].reg = (uint32_t) value;
			syncTCC(TCCx);
			TCCx->CTRLBCLR.bit.LUPD = 1;
			syncTCC(TCCx);
		  }
		}
	  return;
  }
#endif

  // -- Defaults to digital write
  pinMode(pin, OUTPUT);
  value = mapResolution(value, _writeResolution, 8);
  if (value < 128) {
    digitalWrite(pin, LOW);
  } else {
    digitalWrite(pin, HIGH);
  }
}

#ifdef __cplusplus
}
#endif
