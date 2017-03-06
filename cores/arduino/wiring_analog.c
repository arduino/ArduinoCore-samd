/*
  Copyright (c) 2014 Arduino LLC.  All right reserved.

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
#include "variant.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Mapping of timer numbers (array index) to generic clock IDs
 * GCM_* values are defined in WVariant.h in the core.
 */
const uint8_t timerClockIDs[] = 
{
#if (SAMD11)
	GCM_TCC0,
	GCM_TC1_TC2,
	GCM_TC1_TC2,
#elif (SAMD21)
	GCM_TCC0_TCC1,
	GCM_TCC0_TCC1,
	GCM_TCC2_TC3,
	GCM_TCC2_TC3,
	GCM_TC4_TC5,
	GCM_TC4_TC5,
	#if (SAMD21J)
	GCM_TC6_TC7,
	GCM_TC6_TC7,
	#endif
#elif (SAML21 || SAMC21)
	GCM_TCC0_TCC1,
	GCM_TCC0_TCC1,
	GCM_TCC2,
	GCM_TC0_TC1,
	GCM_TC0_TC1,
	GCM_TC2_TC3,
	GCM_TC2_TC3,
	GCM_TC4,
#else
#error "wiring_analog.c: Unsupported chip"
#endif
};

static int _readResolution = 10;
static int _ADCResolution = 10;
static int _writeResolution = 8;

// Wait for synchronization of registers between the clock domains
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
#if (SAMD)
  while ( ADC->STATUS.bit.SYNCBUSY == 1 );
#elif (SAML21)
  while ( ADC->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );
#elif (SAMC21)
  while ( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );
  while ( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );
#endif
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncDAC() __attribute__((always_inline, unused));
static void syncDAC() {
#if (SAMD)
  while ( DAC->STATUS.bit.SYNCBUSY == 1 );
#elif (SAML21 || SAMC21)
  while ( DAC->SYNCBUSY.reg & DAC_SYNCBUSY_MASK );
#endif
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTC_8(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_8(Tc* TCx) {
#if (SAMD)
  while (TCx->COUNT8.STATUS.bit.SYNCBUSY);
#elif (SAML21 || SAMC21)
  while (TCx->SYNCBUSY.reg);
#endif
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}

void analogReadResolution( int res )
{
  _readResolution = res ;
  if (res > 10)
  {
#if (SAMD)
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
#elif (SAML21)
    ADC->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_12BIT_Val;
#elif (SAMC21)
    ADC0->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_12BIT_Val;
    ADC1->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_12BIT_Val;
#endif
    _ADCResolution = 12;
  }
  else if (res > 8)
  {
#if (SAMD)
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;
#elif (SAML21)
    ADC->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_10BIT_Val;
#elif (SAMC21)
    ADC0->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_10BIT_Val;
    ADC1->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_10BIT_Val;
#endif
    _ADCResolution = 10;
  }
  else
  {
#if (SAMD)
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_8BIT_Val;
#elif (SAML21)
    ADC->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_8BIT_Val;
#elif (SAMC21)
    ADC0->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_8BIT_Val;
    ADC1->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_8BIT_Val;
#endif
    _ADCResolution = 8;
  }
  syncADC();
}

void analogWriteResolution( int res )
{
  _writeResolution = res ;
}

static inline uint32_t mapResolution( uint32_t value, uint32_t from, uint32_t to )
{
  if ( from == to )
  {
    return value ;
  }

  if ( from > to )
  {
    return value >> (from-to) ;
  }
  else
  {
    return value << (to-from) ;
  }
}

/*
 * Default Internal Reference for the SAMD is at 1.65V (with ADC gain of 1/2)
 * External Reference for the SAMD should be between 1v and VDDANA-0.6v=2.7v
 *
 * Warning : The maximum IO voltage is Vcc (for SAMD/SAML Vcc can be up to 3.6 volts)
 */
void analogReference( eAnalogReference ulMode )
{
  syncADC();

  if (ulMode == AR_EXTERNAL_REFA) {
    if ( pinPeripheral(REFA_PIN, PIO_ANALOG_REF) != RET_STATUS_OK ) {
        return;
    }
  }

#if (SAMD || SAML21)
  if (ulMode == AR_EXTERNAL_REFB) {
    if ( pinPeripheral(REFB_PIN, PIO_ANALOG_REF) != RET_STATUS_OK ) {
        return;
    }
  }
#endif

#if (SAMD)
  if (ulMode == AR_DEFAULT) {
    ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;    // GAIN_DIV2 allows values up to VDDANA on the pin
    ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // 1/2 VDDANA
  } else {
    ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;
    ADC->REFCTRL.bit.REFSEL = ulMode;
  }
#elif (SAML21 || SAMC21)
  if (ulMode == 0) {		// Set to 1.0V for the SAML, 1.024V for the SAMC
    SUPC->VREF.reg &= ~SUPC_VREF_SEL_Msk;
  } else if (ulMode > 5) {		// Values above 5 are used for the Supply Controller reference (AR_INTREF)
    SUPC->VREF.reg &= ~SUPC_VREF_SEL_Msk;
    SUPC->VREF.reg |= SUPC_VREF_SEL(ulMode - 6);	// 
    ulMode = 0;
  }
  #if (SAML21)
  ADC->REFCTRL.bit.REFSEL = ulMode;
  #elif (SAMC21)
  ADC0->REFCTRL.bit.REFSEL = ulMode;
  ADC1->REFCTRL.bit.REFSEL = ulMode;
  #endif
#endif

  syncADC();
}

uint32_t analogRead( uint32_t ulPin )
{
  uint32_t valueRead = 0;

  REMAP_ANALOG_PIN_ID ;

#if (SAMC21)
  Adc* ADC;
  if ( (g_APinDescription[ulPin].ulPeripheralAttribute & PER_ATTR_ADC_MASK) == PER_ATTR_ADC_STD ) {
    ADC = ADC0;
  } else {
    ADC = ADC1;
  }
#endif

  if ( pinPeripheral(ulPin, PIO_ANALOG_ADC) == RET_STATUS_OK )
  {
    // Disable DAC, if analogWrite(A0,dval) used previously the DAC is enabled
    if ( (g_APinDescription[ulPin].ulPinAttribute & PIN_ATTR_DAC) == PIN_ATTR_DAC )
    {
      syncDAC();
#if (SAMD || SAMC21)
      DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC
      //DAC->CTRLB.bit.EOEN = 0x00; // The DAC output is turned off.
#elif (SAML21)
      DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC controller
      uint8_t DACNumber = 0x00;
      if ( (g_APinDescription[ulPin].ulPort == 0) && (g_APinDescription[ulPin].ulPin == 5) ) {
        DACNumber = 0x01;
      }
      syncDAC();
      DAC->DACCTRL[DACNumber].bit.ENABLE = 0x00; // The DACx output is turned off.
      syncDAC();
      DAC->CTRLA.bit.ENABLE = 0x01;     // Enable DAC controller (in case other DACx is in use)
#endif
      syncDAC();
    }

    syncADC();
    ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ulPin].ulADCChannelNumber; // Selection for the positive ADC input

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
    ADC->INTFLAG.bit.RESRDY = 1;

    // Start conversion again, since The first conversion after the reference is changed must not be used.
    syncADC();
    ADC->SWTRIG.bit.START = 1;

    // Store the value
    while ( ADC->INTFLAG.bit.RESRDY == 0 );   // Waiting for conversion to complete
    valueRead = ADC->RESULT.reg;

    syncADC();
    ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
    syncADC();
  }

  return mapResolution(valueRead, _ADCResolution, _readResolution);
}


// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.
void analogWrite( uint32_t ulPin, uint32_t ulValue )
{
  if ( pinPeripheral(ulPin, PIO_ANALOG_DAC) == RET_STATUS_OK )
  {
    syncDAC();
#if (SAMD || SAMC21)
    ulValue = mapResolution(ulValue, _writeResolution, 10);
    DAC->DATA.reg = ulValue & 0x3FF;  // DAC on 10 bits.
    syncDAC();
    DAC->CTRLA.bit.ENABLE = 0x01;     // Enable DAC
#elif (SAML21)
    DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC controller (so that DACCTRL can be modified)
    uint8_t DACNumber = 0x00;
    if ( (g_APinDescription[ulPin].ulPort == 0) && (g_APinDescription[ulPin].ulPin == 5) ) {
        DACNumber = 0x01;
    }
    ulValue = mapResolution(ulValue, _writeResolution, 12);
    syncDAC();
    DAC->DATA[DACNumber].reg = ulValue & 0xFFF;  // DACx on 12 bits.
    syncDAC();
    DAC->DACCTRL[DACNumber].bit.ENABLE = 0x01; // The DACx output is turned on.
    syncDAC();
    while ( (DAC->STATUS.reg & (1 << DACNumber)) == 0 );   // Must wait for DACx to start
    DAC->CTRLA.bit.ENABLE = 0x01;     // Enable DAC controller
#endif
    syncDAC();
    return ;
  }
  else if ( pinPeripheral(ulPin, PIO_TIMER_PWM) == RET_STATUS_OK )
  {
    Tc*  TCx  = 0 ;
    Tcc* TCCx = 0 ;
    uint8_t Channelx = GetTCChannelNumber( g_APinDescription[ulPin].ulTCChannel ) ;
    uint8_t Timerx = GetTCNumber( g_APinDescription[ulPin].ulTCChannel ) ;
    uint8_t Typex = GetTCType( g_APinDescription[ulPin].ulTCChannel ) ;
    
    if ( Typex == 1 ) {
      TCx = (Tc*) GetTC( g_APinDescription[ulPin].ulTCChannel ) ;
    } else {
      TCCx = (Tcc*) GetTC( g_APinDescription[ulPin].ulTCChannel ) ;
    }

    // Enable peripheral clock
    if ( TCx ) {
#if (SAML21)
      if (TCx == TC4) {
        timerIndex = 7;	// TC4 is on a different lower-power bridge on the SAML
      } else {
        timerIndex = (uint8_t)(((uint32_t)TCx - (uint32_t)TCC0) >> 10);
      }
#else
      timerIndex = (uint8_t)(((uint32_t)TCx - (uint32_t)TCC0) >> 10);
#endif
    } else {
      timerIndex = (uint8_t)(((uint32_t)TCCx - (uint32_t)TCC0) >> 10);
    }

#if (SAMD)
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( timerClockIDs[timerIndex] )) ;
    while ( GCLK->STATUS.bit.SYNCBUSY == 1 ) ;
#elif (SAML21 || SAMC21)
    GCLK->PCHCTRL[timerClockIDs[timerIndex]].reg = (GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0 );
    while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK );
#endif

    ulValue = mapResolution(ulValue, _writeResolution, 8);

    // Set PORT
    if ( TCx )
    {
      // -- Configure TC
      // Disable TCx
      TCx->COUNT8.CTRLA.reg &= ~TC_CTRLA_ENABLE;
      syncTC_8(TCx);
      // Set Timer counter Mode to 8 bits
      TCx->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8;
      // Set TCx as normal PWM
#if (SAMD)
      TCx->COUNT8.CTRLA.reg |= TC_CTRLA_WAVEGEN_NPWM;
#elif (SAML21 || SAMC21)
      TCx->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;
#endif
      syncTC_8(TCx);
      // Set TCx in waveform mode Normal PWM
      TCx->COUNT8.CC[Channelx].reg = (uint8_t) ulValue;
      syncTC_8(TCx);
      // Set PER to maximum counter value (resolution : 0xFF)
      TCx->COUNT8.PER.reg = 0xFF;
      syncTC_8(TCx);
      // Enable TCx
      TCx->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
      syncTC_8(TCx);
    }
    else
    {
      // -- Configure TCC
      // Disable TCCx
      TCCx->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
      syncTCC(TCCx);
      // Set TCx as normal PWM
      TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
      syncTCC(TCCx);
      // Set TCx in waveform mode Normal PWM
      TCCx->CC[Channelx].reg = (uint32_t)ulValue;
      syncTCC(TCCx);
      // Set PER to maximum counter value (resolution : 0xFF)
      TCCx->PER.reg = 0xFF;
      syncTCC(TCCx);
      // Enable TCCx
      TCCx->CTRLA.reg |= TCC_CTRLA_ENABLE ;
      syncTCC(TCCx);
    }
  }

  // -- Defaults to digital write
  else if ( pinPeripheral(ulPin, PIO_OUTPUT) == RET_STATUS_OK )
  {
    ulValue = mapResolution(ulValue, _writeResolution, 8);
    if ( ulValue < 128 )
    {
      digitalWrite( ulPin, LOW ) ;
    }
    else
    {
      digitalWrite( ulPin, HIGH ) ;
    }
  }
}

#ifdef __cplusplus
}
#endif
