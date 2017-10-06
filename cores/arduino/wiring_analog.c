/*
  Copyright (c) 2014 Arduino LLC.  All right reserved.
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

#include "Arduino.h"
#include "wiring_private.h"
#include "variant.h"
#include "delay.h"

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
	GCM_TC6_TC7,
	GCM_TC6_TC7,
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
extern bool dacEnabled[];

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
static __inline__ void syncTC_16(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_16(Tc* TCx) {
#if (SAMD)
  while (TCx->COUNT16.STATUS.bit.SYNCBUSY);
#elif (SAML21 || SAMC21)
  while (TCx->COUNT16.SYNCBUSY.reg & (TC_SYNCBUSY_SWRST | TC_SYNCBUSY_ENABLE | TC_SYNCBUSY_CTRLB | TC_SYNCBUSY_STATUS | TC_SYNCBUSY_COUNT));
#endif
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}

void analogReadResolution(int res)
{
  _readResolution = res;
  if (res > 10) {
#if (SAMD)
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
#elif (SAML21)
    ADC->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_12BIT_Val;
#elif (SAMC21)
    ADC0->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_12BIT_Val;
    ADC1->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_12BIT_Val;
#endif
    _ADCResolution = 12;
  } else if (res > 8) {
#if (SAMD)
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;
#elif (SAML21)
    ADC->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_10BIT_Val;
#elif (SAMC21)
    ADC0->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_10BIT_Val;
    ADC1->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_10BIT_Val;
#endif
    _ADCResolution = 10;
  } else {
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
 * Default Internal Reference for the SAMD is at 1.65V (with ADC gain of 1/2)
 * External Reference for the SAMD should be between 1v and VDDANA-0.6v=2.7v
 *
 * Warning : The maximum IO voltage is Vcc (for SAMD/SAML Vcc can be up to 3.6 volts)
 */
void analogReference(eAnalogReference mode)
{
  syncADC();
#if (!SAMD11C)
  #if defined(REFA_PIN)
    if (mode == AR_EXTERNAL_REFA) {
      if ( pinPeripheral(REFA_PIN, PIO_ANALOG_REF) != RET_STATUS_OK ) {
          return;
      }
    }
  #endif
#endif

#if (SAMD || SAML21)
  #if defined(REFB_PIN)
    if (mode == AR_EXTERNAL_REFB) {
      if ( pinPeripheral(REFB_PIN, PIO_ANALOG_REF) != RET_STATUS_OK ) {
          return;
      }
    }
  #endif
#endif

#if (SAMD)
  if (mode == AR_DEFAULT) {
    ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;    // GAIN_DIV2 allows values up to VDDANA on the pin
    ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // 1/2 VDDANA
  } else {
    ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;
    ADC->REFCTRL.bit.REFSEL = mode;	// Note that mode value can be used with the register. See header file.
  }
#elif (SAML21 || SAMC21)
  if (mode == 0) {		// Set to 1.0V for the SAML, 1.024V for the SAMC
    SUPC->VREF.reg &= ~SUPC_VREF_SEL_Msk;
  } else if (mode > 5) {		// Values above 5 are used for the Supply Controller reference (AR_INTREF)
    SUPC->VREF.reg &= ~SUPC_VREF_SEL_Msk;
    SUPC->VREF.reg |= SUPC_VREF_SEL(mode - 6);	// 
    mode = 0;
  }
  #if (SAML21)
  ADC->REFCTRL.bit.REFSEL = mode;
  #elif (SAMC21)
  ADC0->REFCTRL.bit.REFSEL = mode;
  ADC1->REFCTRL.bit.REFSEL = mode;
  #endif
#endif
  syncADC();

  // Start conversion, since The first conversion after the reference is changed must not be used.
#if (SAMC21)
  ADC0->CTRLA.bit.ENABLE = 0x01;             // Enable ADC
  ADC1->CTRLA.bit.ENABLE = 0x01;             // Enable ADC
  syncADC();
  ADC0->SWTRIG.bit.START = 1;
  ADC1->SWTRIG.bit.START = 1;
  syncADC();
  while ((ADC0->INTFLAG.bit.RESRDY == 0) || (ADC1->INTFLAG.bit.RESRDY == 0));     // Waiting for conversion to complete
  ADC0->INTFLAG.reg = ADC_INTFLAG_RESRDY;    // Clear the Data Ready flag
  ADC1->INTFLAG.reg = ADC_INTFLAG_RESRDY;    // Clear the Data Ready flag
  ADC0->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  ADC1->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
#else
  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC
  syncADC();
  ADC->SWTRIG.bit.START = 1;
  syncADC();
  while (ADC->INTFLAG.bit.RESRDY == 0);     // Waiting for conversion to complete
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;    // Clear the Data Ready flag
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
#endif
  syncADC();
}

uint32_t analogRead( uint32_t pin )
{
  uint32_t valueRead = 0;

#if defined(REMAP_ANALOG_PIN_ID)
  REMAP_ANALOG_PIN_ID(pin);
#endif

#if (SAMC21)
  Adc* ADC;
  if ( (g_APinDescription[pin].ulPeripheralAttribute & PER_ATTR_ADC_MASK) == PER_ATTR_ADC_STD ) {
    ADC = ADC0;
  } else {
    ADC = ADC1;
  }
#endif

  // pinPeripheral now handles disabling the DAC (if active)
  if ( pinPeripheral(pin, PIO_ANALOG_ADC) == RET_STATUS_OK )
  {
    ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber; // Selection for the positive ADC input
    syncADC();

    ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC
    syncADC();

    // Start conversion
    ADC->SWTRIG.bit.START = 1;
    syncADC();

    // Store the value
    while (ADC->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
    valueRead = ADC->RESULT.reg;

    ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
    syncADC();
  }

  return mapResolution(valueRead, _ADCResolution, _readResolution);
}


// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.
void analogWrite(uint32_t pin, uint32_t value)
{
  if ( pinPeripheral(pin, PIO_ANALOG_DAC) == RET_STATUS_OK )
  {
    syncDAC();
#if (SAMD || SAMC21)
    value = mapResolution(value, _writeResolution, 10);
    DAC->DATA.reg = value & 0x3FF;  // DAC on 10 bits.
    syncDAC();
    if (!dacEnabled[0]) {
      dacEnabled[0] = true;
      DAC->CTRLA.bit.ENABLE = 0x01;     // Enable DAC
      syncDAC();
    }
#elif (SAML21)
    uint8_t DACNumber = 0x00;
    value = mapResolution(value, _writeResolution, 12);

    if ( (g_APinDescription[pin].ulPort == 0) && (g_APinDescription[pin].ulPin == 5) ) {
        DACNumber = 0x01;
    }

    if (!dacEnabled[DACNumber]) {
      dacEnabled[DACNumber] = true;
      DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC controller (so that DACCTRL can be modified)
      delayMicroseconds(40);	// Must delay for at least 30us when turning off while refresh is on due to DAC errata

      DAC->DATA[DACNumber].reg = value & 0xFFF;  // DACx on 12 bits.
      syncDAC();
      DAC->DACCTRL[DACNumber].bit.ENABLE = 0x01; // The DACx output is turned on.
      DAC->CTRLA.bit.ENABLE = 0x01;     // Enable DAC controller
      syncDAC();
      while ( (DAC->STATUS.reg & (1 << DACNumber)) == 0 );   // Must wait for DACx to start

      DAC->DATA[DACNumber].reg = value & 0xFFF;  // DACx on 12 bits.
      syncDAC();
    } else {
      DAC->DATA[DACNumber].reg = value & 0xFFF;  // DACx on 12 bits.
      syncDAC();
    }
#endif
    return;
  }
  else if ( pinPeripheral(pin, PIO_TIMER_PWM) == RET_STATUS_OK )
  {
    Tc*  TCx  = 0 ;
    Tcc* TCCx = 0 ;
    uint8_t timerIndex;
    uint8_t Channelx = GetTCChannelNumber( g_APinDescription[pin].ulTCChannel ) ;
    // uint8_t Timerx = GetTCNumber( g_APinDescription[pin].ulTCChannel ) ;
    uint8_t Typex = GetTCType( g_APinDescription[pin].ulTCChannel ) ;
    static bool tcEnabled[TCC_INST_NUM+TC_INST_NUM];

    if ( Typex == 1 ) {
      TCx = (Tc*) GetTC( g_APinDescription[pin].ulTCChannel ) ;
    } else {
      TCCx = (Tcc*) GetTC( g_APinDescription[pin].ulTCChannel ) ;
    }

    // Enable peripheral clock, SAML and SAMC have TC numbers starting at 0
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

    value = mapResolution(value, _writeResolution, 16);

    if (!tcEnabled[timerIndex]) {
      tcEnabled[timerIndex] = true;

#if (SAMD)
      GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( timerClockIDs[timerIndex] )) ;
      while ( GCLK->STATUS.bit.SYNCBUSY == 1 );
#elif (SAML21 || SAMC21)
      GCLK->PCHCTRL[timerClockIDs[timerIndex]].reg = (GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0 );
      while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK );
#endif

      // Set PORT
      if ( TCx )
      {
        // -- Configure TC
        // Disable TCx
        TCx->COUNT16.CTRLA.bit.ENABLE = 0;
        syncTC_16(TCx);
        // Set Timer counter Mode to 16 bits, normal PWM
        TCx->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
        syncTC_16(TCx);
        // Set TCx as normal PWM
#if (SAMD)
        TCx->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_NPWM;
#elif (SAML21 || SAMC21)
        TCx->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_NPWM;
#endif
        syncTC_16(TCx);
        // Set the initial value
        TCx->COUNT16.CC[Channelx].reg = (uint32_t) value;
        syncTC_16(TCx);
        // Enable TCx
        TCx->COUNT16.CTRLA.bit.ENABLE = 1;
        syncTC_16(TCx);
      } else {
        // -- Configure TCC
        // Disable TCCx
        TCCx->CTRLA.bit.ENABLE = 0;
        syncTCC(TCCx);
        // Set TCCx as normal PWM
        TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
        syncTCC(TCCx);
        // Set the initial value
        TCCx->CC[Channelx].reg = (uint32_t)value;
        syncTCC(TCCx);
        // Set PER to maximum counter value (resolution : 0xFFFF)
        TCCx->PER.reg = 0xFFFF;
        syncTCC(TCCx);
        // Enable TCCx
        TCCx->CTRLA.bit.ENABLE = 1;
        syncTCC(TCCx);
      }
    } else {
      if (TCx) {
#if (SAMD)
        TCx->COUNT16.CC[Channelx].reg = (uint32_t) value;
#elif (SAML21 || SAMC21)
        // SAML and SAMC have double-buffered TCs
        TCx->COUNT16.CCBUF[Channelx].reg = (uint32_t) value;
#endif
        syncTC_16(TCx);
      } else {
#if (SAMD)
        TCCx->CTRLBSET.bit.LUPD = 1;
        syncTCC(TCCx);
	TCCx->CCB[Channelx].reg = (uint32_t) value;
	syncTCC(TCCx);
	TCCx->CTRLBCLR.bit.LUPD = 1;
// LUPD caused endless spinning in syncTCC() on SAML (and probably SAMC). Note that CCBUF writes are already
// atomic. The LUPD bit is intended for updating several registers at once, which analogWrite() does not do.
#elif (SAML21 || SAMC21)
	TCCx->CCBUF[Channelx].reg = (uint32_t) value;
#endif
        syncTCC(TCCx);
      }
    }
  }

  // -- Defaults to digital write
  else if ( pinPeripheral(pin, PIO_OUTPUT) == RET_STATUS_OK )
  {
    value = mapResolution(value, _writeResolution, 8);
    if (value < 128) {
	    digitalWrite(pin, LOW);
    } else {
	    digitalWrite(pin, HIGH);
    }
  }
}

#ifdef __cplusplus
}
#endif
