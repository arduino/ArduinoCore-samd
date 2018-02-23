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
#elif (SAMD51)
        GCM_TCC0_TCC1,
        GCM_TCC0_TCC1,
  #if (SAMD51G)
        GCM_TCC2_TCC3,
  #else
        GCM_TCC2_TCC3,
        GCM_TCC2_TCC3,
        GCM_TCC4,
  #endif
        GCM_TC0_TC1,
        GCM_TC0_TC1,
        GCM_TC2_TC3,
        GCM_TC2_TC3,
        GCM_TC4_TC5,
        GCM_TC4_TC5,
        GCM_TC6_TC7,
        GCM_TC6_TC7,
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
#if (SAMD21 || SAMD11)
  while ( ADC->STATUS.bit.SYNCBUSY == 1 );
#elif (SAML21)
  while ( ADC->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );
#elif (SAMC21 || SAMD51)
  while ( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );
  while ( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );
#endif
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncDAC() __attribute__((always_inline, unused));
static void syncDAC() {
#if (SAMD21 || SAMD11)
  while ( DAC->STATUS.bit.SYNCBUSY == 1 );
#elif (SAML21 || SAMC21 || SAMD51)
  while ( DAC->SYNCBUSY.reg & DAC_SYNCBUSY_MASK );
#endif
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTC_16(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_16(Tc* TCx) {
#if (SAMD21 || SAMD11)
  while (TCx->COUNT16.STATUS.bit.SYNCBUSY);
#elif (SAML21 || SAMC21 || SAMD51)
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
#if (SAMD21 || SAMD11)
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
#elif (SAML21)
    ADC->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_12BIT_Val;
#elif (SAMC21)
    ADC0->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_12BIT_Val;
    ADC1->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_12BIT_Val;
#elif (SAMD51)
    ADC0->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
    ADC1->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
#endif
    _ADCResolution = 12;
  } else if (res > 8) {
#if (SAMD21 || SAMD11)
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;
#elif (SAML21)
    ADC->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_10BIT_Val;
#elif (SAMC21)
    ADC0->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_10BIT_Val;
    ADC1->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_10BIT_Val;
#elif (SAMD51)
    ADC0->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;
    ADC1->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;
#endif
    _ADCResolution = 10;
  } else {
#if (SAMD21 || SAMD11)
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_8BIT_Val;
#elif (SAML21)
    ADC->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_8BIT_Val;
#elif (SAMC21)
    ADC0->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_8BIT_Val;
    ADC1->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_8BIT_Val;
#elif (SAMD51)
    ADC0->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_8BIT_Val;
    ADC1->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_8BIT_Val;
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
 * Default Internal Reference for the D21/D11 is at 1.65V (measurements up to Vcc with ADC gain of 1/2)
 * Default Internal Reference for the D51/L21/C21 is at Vcc (generally 3.3V)
 * External Reference should be between 1V and VDDANA-0.6V=2.7V (1V and VDDANA-0.4V=2.9V for D51)
 *
 * Warning : The maximum IO voltage is Vcc (up to 3.6 volts for the SAMD/SAML, 5V for the SAMC)
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

#if (SAMD21 || SAMD11 || SAML21 || SAMD51)
  #if defined(REFB_PIN)
    if (mode == AR_EXTERNAL_REFB) {
      if ( pinPeripheral(REFB_PIN, PIO_ANALOG_REF) != RET_STATUS_OK ) {
          return;
      }
    }
  #endif
#endif

#if (SAMD51) && defined(REFC_PIN)
    if (mode == AR_EXTERNAL_REFC) {
      if ( pinPeripheral(REFC_PIN, PIO_ANALOG_REF) != RET_STATUS_OK ) {
          return;
      }
    }
#endif

#if (SAMD21 || SAMD11)
  if (mode == AR_DEFAULT) {
    ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;    // GAIN_DIV2 allows values up to VDDANA on the pin
    ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // 1/2 VDDANA
  } else {
    ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;
    ADC->REFCTRL.bit.REFSEL = mode;	// Note that mode value can be used with the register. See header file.
  }
#elif (SAML21 || SAMC21 || SAMD51)
  if (mode == 0) {		// Set to 1.0V for the SAML, 1.024V for the SAMC
    SUPC->VREF.reg &= ~SUPC_VREF_SEL_Msk;
  } else if (mode >= AR_INTREF_1V0) {		// Values starting at AR_INTREF_1V0 are used for the Supply Controller reference (AR_INTREF)
    SUPC->VREF.reg &= ~SUPC_VREF_SEL_Msk;
    #if (SAMD51)
      SUPC->VREF.reg |= SUPC_VREF_SEL(mode - AR_INTREF_1V0);  // See eAnalogReference typedef in wiring_analog.h. AR_INTREF_1V0 = 7.
    #else
      SUPC->VREF.reg |= SUPC_VREF_SEL(mode - AR_INTREF_1V0);  // See eAnalogReference typedef in wiring_analog.h. AR_INTREF_1V0 = 6.
    #endif
    mode = 0;
  }
  #if (SAML21)
    ADC->REFCTRL.bit.REFSEL = mode;
  #elif (SAMC21 || SAMD51)
    ADC0->REFCTRL.bit.REFSEL = mode;
    ADC1->REFCTRL.bit.REFSEL = mode;
  #endif
#endif
  syncADC();

  // Start conversion, since The first conversion after the reference is changed must not be used.
#if (SAMC21 || SAMD51)
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

#if (SAMC21 || SAMD51)
  Adc* ADC;
  if ( (g_APinDescription[pin].ulPeripheralAttribute & PER_ATTR_ADC_MASK) == PER_ATTR_ADC_ALT ) {
    ADC = ADC1;
  } else {
    ADC = ADC0;
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
#if (SAMD21 || SAMD11 || SAMC21)
    value = mapResolution(value, _writeResolution, 10);
    DAC->DATA.reg = value & 0x3FF;  // DAC on 10 bits.
    syncDAC();
    if (!dacEnabled[0]) {
      dacEnabled[0] = true;
      DAC->CTRLA.bit.ENABLE = 0x01;     // Enable DAC
      syncDAC();
    }
#elif (SAML21 || SAMD51)
    uint8_t DACNumber = 0x00;
    value = mapResolution(value, _writeResolution, 12);

    if ( (g_APinDescription[pin].ulPort == 0) && (g_APinDescription[pin].ulPin == 5) ) {
        DACNumber = 0x01;
    }

    if (!dacEnabled[DACNumber]) {
      dacEnabled[DACNumber] = true;
      DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC controller (so that DACCTRL can be modified)
      #if (SAML21)
        delayMicroseconds(40);	// Must delay for at least 30us when turning off while refresh is on due to L21 DAC errata
      #endif

      DAC->DATA[DACNumber].reg = value & 0xFFF;  // DACx on 12 bits.
      syncDAC();
      DAC->DACCTRL[DACNumber].bit.ENABLE = 0x01; // The DACx output is turned on.
      DAC->CTRLA.bit.ENABLE = 0x01;     // Enable DAC controller
      syncDAC();
      while ( (DAC->STATUS.reg & (1 << DACNumber)) == 0 );   // Must wait for DACx to start
    }

    DAC->DATA[DACNumber].reg = value & 0xFFF;  // DACx on 12 bits.
    syncDAC();
#endif
    return;
  }
  else if ( pinPeripheral(pin, PIO_TIMER_PWM) == RET_STATUS_OK )
  {
    Tc*  TCx  = 0 ;
    Tcc* TCCx = 0 ;
    uint8_t timerIndex;
    uint8_t timerType = GetTCType( g_APinDescription[pin].ulTCChannel ) ;
    uint8_t timerNumber = GetTCNumber( g_APinDescription[pin].ulTCChannel ) ;
    uint8_t timerChannel = GetTCChannelNumber( g_APinDescription[pin].ulTCChannel ) ;
    static bool timerEnabled[TCC_INST_NUM+TC_INST_NUM];

    if ( timerType == 1 ) {
      TCx = (Tc*) GetTC( g_APinDescription[pin].ulTCChannel ) ;
#if (SAMD21 || SAMD11)
        timerIndex = timerNumber;
#else
        timerIndex = (timerNumber + TCC_INST_NUM);
#endif
    } else {
      TCCx = (Tcc*) GetTC( g_APinDescription[pin].ulTCChannel ) ;
      timerIndex = timerNumber;
    }

    value = mapResolution(value, _writeResolution, 16);

    if (!timerEnabled[timerIndex]) {
      timerEnabled[timerIndex] = true;

#if (SAMD21 || SAMD11)
      GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( timerClockIDs[timerIndex] )) ;
      while ( GCLK->STATUS.bit.SYNCBUSY == 1 );
#elif (SAML21 || SAMC21 || SAMD51)
  #if (SAMD51 && (VARIANT_MCK == 120000000ul))
      GCLK->PCHCTRL[timerClockIDs[timerIndex]].reg = (GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK5);  // use 48MHz clock from GCLK5, which was setup in startup.c
  #else
      GCLK->PCHCTRL[timerClockIDs[timerIndex]].reg = (GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0);
  #endif
    while ( (GCLK->PCHCTRL[timerClockIDs[timerIndex]].reg & GCLK_PCHCTRL_CHEN) == 0 );        // wait for sync
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
#if (SAMD21 || SAMD11)
        TCx->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_NPWM;
#elif (SAML21 || SAMC21 || SAMD51)
        TCx->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_NPWM;
#endif
        syncTC_16(TCx);
        // Set the initial value
        TCx->COUNT16.CC[timerChannel].reg = (uint32_t) value;
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
        TCCx->CC[timerChannel].reg = (uint32_t)value;
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
#if (SAMD21 || SAMD11)
        TCx->COUNT16.CC[timerChannel].reg = (uint32_t) value;
#elif (SAML21 || SAMC21 || SAMD51)
        // SAML and SAMC have double-buffered TCs
        TCx->COUNT16.CCBUF[timerChannel].reg = (uint32_t) value;
#endif
        syncTC_16(TCx);
      } else {
#if (SAMD21 || SAMD11)
        TCCx->CTRLBSET.bit.LUPD = 1;
        syncTCC(TCCx);
        TCCx->CCB[timerChannel].reg = (uint32_t) value;
        syncTCC(TCCx);
        TCCx->CTRLBCLR.bit.LUPD = 1;
// LUPD caused endless spinning in syncTCC() on SAML (and probably SAMC). Note that CCBUF writes are already
// atomic. The LUPD bit is intended for updating several registers at once, which analogWrite() does not do.
#elif (SAML21 || SAMC21 || SAMD51)
        TCCx->CCBUF[timerChannel].reg = (uint32_t) value;
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
