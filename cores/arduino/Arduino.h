/*
  Arduino.h - Main include file for the Arduino SDK
  Copyright (c) 2005-2013 Arduino Team.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef Arduino_h
#define Arduino_h

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// some libraries and sketches depend on this AVR stuff,
// assuming Arduino.h or WProgram.h automatically includes it...
//
#include "avr/pgmspace.h"
#include "avr/interrupt.h"

#include "binary.h"
#include "itoa.h"

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

// Include Atmel headers
#include "sam.h"

#include "wiring_constants.h"

#define clockCyclesPerMicrosecond() ( SystemCoreClock / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (SystemCoreClock / 1000L) )
#define microsecondsToClockCycles(a) ( (a) * (SystemCoreClock / 1000000L) )

void yield( void ) ;

/* sketch */
extern void setup( void ) ;
extern void loop( void ) ;

#define NOT_AN_INTERRUPT -1

typedef enum _EExt_Interrupts
{
  EXTERNAL_INT_0=0,
  EXTERNAL_INT_1=1,
  EXTERNAL_INT_2=2,
  EXTERNAL_INT_3=3,
  EXTERNAL_INT_4=4,
  EXTERNAL_INT_5=5,
  EXTERNAL_INT_6=6,
  EXTERNAL_INT_7=7,
  EXTERNAL_NUM_INTERRUPTS
} EExt_Interrupts ;

typedef void (*voidFuncPtr)( void ) ;

/* Define attribute */
#if defined   ( __CC_ARM   ) /* Keil uVision 4 */
    #define WEAK (__attribute__ ((weak)))
#elif defined ( __ICCARM__ ) /* IAR Ewarm 5.41+ */
    #define WEAK __weak
#elif defined (  __GNUC__  ) /* GCC CS */
    #define WEAK __attribute__ ((weak))
#endif

/* Definitions and types for pins */
typedef enum _EAnalogChannel
{
  NO_ADC=-1,
  ADC0=0,
  ADC1=1,
  ADC2=2,
  ADC3=3,
  ADC4=4,
  ADC5=5,
  ADC6=6,
  ADC7=7,
  ADC10=10,
  ADC16=16,
  ADC17=17,
  ADC18=18,
  ADC19=19,
  DAC0,
} EAnalogChannel ;

#define ADC_CHANNEL_NUMBER_NONE 0xffffffff

// Definitions for TC channels
typedef enum _ETCChannel
{
  NOT_ON_TIMER=-1,
	TC3_CH0,
	TC3_CH1,
	TCC0_CH0,
	TCC0_CH1,
	TCC0_CH4,
	TCC0_CH5,
	TCC0_CH6,
	TCC0_CH7,
	TCC1_CH0,
	TCC1_CH1,
	TCC2_CH0,
	TCC2_CH1
} ETCChannel ;

// Definitions for PWM channels
typedef enum _EPWMChannel
{
  NOT_ON_PWM=-1,
	PWM3_CH0=TC3_CH0,
	PWM3_CH1=TC3_CH1,
	PWM0_CH0=TCC0_CH0,
	PWM0_CH1=TCC0_CH1,
	PWM0_CH4=TCC0_CH4,
	PWM0_CH5=TCC0_CH5,
	PWM0_CH6=TCC0_CH6,
	PWM0_CH7=TCC0_CH7,
	PWM1_CH0=TCC1_CH0,
	PWM1_CH1=TCC1_CH1,
	PWM2_CH0=TCC2_CH0,
	PWM2_CH1=TCC2_CH1
} EPWMChannel ;

typedef enum _EPortType
{
	NOT_A_PORT=-1,
	PORTA=0,
	PORTB=1,
	PORTC=2,
} EPortType ;

//A    B                 C       D          E      F   G   H
//EIC REF ADC AC PTC DAC SERCOM SERCOM_ALT TC/TCC TCC COM AC/GCLK

typedef enum _EPioType
{
  PIO_NOT_A_PIN=-1,  /* Not under control of a peripheral. */
  PIO_EXTINT,     /* The pin is controlled by the associated signal of peripheral A. */
  PIO_ANALOG,     /* The pin is controlled by the associated signal of peripheral B. */
  PIO_SERCOM,     /* The pin is controlled by the associated signal of peripheral C. */
  PIO_SERCOM_ALT, /* The pin is controlled by the associated signal of peripheral D. */
  PIO_TIMER,      /* The pin is controlled by the associated signal of peripheral E. */
  PIO_TIMER_ALT,  /* The pin is controlled by the associated signal of peripheral F. */
  PIO_COM,        /* The pin is controlled by the associated signal of peripheral G. */
  PIO_AC_CLK,     /* The pin is controlled by the associated signal of peripheral H. */
  PIO_PWM=PIO_TIMER,
  PIO_PWM_ALT=PIO_TIMER_ALT,

  PIO_DIGITAL,    /* The pin is controlled by PORT. */
  PIO_INPUT,        /* The pin is controlled by PORT and is an input. */
  PIO_INPUT_PULLUP, /* The pin is controlled by PORT and is an input with internal pull-up resistor enabled. */
  PIO_OUTPUT,       /* The pin is controlled by PORT and is an output. */
} EPioType ;

/**
 * Pin Attributes to be OR-ed
 */
#define PIN_ATTR_COMBO         (1UL<<0)
#define PIN_ATTR_ANALOG        (1UL<<1)
#define PIN_ATTR_DIGITAL       (1UL<<2)
#define PIN_ATTR_PWM           (1UL<<3)
#define PIN_ATTR_TIMER         (1UL<<4)

/* Types used for the table below */
typedef struct _PinDescription
{
  uint32_t ulPort ;
  uint32_t ulPin ;
  EPioType ulPinType ;
  uint32_t ulPinConfiguration ;
  uint32_t ulPinAttribute ;
  EAnalogChannel ulAnalogChannel ; /* Analog pin in the Arduino context (label on the board) */
  EAnalogChannel ulADCChannelNumber ; /* ADC Channel number in the SAM device */
  EPWMChannel ulPWMChannel ;
  ETCChannel ulTCChannel ;
} PinDescription ;

/* Pins table to be instanciated into variant.cpp */
extern const PinDescription g_APinDescription[] ;

#ifdef __cplusplus
} // extern "C"

#include "WCharacter.h"
#include "WString.h"
#include "Tone.h"
#include "WMath.h"
#include "HardwareSerial.h"
#include "wiring_pulse.h"
#include "delay.h"

#endif // __cplusplus

// Include board variant
//#include "variant.h"

#include "wiring.h"
#include "wiring_digital.h"
#include "wiring_analog.h"
#include "wiring_shift.h"
#include "WInterrupts.h"

// USB Device
#define USB_VID            0x2341 // arduino LLC vid
#define USB_PID_LEONARDO   0x0034
#define USB_PID_MICRO      0x0035
#define USB_PID_DUE        0x003E
#define USB_PID_ZERO       0x004D
#include "USB/USBDesc.h"
#include "USB/USBCore.h"
#include "USB/USBAPI.h"

#endif // Arduino_h
