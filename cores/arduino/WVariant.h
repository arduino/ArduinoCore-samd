/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
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

#pragma once

#include <stdint.h>
#include "sam.h"
#include "variant.h"
#include "../../config.h"

#ifdef __cplusplus
extern "C" {
#endif

// Definitions for TC channels
// Timer Enable (1 bit: 0=enabled, 1=disabled) | Timer Number (3 bits: 0-7) | Timer Type (1 bit: 0=TCC, 1=TC) | Timer Channel (3 bits: 0-7)
typedef enum _ETCChannel
{
  TCC0_CH0 = (0<<4)|(0<<3)|(0),
  TCC0_CH1 = (0<<4)|(0<<3)|(1),
  TCC0_CH2 = (0<<4)|(0<<3)|(2),
  TCC0_CH3 = (0<<4)|(0<<3)|(3),
#if (SAMD51)
  TCC0_CH4 = (0<<4)|(0<<3)|(4), // There are 6 compare channels on TCC0 on the D51
  TCC0_CH5 = (0<<4)|(0<<3)|(5), // There are 6 compare channels on TCC0 on the D51
  TCC0_CH6 = (0<<4)|(0<<3)|(0), // Channel 6 is 0!
  TCC0_CH7 = (0<<4)|(0<<3)|(1), // Channel 7 is 1!
#else
  TCC0_CH4 = (0<<4)|(0<<3)|(0), // Channel 4 is 0!
  TCC0_CH5 = (0<<4)|(0<<3)|(1), // Channel 5 is 1!
  TCC0_CH6 = (0<<4)|(0<<3)|(2), // Channel 6 is 2!
  TCC0_CH7 = (0<<4)|(0<<3)|(3), // Channel 7 is 3!
#endif
  TCC1_CH0 = (1<<4)|(0<<3)|(0),
  TCC1_CH1 = (1<<4)|(0<<3)|(1),
#if (SAMD51)
  TCC1_CH2 = (1<<4)|(0<<3)|(2), // There are 4 compare channels on TCC1 on the D51
  TCC1_CH3 = (1<<4)|(0<<3)|(3), // There are 4 compare channels on TCC1 on the D51
  TCC1_CH4 = (1<<4)|(0<<3)|(0), // Channel 4 is 0!
  TCC1_CH5 = (1<<4)|(0<<3)|(1), // Channel 5 is 1!
  TCC1_CH6 = (1<<4)|(0<<3)|(2), // Channel 6 is 2!
  TCC1_CH7 = (1<<4)|(0<<3)|(3), // Channel 7 is 3!
#else
  TCC1_CH2 = (1<<4)|(0<<3)|(0), // Channel 2 is 0!
  TCC1_CH3 = (1<<4)|(0<<3)|(1), // Channel 3 is 1!
#endif
  TCC2_CH0 = (2<<4)|(0<<3)|(0),
  TCC2_CH1 = (2<<4)|(0<<3)|(1),
#if (SAMD51)
  TCC2_CH2 = (2<<4)|(0<<3)|(2), // There are 3 compare channels on TCC2 on the D51
  TCC3_CH0 = (3<<4)|(0<<3)|(0),
  TCC3_CH1 = (3<<4)|(0<<3)|(1),
  TCC4_CH0 = (4<<4)|(0<<3)|(0),
  TCC4_CH1 = (4<<4)|(0<<3)|(1),
#endif
  TC0_CH0  = (0<<4)|(1<<3)|(0),
  TC0_CH1  = (0<<4)|(1<<3)|(1),
  TC1_CH0  = (1<<4)|(1<<3)|(0),
  TC1_CH1  = (1<<4)|(1<<3)|(1),
  TC2_CH0  = (2<<4)|(1<<3)|(0),
  TC2_CH1  = (2<<4)|(1<<3)|(1),
  TC3_CH0  = (3<<4)|(1<<3)|(0),
  TC3_CH1  = (3<<4)|(1<<3)|(1),
  TC4_CH0  = (4<<4)|(1<<3)|(0),
  TC4_CH1  = (4<<4)|(1<<3)|(1),
  TC5_CH0  = (5<<4)|(1<<3)|(0),
  TC5_CH1  = (5<<4)|(1<<3)|(1),
  TC6_CH0  = (6<<4)|(1<<3)|(0),
  TC6_CH1  = (6<<4)|(1<<3)|(1),
  TC7_CH0  = (7<<4)|(1<<3)|(0),
  TC7_CH1  = (7<<4)|(1<<3)|(1),
  NOT_ON_TIMER = (1<<7),
} ETCChannel ;

extern const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM] ;

#define GetTCNumber( x ) ( ((x) >> 4) & 0x07 )
#define GetTCType( x ) ( ((x) >> 3) & 0x01 )
#define GetTCChannelNumber( x ) ( (x) & 0x07 )
#if (SAMD21 || SAMD11)
#define GetTC( x ) ( g_apTCInstances[GetTCNumber(x)] )
#elif (SAML21)
#define GetTC( x ) ( GetTCType(x) == 0 ? g_apTCInstances[GetTCNumber(x)] : (GetTCNumber(x) == 4 ? TC4 : g_apTCInstances[GetTCNumber(x) + TCC_INST_NUM]) )
#elif (SAMC21 || SAMD51)
#define GetTC( x ) ( GetTCType(x) == 0 ? g_apTCInstances[GetTCNumber(x)] : g_apTCInstances[GetTCNumber(x) + TCC_INST_NUM] )
#else
#error "WVariant.h: Unsupported chip"
#endif

// TODO: Definitions for GCLK_CCL column, AC. The format may change.
// GCLK/CCL Enable (1 bit: 0=enabled, 1=disabled) | GCLK (3 bits: 0-7) | CCL (4 bits: 2 for CCL number, 2 for pin (or 4 for IN/OUT pin ID on D51))
typedef enum _EGCLK_CCL
{
  GCLK_CCL_NONE = (1<<7),
} EGCLK_CCL ;

typedef enum _EPortType
{
  PORTA=0,
  PORTB=1,
  PORTC=2,
  PORTD=3,
  NOT_A_PORT,
} EPortType ;

// When PIN_DESCRIPTION_TABLE_SIMPLE defined, the port and pin are combined into one single byte element rather than two
// PORT (3 bits: 0-7, EPortType) | PIN (5 bits: 0-31)
#if defined(PIN_DESCRIPTION_TABLE_SIMPLE)
  // Use SetPortPin in the ulPortPin column of the PinDescription table (when using PIN_DESCRIPTION_TABLE_SIMPLE only).
  #define SetPortPin( port, pin ) ( (((port & 0x07) << 5) | (pin & 0x1F)) )
  #define GetPort( x ) ( ((g_APinDescription[x].ulPortPin) >> 5) & 0x07 )
  #define GetPin( x ) ( (g_APinDescription[x].ulPortPin) & 0x1F )
#else
  #define GetPort( x ) g_APinDescription[x].ulPort
  #define GetPin( x ) g_APinDescription[x].ulPin
#endif

/* Definitions and types for pins */
typedef enum _EAnalogChannel
{
  ADC_Channel0=0,
  ADC_Channel1=1,
  ADC_Channel2=2,
  ADC_Channel3=3,
  ADC_Channel4=4,
  ADC_Channel5=5,
  ADC_Channel6=6,
  ADC_Channel7=7,
  ADC_Channel8=8,
  ADC_Channel9=9,
  ADC_Channel10=10,
  ADC_Channel11=11,
  ADC_Channel12=12,
  ADC_Channel13=13,
  ADC_Channel14=14,
#if defined(PIN_DESCRIPTION_TABLE_SIMPLE)
  No_ADC_Channel = 15,
#else
  ADC_Channel15=15,
  ADC_Channel16=16,
  ADC_Channel17=17,
  ADC_Channel18=18,
  ADC_Channel19=19,
  DAC_Channel0,
  DAC_Channel1,
  No_ADC_Channel,
#endif
} EAnalogChannel ;

typedef enum
{
  EXTERNAL_INT_0 = 0,
  EXTERNAL_INT_1,
  EXTERNAL_INT_2,
  EXTERNAL_INT_3,
  EXTERNAL_INT_4,
  EXTERNAL_INT_5,
  EXTERNAL_INT_6,
  EXTERNAL_INT_7,
  EXTERNAL_INT_8,
  EXTERNAL_INT_9,
  EXTERNAL_INT_10,
  EXTERNAL_INT_11,
  EXTERNAL_INT_12,
  EXTERNAL_INT_13,
  EXTERNAL_INT_14,
#if defined(PIN_DESCRIPTION_TABLE_SIMPLE)
  NOT_AN_INTERRUPT = 15,
  EXTERNAL_NUM_INTERRUPTS = NOT_AN_INTERRUPT,
#else
  EXTERNAL_INT_15,
  EXTERNAL_INT_NMI,
  EXTERNAL_NUM_INTERRUPTS,
  NOT_AN_INTERRUPT,
#endif
  EXTERNAL_INT_NONE = NOT_AN_INTERRUPT,
} EExt_Interrupts ;

// When PIN_DESCRIPTION_TABLE_SIMPLE defined, the ExtInt and ADCChannelNumber fields are combined into one single byte element (ulExtIntADC) rather than two
// Because of this, only interrupts 0 through 14 are supported (15 = NOT_AN_INTERRUPT) and only ADC channels 0 through 14 are supported (15 = No_ADC_Channel)
// ExtInt (4 bits: 0-15, EExt_Interrupts) | ADC (4 bits: 0-15, EAnalogChannel)
#if defined(PIN_DESCRIPTION_TABLE_SIMPLE)
  // Use SetExtIntADC in the ulExtIntADC column of the PinDescription table (when using PIN_DESCRIPTION_TABLE_SIMPLE only).
  #define SetExtIntADC( ExtInt, ADC ) ( (((ExtInt & 0x0F) << 4) | (ADC & 0x0F)) )
  #define GetExtInt( x ) ( ((g_APinDescription[x].ulExtIntADC) >> 4) & 0x0F )
  #define GetADC( x ) ( (g_APinDescription[x].ulExtIntADC) & 0x0F )
#else
  #define GetExtInt( x ) g_APinDescription[x].ulExtInt
  #define GetADC( x ) g_APinDescription[x].ulADCChannelNumber
#endif


/* Copied from wiring_constants.h */
#define INPUT           (0x0)
#define OUTPUT          (0x1)
#define INPUT_PULLUP    (0x2)
#define INPUT_PULLDOWN  (0x3)
typedef enum _EPioType
{
  PIO_INPUT=INPUT,                    /* The pin is controlled by PORT and is an input. */
  PIO_OUTPUT=OUTPUT,                  /* The pin is controlled by PORT and is an output. */
  PIO_INPUT_PULLUP=INPUT_PULLUP,      /* The pin is controlled by PORT and is an input with internal pull-up resistor enabled. */
  PIO_INPUT_PULLDOWN=INPUT_PULLDOWN,  /* The pin is controlled by PORT and is an input with internal pull-down resistor enabled. */

  PIO_EXTINT=4,                       /* The pin is controlled by the EXTINT peripheral and is an input with interrupt. */

  PIO_ANALOG_ADC=5,                   /* The pin is controlled by the ANALOG peripheral and is an ADC input. */
  PIO_ANALOG_DAC=6,                   /* The pin is controlled by the ANALOG peripheral and is a DAC output. */
  PIO_ANALOG_REF=7,                   /* The pin is controlled by the ANALOG peripheral and is a voltage reference input (3.3V MAX). */
  PIO_ANALOG_AC=8,                    /* The pin is controlled by the ANALOG peripheral and is used by the AC (analog comparator). */
  PIO_ANALOG_PTC=9,                   /* The pin is controlled by the ANALOG peripheral and is used by the PTC (peripheral touch controller). */
  PIO_ANALOG_SDADC=10,                /* The pin is controlled by the ANALOG peripheral and is used by the SDADC (sigma-delta ADC). */
  PIO_ANALOG_OPAMP=11,                /* The pin is controlled by the ANALOG peripheral and is used by the OPAMP (L21 only). */

  PIO_TIMER_PWM=12,                   /* The pin is controlled by a TIMER peripheral and is a PWM output. */
  PIO_TIMER_CAPTURE=13,               /* The pin is controlled by a TIMER peripheral and is a capture input. */

  PIO_SERCOM=14,                      /* The pin is controlled by a SERCOM peripheral (UART, SPI, or I2C). */
  PIO_COM=15,                         /* The pin is controlled by the COM peripheral (USB, CAN, or CORTEX (I2S on D21 only)). */
  PIO_USB=16,                         /* The pin is controlled by the COM peripheral (except C21). */
  PIO_CAN=17,                         /* The pin is controlled by the COM or SDHC peripheral (D51) or the COM peripheral (C21). */
  PIO_QSPI=18,                        /* The pin is controlled by the COM peripheral (QSPI on D51 only). */
  PIO_SDHC=19,                        /* The pin is controlled by the SDHC peripheral (SDHC and CAN0 on D51 only). */
  PIO_I2S=20,                         /* The pin is controlled by the I2S (inter-IC sound) peripheral (D51) or the COM peripheral (D21). */
  PIO_GMAC=21,                        /* The pin is controlled by the GMAC peripheral (ethernet on D51 only). */

  PIO_PCC=22,                         /* The pin is controlled by the PCC peripheral (parallel capture controller on D51 only). */
  PIO_AC_GCLK=23,                     /* The pin is controlled by the AC_GCLK peripheral (analog comparator / generic clock). */
  PIO_CCL=24,                         /* The pin is controlled by the CCL (configurable custom logic) peripheral (I/O). */

  PIO_MULTI,                          /* The pin can be configured to any type based on the attributes. */
  PIO_STARTUP,                        /* Used as parameter to pinPeripheral() only to set startup state (enable INEN only) */
  PIO_NOT_A_PIN,                      /* Not under control of a peripheral. */
} EPioType ;

/**
 * Pin Attributes to be OR-ed
 */
#define PIN_ATTR_NONE              (0UL << 0)

  #define PIN_ATTR_INPUT           (1UL << PIO_INPUT)
  #define PIN_ATTR_OUTPUT          (1UL << PIO_OUTPUT)
  #define PIN_ATTR_INPUT_PULLUP    (1UL << PIO_INPUT_PULLUP)
  #define PIN_ATTR_INPUT_PULLDOWN  (1UL << PIO_INPUT_PULLDOWN)
#define PIN_ATTR_DIGITAL           (PIN_ATTR_INPUT|PIN_ATTR_INPUT_PULLUP|PIN_ATTR_INPUT_PULLDOWN|PIN_ATTR_OUTPUT)

#define PIN_ATTR_EXTINT            (1UL << PIO_EXTINT)

#define PIN_ATTR_ADC               (1UL << PIO_ANALOG_ADC)
#define PIN_ATTR_DAC               (1UL << PIO_ANALOG_DAC)
#define PIN_ATTR_REF               (1UL << PIO_ANALOG_REF)
#define PIN_ATTR_AC                (1UL << PIO_ANALOG_AC)
#define PIN_ATTR_PTC               (1UL << PIO_ANALOG_PTC)
#define PIN_ATTR_SDADC             (1UL << PIO_ANALOG_SDADC)
#define PIN_ATTR_OPAMP             (1UL << PIO_ANALOG_OPAMP)
  #define PIN_ATTR_ANALOG          PIN_ATTR_ADC

  #define PIN_ATTR_TIMER_PWM       (1UL << PIO_TIMER_PWM)
  #define PIN_ATTR_TIMER_CAPTURE   (1UL << PIO_TIMER_CAPTURE)
#define PIN_ATTR_TIMER_BOTH        (PIN_ATTR_TIMER_PWM|PIN_ATTR_TIMER_CAPTURE)
#define PIN_ATTR_TIMER             PIN_ATTR_TIMER_BOTH

#define PIN_ATTR_SERCOM            (1UL << PIO_SERCOM)
#define PIN_ATTR_COM               (1UL << PIO_COM)
#define PIN_ATTR_USB               (1UL << PIO_USB)
#define PIN_ATTR_CAN               (1UL << PIO_CAN)
#define PIN_ATTR_QSPI              (1UL << PIO_QSPI)
#define PIN_ATTR_SDHC              (1UL << PIO_SDHC)
#define PIN_ATTR_I2S               (1UL << PIO_I2S)
#define PIN_ATTR_GMAC              (1UL << PIO_GMAC)

#define PIN_ATTR_PCC               (1UL << PIO_PCC)
#define PIN_ATTR_AC_GCLK           (1UL << PIO_AC_GCLK)
#define PIN_ATTR_CCL               (1UL << PIO_CCL)


/*            A  |------ B -------|   C        D        E     F     G         H       I    J   K   L      M     N
D21/L21/C21: EIC REF ADC AC PTC DAC SERCOM SERCOM_ALT TC/TCC TCC   COM     AC/GCLK   CCL
D51:         EIC REF ADC AC PTC DAC SERCOM SERCOM_ALT TC     TCC TCC/PDEC  COM/QSPI  SDHC I2S PCC GMAC AC/GCLK CCL
*/
typedef enum _EPioPeripheral
{
	PER_EXTINT=0,         /* The pin is controlled by the associated signal of peripheral A. */
	PER_ANALOG=1,         /* The pin is controlled by the associated signal of peripheral B. */
	PER_SERCOM=2,         /* The pin is controlled by the associated signal of peripheral C. */
	PER_SERCOM_ALT=3,     /* The pin is controlled by the associated signal of peripheral D. */
	PER_TIMER=4,          /* The pin is controlled by the associated signal of peripheral E. */
        PER_TIMER_ALT=5,      /* The pin is controlled by the associated signal of peripheral F. */
#if (SAMD51)
        PER_TIMER_ALT2=6,     /* The pin is controlled by the associated signal of peripheral G. */
        PER_COM=7,            /* The pin is controlled by the associated signal of peripheral H. */
        PER_SDHC=8,           /* The pin is controlled by the associated signal of peripheral I. */
        PER_I2S=9,            /* The pin is controlled by the associated signal of peripheral J. */
        PER_PCC=10,           /* The pin is controlled by the associated signal of peripheral K. */
        PER_GMAC=11,          /* The pin is controlled by the associated signal of peripheral L. */
        PER_AC_CLK=12,        /* The pin is controlled by the associated signal of peripheral M. */
        PER_CCL=13,           /* The pin is controlled by the associated signal of peripheral N. */
#else
	PER_COM=6,            /* The pin is controlled by the associated signal of peripheral G. */
	PER_AC_CLK=7,         /* The pin is controlled by the associated signal of peripheral H. */
	PER_CCL=8,            /* The pin is controlled by the associated signal of peripheral I. */
#endif
        PER_PORT,             /* The pin is controlled by PORT. */
} EPioPeripheral ;

/**
 * Peripheral Attributes to be OR-ed
 */
#define PER_ATTR_NONE            (0UL<<0)

#define PER_ATTR_SERCOM_STD      (0UL<<0)
#define PER_ATTR_SERCOM_ALT      (1UL<<0)
#define PER_ATTR_SERCOM_MASK     (1UL<<0)

#define PER_ATTR_TIMER_STD       (0UL<<1)
#define PER_ATTR_TIMER_ALT       (1UL<<1)
#define PER_ATTR_TIMER_ALT2      (2UL<<1)
#define PER_ATTR_TIMER_MASK      (3UL<<1)

#define PER_ATTR_DRIVE_STD       (0UL<<3)
#define PER_ATTR_DRIVE_STRONG    (1UL<<3)
#define PER_ATTR_DRIVE_MASK      (1UL<<3)

#define PER_ATTR_OUTPUT_TYPE_STD            (0UL<<4)
#define PER_ATTR_OUTPUT_TYPE_OPEN_DRAIN     (1UL<<4)
#define PER_ATTR_OUTPUT_TYPE_OPEN_SOURCE    (2UL<<4)
#define PER_ATTR_OUTPUT_TYPE_BUSKEEPER      (3UL<<4)
#define PER_ATTR_OUTPUT_TYPE_MASK           (3UL<<4)

#define PER_ATTR_INPUT_SYNCHRONIZER_ON_DEMAND     (0UL<<6)
#define PER_ATTR_INPUT_SYNCHRONIZER_ALWAYS_ON     (1UL<<6)
#define PER_ATTR_INPUT_SYNCHRONIZER_MASK          (1UL<<6)

#define PER_ATTR_ADC_STD             (0UL<<7)
#define PER_ATTR_ADC_ALT             (1UL<<7)
#define PER_ATTR_ADC_MASK            (1UL<<7)


/* PinDescription table */
#if defined(PIN_DESCRIPTION_TABLE_SIMPLE)
  #if !defined(MATTAIRTECH_ARDUINO_SAMD_VARIANT_COMPLIANCE) || (MATTAIRTECH_ARDUINO_SAMD_VARIANT_COMPLIANCE < 10618)
    #error "The PinDescription table in the variant.cpp file of your board variant must be updated (MATTAIRTECH_ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10618) in order to use PIN_DESCRIPTION_TABLE_SIMPLE. See VARIANT_COMPLIANCE_CHANGELOG."
  #endif
  #if !(SAMD11)
    #error "PIN_DESCRIPTION_TABLE_SIMPLE is currently only supported by the D11 variants (although you can still add support yourself, see the variant.cpp of a D11 variant for an example)."
  #endif
// This struct MUST be 4 bytes long (elements are ordered to prevent unaligned access).
typedef struct _PinDescription
{
  uint8_t         ulPortPin ;                   // Must be 8 bits
  uint8_t         ulPeripheralAttribute ;       // Must be 8 bit bitfield
  uint8_t         ulTCChannel ;                 // Must be 8 bits
  uint8_t         ulExtIntADC ;                 // Must be 8 bits
} PinDescription ;

#else
// This struct MUST be 12 bytes long (elements are ordered to prevent unaligned access).
typedef struct _PinDescription
{
  uint8_t         ulPort ;	                // Must be 8 bits
  uint8_t         ulPin ;	                // Must be 8 bits
  uint8_t         ulPinType ;	                // Must be 8 bits
  uint8_t         ulPeripheralAttribute ;	// Must be 8 bit bitfield
  uint32_t        ulPinAttribute ;	        // Must be 32 bit bitfield
#if defined(MATTAIRTECH_ARDUINO_SAMD_VARIANT_COMPLIANCE) && (MATTAIRTECH_ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10608)
  uint8_t         ulTCChannel ;	                // Must be 8 bits
  uint8_t         ulADCChannelNumber ;	        // Must be 8 bits
  uint8_t         ulExtInt ;	                // Must be 8 bits
  uint8_t         ulGCLKCCL ;	                // Must be 8 bits
#else
#error "The PinDescription table in the variant.cpp file of your board variant must be updated so that MATTAIRTECH_ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10608. See VARIANT_COMPLIANCE_CHANGELOG."
//  uint16_t        ulTCChannel ;	        // Must be 16 bits
//  uint8_t         ulADCChannelNumber ;	// Must be 8 bits
//  uint8_t         ulExtInt ;	                // Must be 8 bits
#endif
} PinDescription ;
#endif

/* Pins table to be instantiated into variant.cpp */
extern const PinDescription g_APinDescription[] ;

/* Generic Clock Multiplexer IDs */
#if (SAMD21 || SAMD11)

#define GCM_DFLL48M_REF           (0x00U)
#define GCM_FDPLL96M_INPUT        (0x01U)
#define GCM_FDPLL96M_32K          (0x02U)
#define GCM_WDT                   (0x03U)
#define GCM_RTC                   (0x04U)
#define GCM_EIC                   (0x05U)
#define GCM_USB                   (0x06U)
#define GCM_EVSYS_CHANNEL_0       (0x07U)
#define GCM_EVSYS_CHANNEL_1       (0x08U)
#define GCM_EVSYS_CHANNEL_2       (0x09U)
#define GCM_EVSYS_CHANNEL_3       (0x0AU)
#define GCM_EVSYS_CHANNEL_4       (0x0BU)
#define GCM_EVSYS_CHANNEL_5       (0x0CU)
#if (SAMD11)
#define GCM_SERCOMx_SLOW          (0x0DU)
#define GCM_SERCOM0_CORE          (0x0EU)
#define GCM_SERCOM1_CORE          (0x0FU)
#define GCM_SERCOM2_CORE          (0x10U)
#define GCM_TCC0                  (0x11U)
#define GCM_TC1_TC2               (0x12U)
#define GCM_ADC                   (0x13U)
#define GCM_AC_DIG                (0x14U)
#define GCM_AC_ANA                (0x15U)
#define GCM_DAC                   (0x16U)
#define GCM_PTC                   (0x17U)
#else
#define GCM_EVSYS_CHANNEL_6       (0x0DU)
#define GCM_EVSYS_CHANNEL_7       (0x0EU)
#define GCM_EVSYS_CHANNEL_8       (0x0FU)
#define GCM_EVSYS_CHANNEL_9       (0x10U)
#define GCM_EVSYS_CHANNEL_10      (0x11U)
#define GCM_EVSYS_CHANNEL_11      (0x12U)
#define GCM_SERCOMx_SLOW          (0x13U)
#define GCM_SERCOM0_CORE          (0x14U)
#define GCM_SERCOM1_CORE          (0x15U)
#define GCM_SERCOM2_CORE          (0x16U)
#define GCM_SERCOM3_CORE          (0x17U)
#define GCM_SERCOM4_CORE          (0x18U)
#define GCM_SERCOM5_CORE          (0x19U)
#define GCM_TCC0_TCC1             (0x1AU)
#define GCM_TCC2_TC3              (0x1BU)
#define GCM_TC4_TC5               (0x1CU)
#define GCM_TC6_TC7               (0x1DU)
#define GCM_ADC                   (0x1EU)
#define GCM_AC_DIG                (0x1FU)
#define GCM_AC_ANA                (0x20U)
#define GCM_DAC                   (0x21U)
#define GCM_PTC                   (0x22U)
#define GCM_I2S_0                 (0x23U)
#define GCM_I2S_1                 (0x24U)
#endif

#elif (SAMD51)
#define GCM_DFLL48M_REF           (0x00U)
#define GCM_FDPLL0_INPUT          (0x01U)
#define GCM_FDPLL1_INPUT          (0x02U)
#define GCM_FDPLL0_32K            (0x03U)
#define GCM_FDPLL1_32K            (GCM_FDPLL_0_32K)
#define GCM_SDHC0_SLOW            (GCM_FDPLL_0_32K)
#define GCM_SDHC1_SLOW            (GCM_FDPLL_0_32K)
#define GCM_SERCOMx_SLOW          (GCM_FDPLL_0_32K)
#define GCM_EIC                   (0x04U)
#define GCM_FREQM_MSR             (0x05U)
#define GCM_FREQM_REF             (0x06U)
#define GCM_SERCOM0_CORE          (0x07U)
#define GCM_SERCOM1_CORE          (0x08U)
#define GCM_TC0_TC1               (0x09U)
#define GCM_USB                   (0x0AU)
#define GCM_EVSYS_CHANNEL_0       (0x0BU)
#define GCM_EVSYS_CHANNEL_1       (0x0CU)
#define GCM_EVSYS_CHANNEL_2       (0x0DU)
#define GCM_EVSYS_CHANNEL_3       (0x0EU)
#define GCM_EVSYS_CHANNEL_4       (0x0FU)
#define GCM_EVSYS_CHANNEL_5       (0x10U)
#define GCM_EVSYS_CHANNEL_6       (0x11U)
#define GCM_EVSYS_CHANNEL_7       (0x12U)
#define GCM_EVSYS_CHANNEL_8       (0x13U)
#define GCM_EVSYS_CHANNEL_9       (0x14U)
#define GCM_EVSYS_CHANNEL_10      (0x15U)
#define GCM_EVSYS_CHANNEL_11      (0x16U)
#define GCM_SERCOM2_CORE          (0x17U)
#define GCM_SERCOM3_CORE          (0x18U)
#define GCM_TCC0_TCC1             (0x19U)
#define GCM_TC2_TC3               (0x1AU)
#define GCM_CAN0                  (0x1BU)
#define GCM_CAN1                  (0x1CU)
#define GCM_TCC2_TCC3             (0x1DU)
#define GCM_TC4_TC5               (0x1EU)
#define GCM_PDEC                  (0x1FU)
#define GCM_AC                    (0x20U)
#define GCM_CCL                   (0x21U)
#define GCM_SERCOM4_CORE          (0x22U)
#define GCM_SERCOM5_CORE          (0x23U)
#define GCM_SERCOM6_CORE          (0x24U)
#define GCM_SERCOM7_CORE          (0x25U)
#define GCM_TCC4                  (0x26U)
#define GCM_TC6_TC7               (0x27U)
#define GCM_ADC0                  (0x28U)
#define GCM_ADC1                  (0x29U)
#define GCM_DAC                   (0x2AU)
#define GCM_I2S                   (0x2BU)
#define GCM_SDHC0                 (0x2CU)
#define GCM_SDHC1                 (0x2DU)
#define GCM_CM4_TRACE             (0x2EU)

#elif (SAML21)
#define GCM_DFLL48M_REF           (0x00U)
#define GCM_FDPLL96M_INPUT        (0x01U)
#define GCM_FDPLL96M_32K          (0x02U)
#define GCM_EIC                   (0x03U)
#define GCM_USB                   (0x04U)
#define GCM_EVSYS_CHANNEL_0       (0x05U)
#define GCM_EVSYS_CHANNEL_1       (0x06U)
#define GCM_EVSYS_CHANNEL_2       (0x07U)
#define GCM_EVSYS_CHANNEL_3       (0x08U)
#define GCM_EVSYS_CHANNEL_4       (0x09U)
#define GCM_EVSYS_CHANNEL_5       (0x0AU)
#define GCM_EVSYS_CHANNEL_6       (0x0BU)
#define GCM_EVSYS_CHANNEL_7       (0x0CU)
#define GCM_EVSYS_CHANNEL_8       (0x0DU)
#define GCM_EVSYS_CHANNEL_9       (0x0EU)
#define GCM_EVSYS_CHANNEL_10      (0x0FU)
#define GCM_EVSYS_CHANNEL_11      (0x10U)
#define GCM_SERCOMx_SLOW          (0x11U)
#define GCM_SERCOM0_CORE          (0x12U)
#define GCM_SERCOM1_CORE          (0x13U)
#define GCM_SERCOM2_CORE          (0x14U)
#define GCM_SERCOM3_CORE          (0x15U)
#define GCM_SERCOM4_CORE          (0x16U)
#define GCM_SERCOM5_SLOW          (0x17U)
#define GCM_SERCOM5_CORE          (0x18U)
#define GCM_TCC0_TCC1             (0x19U)
#define GCM_TCC2                  (0x1AU)
#define GCM_TC0_TC1               (0x1BU)
#define GCM_TC2_TC3               (0x1CU)
#define GCM_TC4                   (0x1DU)
#define GCM_ADC                   (0x1EU)
#define GCM_AC                    (0x1FU)
#define GCM_DAC                   (0x20U)
#define GCM_PTC                   (0x21U)
#define GCM_CCL                   (0x22U)

#elif (SAMC21)
#define GCM_FDPLL96M_INPUT        (0x00U)
#define GCM_FDPLL96M_32K          (0x01U)
#define GCM_EIC                   (0x02U)
#define GCM_FREQM_MEASURE         (0x03U)
#define GCM_FREQM_REF             (0x04U)
#define GCM_TSENS                 (0x05U)
#define GCM_EVSYS_CHANNEL_0       (0x06U)
#define GCM_EVSYS_CHANNEL_1       (0x07U)
#define GCM_EVSYS_CHANNEL_2       (0x08U)
#define GCM_EVSYS_CHANNEL_3       (0x09U)
#define GCM_EVSYS_CHANNEL_4       (0x0AU)
#define GCM_EVSYS_CHANNEL_5       (0x0BU)
#define GCM_EVSYS_CHANNEL_6       (0x0CU)
#define GCM_EVSYS_CHANNEL_7       (0x0DU)
#define GCM_EVSYS_CHANNEL_8       (0x0EU)
#define GCM_EVSYS_CHANNEL_9       (0x0FU)
#define GCM_EVSYS_CHANNEL_10      (0x10U)
#define GCM_EVSYS_CHANNEL_11      (0x11U)
#define GCM_SERCOMx_SLOW          (0x12U)
#define GCM_SERCOM0_CORE          (0x13U)
#define GCM_SERCOM1_CORE          (0x14U)
#define GCM_SERCOM2_CORE          (0x15U)
#define GCM_SERCOM3_CORE          (0x16U)
#define GCM_SERCOM4_CORE          (0x17U)
#define GCM_SERCOM5_SLOW          (0x18U)
#define GCM_SERCOM5_CORE          (0x19U)
#define GCM_CAN0                  (0x1AU)
#define GCM_CAN1                  (0x1BU)
#define GCM_TCC0_TCC1             (0x1CU)
#define GCM_TCC2                  (0x1DU)
#define GCM_TC0_TC1               (0x1EU)
#define GCM_TC2_TC3               (0x1FU)
#define GCM_TC4                   (0x20U)
#define GCM_ADC0                  (0x21U)
#define GCM_ADC1                  (0x22U)
#define GCM_SDADC                 (0x23U)
#define GCM_AC                    (0x22U)
#define GCM_DAC                   (0x24U)
#define GCM_PTC                   (0x25U)
#define GCM_CCL                   (0x26U)
#define GCM_NVMCTRL               (0x27U)

#endif

#ifdef __cplusplus
} // extern "C"
#endif
