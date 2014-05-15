#ifndef _VARIANTS_
#define _VARIANTS_

#include <stdint.h>
#include "sam.h"

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/* Definitions and types for pins */
typedef enum _EAnalogChannel
{
  No_ADC_Channel=-1,
  ADC_Channel0=0,
  ADC_Channel1=1,
  ADC_Channel2=2,
  ADC_Channel3=3,
  ADC_Channel4=4,
  ADC_Channel5=5,
  ADC_Channel6=6,
  ADC_Channel7=7,
  ADC_Channel10=10,
  ADC_Channel16=16,
  ADC_Channel17=17,
  ADC_Channel18=18,
  ADC_Channel19=19,
  DAC_Channel0,
} EAnalogChannel ;

// Definitions for TC channels
typedef enum _ETCChannel
{
  NOT_ON_TIMER=-1,
	TC3_CH0 = (3<<8)|(0),
	TC3_CH1 = (3<<8)|(1),
	TCC0_CH0 = (0<<8)|(0),
	TCC0_CH1 = (0<<8)|(1),
	TCC0_CH4 = (0<<8)|(4),
	TCC0_CH5 = (0<<8)|(5),
	TCC0_CH6 = (0<<8)|(6),
	TCC0_CH7 = (0<<8)|(7),
	TCC1_CH0 = (1<<8)|(0),
	TCC1_CH1 = (1<<8)|(1),
	TCC2_CH0 = (2<<8)|(0),
	TCC2_CH1 = (2<<8)|(1)
} ETCChannel ;

extern const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM] ;

#define GetTCNumber( x ) ( (x) >> 8 )
#define GetTCChannelNumber( x ) ( (x) && 0xff )
#define GetTC( x ) ( g_apTCInstances[(x) >> 8] )

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
  PIO_DIGITAL,    /* The pin is controlled by PORT. */
  PIO_INPUT,        /* The pin is controlled by PORT and is an input. */
  PIO_INPUT_PULLUP, /* The pin is controlled by PORT and is an input with internal pull-up resistor enabled. */
  PIO_OUTPUT,       /* The pin is controlled by PORT and is an output. */

  PIO_PWM=PIO_TIMER,
  PIO_PWM_ALT=PIO_TIMER_ALT,
} EPioType ;

/**
 * Pin Attributes to be OR-ed
 */
#define PIN_ATTR_NONE          (0UL<<0)
#define PIN_ATTR_COMBO         (1UL<<0)
#define PIN_ATTR_ANALOG        (1UL<<1)
#define PIN_ATTR_DIGITAL       (1UL<<2)
#define PIN_ATTR_PWM           (1UL<<3)
#define PIN_ATTR_TIMER         (1UL<<4)

/* Types used for the table below */
typedef struct _PinDescription
{
  EPortType ulPort ;
  uint32_t ulPin ;
  EPioType ulPinType ;
  uint32_t ulPinAttribute ;
  EAnalogChannel ulADCChannelNumber ; /* ADC Channel number in the SAM device */
  EPWMChannel ulPWMChannel ;
  ETCChannel ulTCChannel ;
} PinDescription ;

/* Pins table to be instanciated into variant.cpp */
extern const PinDescription g_APinDescription[] ;

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // _VARIANTS_
