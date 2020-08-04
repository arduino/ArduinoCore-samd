/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

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

#ifndef _VARIANT_MONSTER_M4SK_
#define _VARIANT_MONSTER_M4SK_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC	   (32768ul)

/** Master clock frequency */
#define VARIANT_MCK        (F_CPU)

#define VARIANT_GCLK0_FREQ (F_CPU)
#define VARIANT_GCLK1_FREQ (48000000UL)
#define VARIANT_GCLK2_FREQ (100000000UL)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (37u)
#define NUM_DIGITAL_PINS     (37u)
#define NUM_ANALOG_INPUTS    (4u)
#define NUM_ANALOG_OUTPUTS   (2u)
#define analogInputToDigitalPin(p)  ((p < NUM_ANALOG_INPUTS) ? (p) + PIN_A0 : -1)

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// LEDs
#define PIN_LED_13           (13u)
#define PIN_LED              PIN_LED_13
#define LED_BUILTIN          PIN_LED_13

/*
 * Analog pins
 */
#define PIN_A0               (14ul)
#define PIN_A1               (PIN_A0 + 1)
#define PIN_A2               (PIN_A0 + 2)
#define PIN_A3               (PIN_A0 + 3)
#define PIN_DAC0              PIN_A0
#define PIN_DAC1              PIN_A1

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;

static const uint8_t DAC0 = PIN_DAC0;
static const uint8_t DAC1 = PIN_DAC1;

#define ADC_RESOLUTION		12

// Other pins
#define PIN_ATN              (26ul)
static const uint8_t ATN = PIN_ATN;

/*
 * Serial interfaces
 */

// Serial1
#define PIN_SERIAL1_RX       (0ul)      // sercom3
#define PIN_SERIAL1_TX       (1ul)      // sercom3
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)
#define PAD_SERIAL1_TX       (UART_TX_PAD_0)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT     3

  // Right TFT
#define PIN_SPI_MISO         (6u) // actually DC pin, ignored
#define PIN_SPI_SCK          (8u)
#define PIN_SPI_MOSI         (7u)
#define PERIPH_SPI           sercom2
#define PAD_SPI_TX           SPI_PAD_0_SCK_1
#define PAD_SPI_RX           SERCOM_RX_PAD_3

static const uint8_t SS	  = 5 ;	
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

  // Left TFT
#define PIN_SPI1_MISO         (10u) // actually DC pin, ignored
#define PIN_SPI1_SCK          (12u)
#define PIN_SPI1_MOSI         (11u)
#define PERIPH_SPI1           sercom5
#define PAD_SPI1_TX           SPI_PAD_0_SCK_1
#define PAD_SPI1_RX           SERCOM_RX_PAD_3

static const uint8_t SS1   = 9 ;	
static const uint8_t MOSI1 = PIN_SPI_MOSI ;
static const uint8_t MISO1 = PIN_SPI_MISO ;
static const uint8_t SCK1  = PIN_SPI_SCK ;

// SPI for PDM
#define PIN_SPI2_MISO         (23u)
#define PIN_SPI2_SCK          (24u)
#define PIN_SPI2_MOSI         (13u)  // actually LED, ignored
#define PERIPH_SPI2           sercom3
#define PAD_SPI2_TX           SPI_PAD_3_SCK_1
#define PAD_SPI2_RX           SERCOM_RX_PAD_0

static const uint8_t SS2   = 13 ;	 // also ignored
static const uint8_t MOSI2 = PIN_SPI_MOSI ;
static const uint8_t MISO2 = PIN_SPI_MISO ;
static const uint8_t SCK2  = PIN_SPI_SCK ;


/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (18u)
#define PIN_WIRE_SCL         (19u)
#define PERIPH_WIRE          sercom1
#define WIRE_IT_HANDLER      SERCOM1_Handler
#define WIRE_IT_HANDLER_0    SERCOM1_0_Handler
#define WIRE_IT_HANDLER_1    SERCOM1_1_Handler
#define WIRE_IT_HANDLER_2    SERCOM1_2_Handler
#define WIRE_IT_HANDLER_3    SERCOM1_3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * USB
 */
#define PIN_USB_HOST_ENABLE (25ul)
#define PIN_USB_DM          (26ul)
#define PIN_USB_DP          (27ul)

/*
 * I2S Interfaces
 */
#define I2S_INTERFACES_COUNT 0

#define I2S_DEVICE          0
// no I2S on G19!

// On-board QSPI Flash
#define EXTERNAL_FLASH_DEVICES   GD25Q64C
#define EXTERNAL_FLASH_USE_QSPI

//QSPI Pins
#define PIN_QSPI_SCK	(29u)
#define PIN_QSPI_CS	(30u)
#define PIN_QSPI_IO0	(31u)
#define PIN_QSPI_IO1	(32u)
#define PIN_QSPI_IO2	(33u)
#define PIN_QSPI_IO3	(34u)

#if !defined(VARIANT_QSPI_BAUD_DEFAULT)
  // TODO: meaningful value for this
  #define VARIANT_QSPI_BAUD_DEFAULT 5000000
#endif

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial

#endif /* _MONSTER_M4SK_ */

