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

/*
 * Modified 16 June 2018 by Justin Mattair (www.mattairtech.com)
 * See README.md for documentation and pin mapping information
 */

#ifndef _VARIANT_MATTAIRTECH_MT_D11_
#define _VARIANT_MATTAIRTECH_MT_D11_

/* This variant requires the MattairTech SAM D|L|C Core for Arduino >= 1.6.18-beta-b1.
 * The format is different than the stock Arduino SAMD core,
 * which uses ARDUINO_SAMD_VARIANT_COMPLIANCE instead.
 */
#define MATTAIRTECH_ARDUINO_SAMD_VARIANT_COMPLIANCE 10618

/*----------------------------------------------------------------------------
 *        Clock Configuration
 *----------------------------------------------------------------------------*/

/** Master clock frequency (also Fcpu frequency) */
#define VARIANT_MCK		(48000000ul)

/* If CLOCKCONFIG_HS_CRYSTAL is defined, then HS_CRYSTAL_FREQUENCY_HERTZ
 * must also be defined with the external crystal frequency in Hertz.
 */
#define HS_CRYSTAL_FREQUENCY_HERTZ	16000000UL

/* If the PLL is used (CLOCKCONFIG_32768HZ_CRYSTAL, or CLOCKCONFIG_HS_CRYSTAL
 * defined), then PLL_FRACTIONAL_ENABLED can be defined, which will result in
 * a more accurate 48MHz output frequency at the expense of increased jitter.
 */
//#define PLL_FRACTIONAL_ENABLED

/* If both PLL_FAST_STARTUP and CLOCKCONFIG_HS_CRYSTAL are defined, the crystal
 * will be divided down to 1MHz - 2MHz, rather than 32KHz - 64KHz, before being
 * multiplied by the PLL. This will result in a faster lock time for the PLL,
 * however, it will also result in a less accurate PLL output frequency if the
 * crystal is not divisible (without remainder) by 1MHz. In this case, define
 * PLL_FRACTIONAL_ENABLED as well.
 */
//#define PLL_FAST_STARTUP

/* The fine calibration value for DFLL open-loop mode is defined here.
 * The coarse calibration value is loaded from NVM OTP (factory calibration values).
 */
#define NVM_SW_CALIB_DFLL48M_FINE_VAL     (512)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"
#include "sam.h"
#include "../../config.h"

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
#define PIN_MAP_STANDARD
//#define PIN_MAP_COMPACT

#if defined PIN_MAP_STANDARD
  #define NUM_PIN_DESCRIPTION_ENTRIES     (32u)
#elif defined PIN_MAP_COMPACT
  #define NUM_PIN_DESCRIPTION_ENTRIES     (17u)
#else
  #error "variant.h: You must set PIN_MAP_STANDARD or PIN_MAP_COMPACT"
#endif

#define PINS_COUNT           NUM_PIN_DESCRIPTION_ENTRIES
#define NUM_DIGITAL_PINS     PINS_COUNT
#define NUM_ANALOG_INPUTS    (10u)
#define NUM_ANALOG_OUTPUTS   (1u)
#define analogInputToDigitalPin(p)  (p)

#define digitalPinToPort(P)        ( &(PORT->Group[GetPort(P)]) )
#define digitalPinToBitMask(P)     ( 1 << GetPin(P) )

#if defined(PIN_DESCRIPTION_TABLE_SIMPLE)
  #define digitalPinHasPWM(P)        ( g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )
#else
  #define digitalPinHasPWM(P)        ( (g_APinDescription[P].ulPinAttribute & PIN_ATTR_TIMER_PWM) == PIN_ATTR_TIMER_PWM )
#endif

//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

/* LEDs
 * None of these defines are currently used by the core.
 * The MT-D11 onboard LED is on pin 16.
 * The RX and TX LEDs are not present.
 * You may optionally add them to any free pins.
 */
#if defined PIN_MAP_STANDARD
#define PIN_LED_13           (16u)
#define PIN_LED_RXL          (10u)
#define PIN_LED_TXL          (11u)
#elif defined PIN_MAP_COMPACT
#define PIN_LED_13           (10u)
#define PIN_LED_RXL          (6u)
#define PIN_LED_TXL          (7u)
#endif

#define PIN_LED              PIN_LED_13
#define PIN_LED2             PIN_LED_RXL
#define PIN_LED3             PIN_LED_TXL
#define LED_BUILTIN          PIN_LED_13

/* Buttons
 * Note that Button A is connected to Reset by default.
 * A solder jumper can be changed to route Button A to pin 15 instead.
 * Thre is a debouncing capacitor connected, so delay reading the pin
 * at least 6ms after turning on the pullup to allow the capacitor to charge.
 */
#if defined PIN_MAP_STANDARD
#define BUTTON               (15u)
#elif defined PIN_MAP_COMPACT
#define BUTTON               (9u)
#endif

#define BUTTON_BUILTIN       BUTTON


/*
 * Analog pins
 */
#if defined PIN_MAP_STANDARD
#define PIN_A2               (2ul)
#define PIN_A3               (3ul)
#define PIN_A4               (4ul)
#define PIN_A5               (5ul)
#define PIN_A6               (6ul)
#define PIN_A7               (7ul)
#define PIN_A10              (10ul)
#define PIN_A11              (11ul)
#define PIN_A14              (14ul)
#define PIN_A15              (15ul)
#define PIN_DAC0             (2ul)
#elif defined PIN_MAP_COMPACT
#define PIN_A2               (0ul)
#define PIN_A3               (1ul)
#define PIN_A4               (2ul)
#define PIN_A5               (3ul)
#define PIN_A6               (4ul)
#define PIN_A7               (5ul)
#define PIN_A10              (6ul)
#define PIN_A11              (7ul)
#define PIN_A14              (8ul)
#define PIN_A15              (9ul)
#define PIN_DAC0             (0ul)
#endif

static const uint8_t A2   = PIN_A2;
static const uint8_t A3   = PIN_A3;
static const uint8_t A4   = PIN_A4;
static const uint8_t A5   = PIN_A5;
static const uint8_t A6   = PIN_A6;
static const uint8_t A7   = PIN_A7;
static const uint8_t A10  = PIN_A10;
static const uint8_t A11  = PIN_A11;
static const uint8_t A14  = PIN_A14;
static const uint8_t A15  = PIN_A15;
static const uint8_t DAC0 = PIN_DAC0;

#define ADC_RESOLUTION		12

// #define REMAP_ANALOG_PIN_ID(pin)	if ( pin < A0 ) pin += A0

/* Set default analog voltage reference */
#define VARIANT_AR_DEFAULT	AR_DEFAULT

/* Reference voltage pins (define even if not enabled with PIN_ATTR_REF in the PinDescription table) */
#if defined PIN_MAP_STANDARD
#define REFA_PIN    (3ul)
#define REFB_PIN    (4ul)
#elif defined PIN_MAP_COMPACT
#define REFA_PIN    (1ul)
#define REFB_PIN    (2ul)
#endif


// The ATN pin may be used in the future as the first SPI chip select.
// On boards that do not have the Arduino physical form factor, it can to set to any free pin.
#if defined PIN_MAP_STANDARD
#define PIN_ATN              (15ul)
#elif defined PIN_MAP_COMPACT
#define PIN_ATN              (9ul)
#endif
static const uint8_t ATN = PIN_ATN;


/*
 * Serial interfaces
 */
// Serial1
#if defined PIN_MAP_STANDARD
#define PIN_SERIAL1_RX       (31ul)
#define PIN_SERIAL1_TX       (30ul)
#elif defined PIN_MAP_COMPACT
#define PIN_SERIAL1_RX       (16ul)
#define PIN_SERIAL1_TX       (15ul)
#endif

#define PAD_SERIAL1_TX       (UART_TX_PAD_2)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_3)
#define SERCOM_INSTANCE_SERIAL1       &sercom1

// Serial2
#if defined PIN_MAP_STANDARD
#define PIN_SERIAL2_RX       (11ul)
#define PIN_SERIAL2_TX       (10ul)
#elif defined PIN_MAP_COMPACT
#define PIN_SERIAL2_RX       (7ul)
#define PIN_SERIAL2_TX       (6ul)
#endif

#define PAD_SERIAL2_TX       (UART_TX_PAD_2)
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_3)
#define SERCOM_INSTANCE_SERIAL2       &sercom0


/*
 * SPI Interfaces
 */
#if defined(ONE_SPI)
#define SPI_INTERFACES_COUNT 1
#else
#define SPI_INTERFACES_COUNT 0
#endif

#if defined PIN_MAP_STANDARD
#define PIN_SPI_MISO         (14u)
#define PIN_SPI_MOSI         (10u)
#define PIN_SPI_SCK          (11u)
#define PIN_SPI_SS           (15u)
#elif defined PIN_MAP_COMPACT
#define PIN_SPI_MISO         (8u)
#define PIN_SPI_MOSI         (6u)
#define PIN_SPI_SCK          (7u)
#define PIN_SPI_SS           (9u)
#endif

#define PERIPH_SPI           sercom0
#define PAD_SPI_TX           SPI_PAD_2_SCK_3
#define PAD_SPI_RX           SERCOM_RX_PAD_0

static const uint8_t SS	  = PIN_SPI_SS ;	// The SERCOM SS PAD is available on this pin but HW SS isn't used. Set here only for reference.
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;


/*
 * Wire Interfaces
 */
#if defined(ONE_WIRE)
#define WIRE_INTERFACES_COUNT 1
#else
#define WIRE_INTERFACES_COUNT 0
#endif

#if defined PIN_MAP_STANDARD
#define PIN_WIRE_SDA         (22u)
#define PIN_WIRE_SCL         (23u)
#elif defined PIN_MAP_COMPACT
#define PIN_WIRE_SDA         (12u)
#define PIN_WIRE_SCL         (13u)
#endif

#define PERIPH_WIRE          sercom2
#define WIRE_IT_HANDLER      SERCOM2_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;


/*
 * USB - Define PIN_USB_HOST_ENABLE to assert the defined pin to
 * PIN_USB_HOST_ENABLE_VALUE during startup. Leave undefined to disable this pin.
 */
#if defined PIN_MAP_STANDARD
#define PIN_USB_DM                      (24ul)
#define PIN_USB_DP                      (25ul)
//#define PIN_USB_HOST_ENABLE             (17ul)
#elif defined PIN_MAP_COMPACT
// USB pins not directly accessible using PIN_MAP_COMPACT
#define PIN_USB_DM                      (0ul)
#define PIN_USB_DP                      (0ul)
//#define PIN_USB_HOST_ENABLE             (0ul)
#endif

#define PIN_USB_HOST_ENABLE_VALUE	HIGH

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

extern Uart Serial1;
extern Uart Serial2;

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
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
// SERIAL_PORT_MONITOR seems to be used only by the USB Host library (as of 1.6.5).
// It normally allows debugging output on the USB programming port, while the USB host uses the USB native port.
// The programming port is connected to a hardware UART through a USB-Serial bridge (EDBG chip) on the Zero.
// Boards that do not have the EDBG chip will have to connect a USB-TTL serial adapter to 'Serial' to get
// the USB Host debugging output.
#define SERIAL_PORT_MONITOR         Serial1
// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

// The MT-D11 does not have the EDBG support chip, which provides a USB-UART bridge
// accessible using Serial (the Arduino serial monitor is normally connected to this).
// So, the USB virtual serial port (SerialUSB) must be used to communicate with the host.
// Because most sketches use Serial to print to the monitor, it is aliased to SerialUSB.
// Remember to use while(!Serial); to wait for a connection before Serial printing.

// When USB CDC is enabled, Serial refers to SerialUSB, otherwise it refers to Serial1.
#if defined(CDC_ONLY) || defined(CDC_HID) || defined(WITH_CDC)
#define Serial                      SerialUSB
#else
#define Serial                      Serial1
#endif

#endif /* _VARIANT_MATTAIRTECH_MT_D11_ */
