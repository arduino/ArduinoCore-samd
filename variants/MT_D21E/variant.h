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
  Modified 20 July 2015 by Justin Mattair
     for MattairTech MT-D21E boards (www.mattairtech.com)
*/

#ifndef _VARIANT_MATTAIRTECH_MT_D21E_
#define _VARIANT_MATTAIRTECH_MT_D21E_

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		(32768ul)

/** Master clock frequency */
#define VARIANT_MCK			  (48000000ul)

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
#define NUM_PIN_DESCRIPTION_ENTRIES     (32u)

#define PINS_COUNT           (32u)
#define NUM_DIGITAL_PINS     (32u)
#define NUM_ANALOG_INPUTS    (10u)
#define NUM_ANALOG_OUTPUTS   (1u)

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( (g_APinDescription[P].ulPinAttribute & PIN_ATTR_TIMER_PWM) == PIN_ATTR_TIMER_PWM )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// Interrupts
#define digitalPinToInterrupt(P)   ( g_APinDescription[P].ulExtInt )


/* LEDs
 * None of these defines are currently used by the core.
 * The MT-D21E onboard LED is on pin 28.
 * The RX and TX LEDs are not present.
 * You may optionally add them to any free pins.
 */
#define PIN_LED_13           (28u)
#define PIN_LED_RXL          (30u)
#define PIN_LED_TXL          (31u)
#define PIN_LED              PIN_LED_13
#define PIN_LED2             PIN_LED_RXL
#define PIN_LED3             PIN_LED_TXL
#define LED_BUILTIN          PIN_LED_13

/* Buttons
 * Note that Button B is connected to Reset by default.
 * A solder jumper can be changed to route Button B to pin 31 instead.
 * If the debouncing capacitor is connected (default), delay reading the
 * pin at least 6ms after turning on the pullup to allow the capacitor to charge.
 */
#define BUTTON_A             (27u)
#define BUTTON_B             (31u)
#define BUTTON_BUILTIN       BUTTON_A


/*
 * Analog pins
 */
#define PIN_A2               (2ul)
#define PIN_A3               (3ul)
#define PIN_A4               (4ul)
#define PIN_A5               (5ul)
#define PIN_A6               (6ul)
#define PIN_A7               (7ul)
#define PIN_A8               (8ul)
#define PIN_A9               (9ul)
#define PIN_A10              (10ul)
#define PIN_A11              (11ul)

static const uint8_t A2   = PIN_A2 ;
static const uint8_t A3   = PIN_A3 ;
static const uint8_t A4   = PIN_A4 ;
static const uint8_t A5   = PIN_A5 ;
static const uint8_t A6   = PIN_A6 ;
static const uint8_t A7   = PIN_A7 ;
static const uint8_t A8   = PIN_A8 ;
static const uint8_t A9   = PIN_A9 ;
static const uint8_t A10  = PIN_A10 ;
static const uint8_t A11  = PIN_A11 ;

#define ADC_RESOLUTION		12

#define REMAP_ANALOG_PIN_ID	while(0)
// #define REMAP_ANALOG_PIN_ID	if ( ulPin < A0 ) ulPin += A0

/* Set default analog voltage reference */
#define VARIANT_AR_DEFAULT	AR_DEFAULT

/* Reference voltage pins (define even if not enabled with PIN_ATTR_REF in the PinDescription table) */
#define REFA_PIN    (3ul)
#define REFB_PIN    (4ul)


// The ATN pin may be used in the future as the first SPI chip select.
// On boards that do not have the Arduino physical form factor, it can to set to any free pin.
#define PIN_ATN              (15ul)
static const uint8_t ATN = PIN_ATN;


/*
 * Serial interfaces
 */
// Serial1
#define PIN_SERIAL1_RX       (11ul)
#define PIN_SERIAL1_TX       (10ul)
#define PAD_SERIAL1_TX       (UART_TX_PAD_2)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_3)

#define SERCOM_INSTANCE_SERIAL1       &sercom0

// Serial2
#define PIN_SERIAL2_RX       (15ul)
#define PIN_SERIAL2_TX       (14ul)
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_3)

#define SERCOM_INSTANCE_SERIAL2       &sercom2


/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI1_MISO         (22u)
#define PIN_SPI1_MOSI         (18u)
#define PIN_SPI1_SCK          (19u)
#define PIN_SPI1_SS           (23u)
#define PIN_SPI_MISO         PIN_SPI1_MISO
#define PIN_SPI_MOSI         PIN_SPI1_MOSI
#define PIN_SPI_SCK          PIN_SPI1_SCK
#define PIN_SPI_SS           PIN_SPI1_SS

static const uint8_t SS	  = PIN_SPI_SS ;	// The SERCOM SS PAD is available on this pin but HW SS isn't used. Set here only for reference.
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

#define SERCOM_INSTANCE_SPI           &sercom3


/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE1_SDA         (16u)
#define PIN_WIRE1_SCL         (17u)
#define PIN_WIRE_SDA          PIN_WIRE1_SDA
#define PIN_WIRE_SCL          PIN_WIRE1_SCL

#define SERCOM_INSTANCE_WIRE           &sercom1

#define SERCOM_WIRE_HANDLER_MACRO \
void SERCOM1_Handler(void) { \
	Wire.onService(); \
}


/*
 * USB
 */
#define PIN_USB_DM                      (24ul)
#define PIN_USB_DP                      (25ul)
#define PIN_USB_HOST_ENABLE             (14ul)
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
extern SERCOM sercom3;

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

// The MT-D21E does not have the EDBG support chip, which provides a USB-UART bridge
// accessible using Serial (the Arduino serial monitor is normally connected to this).
// So, the USB virtual serial port (SerialUSB) must be used to communicate with the host.
// Because most sketches use Serial to print to the monitor, it is aliased to SerialUSB.
// Remember to use while(!Serial); to wait for a connection before Serial printing.
#define Serial                      SerialUSB

#endif /* _VARIANT_ARDUINO_ZERO_ */

