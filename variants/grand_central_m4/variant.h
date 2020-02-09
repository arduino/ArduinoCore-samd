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

#ifndef _VARIANT_GRAND_CENTRAL_M4_
#define _VARIANT_GRAND_CENTRAL_M4_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		(32768ul)

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
#define PINS_COUNT           (96)
#define NUM_DIGITAL_PINS     (53)
#define NUM_ANALOG_INPUTS    (19)
#define NUM_ANALOG_OUTPUTS   (2)
#define analogInputToDigitalPin(p) ((p < 8) ? 67 + (p) : (p < 16) ? 54 + (p) - 8 : (p < 18) ? 12 + (p) - 16 : (p == 18) ? 9 : -1)

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
#define PIN_LED_13          (13)
#define PIN_LED_RXL         (75)
#define PIN_LED_TXL         (76)
#define PIN_LED             PIN_LED_13
#define PIN_LED2            PIN_LED_RXL
#define PIN_LED3            PIN_LED_TXL
#define LED_BUILTIN         PIN_LED_13
#define PIN_NEOPIXEL		    (88)

/*
 * Analog pins
 */
#define PIN_A0              (67)
#define PIN_A1              (PIN_A0 + 1)
#define PIN_A2              (PIN_A0 + 2)
#define PIN_A3              (PIN_A0 + 3)
#define PIN_A4              (PIN_A0 + 4)
#define PIN_A5              (PIN_A0 + 5)
#define PIN_A6              (PIN_A0 + 6)
#define PIN_A7              (PIN_A0 + 7)

#define PIN_A8              (54)
#define PIN_A9              (PIN_A8 + 1)
#define PIN_A10             (PIN_A8 + 2)
#define PIN_A11             (PIN_A8 + 3)
#define PIN_A12             (PIN_A8 + 4)
#define PIN_A13             (PIN_A8 + 5)
#define PIN_A14             (PIN_A8 + 6)
#define PIN_A15             (PIN_A8 + 7)

#define PIN_DAC0            PIN_A0
#define PIN_DAC1            PIN_A1

static const uint8_t A0   = PIN_A0;
static const uint8_t A1   = PIN_A1;
static const uint8_t A2   = PIN_A2;
static const uint8_t A3   = PIN_A3;
static const uint8_t A4   = PIN_A4;
static const uint8_t A5   = PIN_A5;
static const uint8_t A6   = PIN_A6;
static const uint8_t A7   = PIN_A7;

static const uint8_t A8   = PIN_A8;
static const uint8_t A9   = PIN_A9;
static const uint8_t A10  = PIN_A10;
static const uint8_t A11  = PIN_A11;
static const uint8_t A12  = PIN_A12;
static const uint8_t A13  = PIN_A13;
static const uint8_t A14  = PIN_A14;
static const uint8_t A15  = PIN_A15;

static const uint8_t DAC0 = PIN_DAC0;
static const uint8_t DAC1 = PIN_DAC1;

#define ADC_RESOLUTION		  12

// Other pins
#define PIN_ATN             (39)
static const uint8_t ATN = PIN_ATN;

/*
 * Serial interfaces
 */

// Serial1
#define PIN_SERIAL1_RX      (0)
#define PIN_SERIAL1_TX      (1)
#define PAD_SERIAL1_TX      (UART_TX_PAD_0)
#define PAD_SERIAL1_RX      (SERCOM_RX_PAD_1)

// Serial2
#define PIN_SERIAL2_RX      (19)
#define PIN_SERIAL2_TX      (18)
#define PAD_SERIAL2_TX      (UART_TX_PAD_0)
#define PAD_SERIAL2_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL2		  sercom4

// Serial3
#define PIN_SERIAL3_RX      (17)
#define PIN_SERIAL3_TX      (16)
#define PAD_SERIAL3_TX      (UART_TX_PAD_0)
#define PAD_SERIAL3_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL3		  sercom1

// Serial4
#define PIN_SERIAL4_RX      (15)
#define PIN_SERIAL4_TX      (14)
#define PAD_SERIAL4_TX      (UART_TX_PAD_0)
#define PAD_SERIAL4_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL4		  sercom5
/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 2

#define PIN_SPI_MISO        (64)
#define PIN_SPI_MOSI        (66)
#define PIN_SPI_SCK         (65)
#define PERIPH_SPI          sercom7
#define PAD_SPI_TX          SPI_PAD_0_SCK_1
#define PAD_SPI_RX          SERCOM_RX_PAD_3

static const uint8_t SS	  = (53);
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

#define PIN_SPI1_MISO       (80)
#define PIN_SPI1_MOSI       (82)
#define PIN_SPI1_SCK        (81)
#define PIN_SPI1_SS         (83)
#define PERIPH_SPI1         sercom2
#define PAD_SPI1_TX         SPI_PAD_0_SCK_1
#define PAD_SPI1_RX         SERCOM_RX_PAD_3

static const uint8_t SS1	 = PIN_SPI1_SS;
static const uint8_t MOSI1 = PIN_SPI1_MOSI;
static const uint8_t MISO1 = PIN_SPI1_MISO;
static const uint8_t SCK1  = PIN_SPI1_SCK;

// Needed for SD library
#define SDCARD_SPI          SPI1
#define SDCARD_MISO_PIN     PIN_SPI1_MISO
#define SDCARD_MOSI_PIN     PIN_SPI1_MOSI
#define SDCARD_SCK_PIN      PIN_SPI1_SCK
#define SDCARD_SS_PIN       PIN_SPI1_SS

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA        (62)
#define PIN_WIRE_SCL        (63)
#define PERIPH_WIRE         sercom3
#define WIRE_IT_HANDLER     SERCOM3_Handler
#define WIRE_IT_HANDLER_0   SERCOM3_0_Handler
#define WIRE_IT_HANDLER_1   SERCOM3_1_Handler
#define WIRE_IT_HANDLER_2   SERCOM3_2_Handler
#define WIRE_IT_HANDLER_3   SERCOM3_3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define PIN_WIRE1_SDA       (25)
#define PIN_WIRE1_SCL       (24)
#define PERIPH_WIRE1        sercom6
#define WIRE1_IT_HANDLER    SERCOM6_Handler
#define WIRE1_IT_HANDLER_0  SERCOM6_0_Handler
#define WIRE1_IT_HANDLER_1  SERCOM6_1_Handler
#define WIRE1_IT_HANDLER_2  SERCOM6_2_Handler
#define WIRE1_IT_HANDLER_3  SERCOM6_3_Handler

static const uint8_t SDA1 = PIN_WIRE1_SDA;
static const uint8_t SCL1 = PIN_WIRE1_SCL;

/*
 * USB
 */
#define PIN_USB_HOST_ENABLE (77)
#define PIN_USB_DM          (78)
#define PIN_USB_DP          (79)

/*
 * I2S Interfaces
 */
#define I2S_INTERFACES_COUNT 1

#define I2S_DEVICE          0
#define I2S_CLOCK_GENERATOR 3

#define PIN_I2S_SDO         (32)
#define PIN_I2S_SDI         (31)
#define PIN_I2S_SCK         PIN_SERIAL4_TX
#define PIN_I2S_FS          (33)
#define PIN_I2S_MCK			    PIN_SERIAL4_RX

// On-board QSPI Flash
#define EXTERNAL_FLASH_DEVICES   GD25Q16C
#define EXTERNAL_FLASH_USE_QSPI

//QSPI Pins
#define PIN_QSPI_SCK	      (89)
#define PIN_QSPI_CS		      (90)
#define PIN_QSPI_IO0	      (91)
#define PIN_QSPI_IO1	      (92)
#define PIN_QSPI_IO2	      (93)
#define PIN_QSPI_IO3	      (94)

//PCC Pins
#define PIN_PCC_DEN1        (26)
#define PIN_PCC_DEN2        (27)
#define PIN_PCC_CLK         (28)
#define PIN_PCC_XCLK	      (29)
#define PIN_PCC_D0          (37)
#define PIN_PCC_D1          (36)
#define PIN_PCC_D2          (35)
#define PIN_PCC_D3          (34)
#define PIN_PCC_D4          (33)
#define PIN_PCC_D5          (32)
#define PIN_PCC_D6          (31)
#define PIN_PCC_D7          (30)
#define PIN_PCC_D8          (39)
#define PIN_PCC_D9          (38)
#define PIN_PCC_D10         (41)
#define PIN_PCC_D11         (40)
#define PIN_PCC_D12         (43)
#define PIN_PCC_D13         (42)

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
extern SERCOM sercom6;
extern SERCOM sercom7;

extern Uart Serial1;

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
// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

#endif /* _VARIANT_GRAND_CENTRAL_M4_ */
