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
 * Modified 29 January 2018 by Justin Mattair
 *   for MattairTech boards (www.mattairtech.com)
 *
 * See README.md for documentation and pin mapping information
 */

#ifndef _VARIANT_MATTAIRTECH_XENO_MINI_
#define _VARIANT_MATTAIRTECH_XENO_MINI_

/* The definitions here need the MattairTech SAMD core >=1.6.8.
 * The format is different than the stock Arduino SAMD core,
 * which uses ARDUINO_SAMD_VARIANT_COMPLIANCE instead.
 */
#define MATTAIRTECH_ARDUINO_SAMD_VARIANT_COMPLIANCE 10608

/*----------------------------------------------------------------------------
 *        Board/Clock Configuration
 *----------------------------------------------------------------------------*/

/* Master clock frequency (also Fcpu frequency). With the D51, this can be
 * either 120000000ul or 48000000ul (selected in the menu). See README.md.
 */
#define VARIANT_MCK                       (F_CPU)

/* If CLOCKCONFIG_HS_CRYSTAL is defined, then HS_CRYSTAL_FREQUENCY_HERTZ
 * must also be defined with the external crystal frequency in Hertz.
 */
#define HS_CRYSTAL_FREQUENCY_HERTZ      24000000UL

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

/* Define CORTEX_M_CACHE_ENABLED to enable the Cortex M cache (D51 only).
 */
#define CORTEX_M_CACHE_ENABLED

/* When the battery charger (CHG) is installed, the charge status is available on pin 18 (A18, STA),
 * which drives low (open-drain) when the battery is charging. Defining BATTERY_CHARGER_INSTALLED
 * will prevent pin A18 from being configured as an output, thus avoiding contention.
 */
#define BATTERY_CHARGER_INSTALLED

/* When the accelerometer/gyroscope/magnetometer (IMU) is installed, all three interrupt outputs
 * are tied together and connected to pin 32 (B22, INT), which by default drives low in a push-pull,
 * active-high configuration until it is configured otherwise (to open-drain, active-low for example).
 * Defining IMU_INSTALLED will prevent pin B22 from being configured as an output, thus avoiding
 * contention.
 */
#define IMU_INSTALLED

/* When the Vin regulator (5V) is installed, an analog comparator (Rev B only) is used to drive the Vbus
 * ideal diode enable pin (EN) low, which disconnects Vbus from VccH. The Vbus voltage divider, which is
 * available on pin 6 (A6, VU), is also connected to the same enable pin. Defining VIN_5V_REGULATOR_INSTALLED
 * will prevent pin A6 from being configured as an output, thus avoiding contention.
 */
#define VIN_5V_REGULATOR_INSTALLED


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"
#include "sam.h"

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
#define NUM_PIN_DESCRIPTION_ENTRIES     (36u)

#define PINS_COUNT           NUM_PIN_DESCRIPTION_ENTRIES
#define NUM_DIGITAL_PINS     PINS_COUNT
#define NUM_ANALOG_INPUTS    (14u)

#if (SAMD21 || SAMC21)
#define NUM_ANALOG_OUTPUTS   (1u)
#elif (SAMD51 || SAML21)
#define NUM_ANALOG_OUTPUTS   (2u)
#else
#error "variant.h: Unsupported chip"
#endif

#define analogInputToDigitalPin(p)  (p)

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

/* LEDs
 * None of these defines are currently used by the core.
 * The Xeno Mini onboard LED is on pin 34.
 * The RX and TX LEDs are not present.
 * You may optionally add them to any free pins.
 */
#define PIN_LED_13           (34u)
#define PIN_LED_RXL          (20u)
#define PIN_LED_TXL          (21u)
#define PIN_LED              PIN_LED_13
#define PIN_LED2             PIN_LED_RXL
#define PIN_LED3             PIN_LED_TXL
#define LED_BUILTIN          PIN_LED_13


/*
 * Analog pins
 */
#define PIN_A0               (0ul)
#define PIN_A1               (1ul)
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
#define PIN_A34              (34ul)
#define PIN_A35              (35ul)
#define PIN_DAC0             (2ul)
#if (SAMD51 || SAML21)
#define PIN_DAC1             (5ul)
#endif

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
static const uint8_t A34  = PIN_A34;
static const uint8_t A35  = PIN_A35;
static const uint8_t DAC0 = PIN_DAC0;
#if (SAMD51 || SAML21)
static const uint8_t DAC1 = PIN_DAC1;
#endif

#define ADC_RESOLUTION		12

// #define REMAP_ANALOG_PIN_ID(pin)	if ( pin < A0 ) pin += A0

/* Set default analog voltage reference */
#define VARIANT_AR_DEFAULT	AR_DEFAULT

/* Reference voltage pins (define even if not enabled with PIN_ATTR_REF in the PinDescription table) */
#define REFA_PIN    (3ul)
#define REFB_PIN    (4ul)
#if (SAMD51)
#define REFC_PIN    (6ul)
#endif


// The ATN pin may be used in the future as the first SPI chip select.
// On boards that do not have the Arduino physical form factor, it can to set to any free pin.
#define PIN_ATN              (27ul)
static const uint8_t ATN = PIN_ATN;


/*
 * Serial interfaces
 */
#if (defined(THREE_UART) && SAMD51)
  #error "variant.h: Only two UARTs are available with the D51. Please choose a TWO_UART_* option"
#endif

// Serial1
#define PIN_SERIAL1_TX       (0ul)
#define PIN_SERIAL1_RX       (1ul)
#define PAD_SERIAL1_TX       (UART_TX_PAD_0)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)

#define SERCOM_INSTANCE_SERIAL1       &sercom4

// Serial2
#define PIN_SERIAL2_TX       (12u)
#define PIN_SERIAL2_RX       (13u)
#define PAD_SERIAL2_TX       (UART_TX_PAD_0)
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_1)

#define SERCOM_INSTANCE_SERIAL2       &sercom2

// Serial3 (a third serial is not available with the D51 on this board)
#define PIN_SERIAL3_TX       (18ul)
#define PIN_SERIAL3_RX       (19ul)
#define PAD_SERIAL3_TX       (UART_TX_PAD_2)
#define PAD_SERIAL3_RX       (SERCOM_RX_PAD_3)

#define SERCOM_INSTANCE_SERIAL3       &sercom3


/*
 * SPI Interfaces
 */
#if defined(TWO_SPI)
#define SPI_INTERFACES_COUNT 2
#elif defined(THREE_SPI)
#define SPI_INTERFACES_COUNT 3
#else
#define SPI_INTERFACES_COUNT 1
#endif

#define PIN_SPI_MISO         (9u)
#define PIN_SPI_MOSI         (8u)
#define PIN_SPI_SCK          (11u)
#define PIN_SPI_SS           (10u)
#define PERIPH_SPI           sercom0
#define PAD_SPI_TX           SPI_PAD_0_SCK_3
#define PAD_SPI_RX           SERCOM_RX_PAD_1

static const uint8_t SS   = PIN_SPI_SS ;        // The SERCOM SS PAD is available on this pin but HW SS isn't used. Set here only for reference.
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

#if (SAMD51)
#define PIN_SPI1_MISO         (21u)
#define PIN_SPI1_MOSI         (23u)
#define PIN_SPI1_SCK          (22u)
#define PIN_SPI1_SS           (20u)
#define PAD_SPI1_TX           SPI_PAD_0_SCK_1
#define PAD_SPI1_RX           SERCOM_RX_PAD_3
#else
#define PIN_SPI1_MISO         (21u)
#define PIN_SPI1_MOSI         (22u)
#define PIN_SPI1_SCK          (23u)
#define PIN_SPI1_SS           (20u)
#define PAD_SPI1_TX           SPI_PAD_0_SCK_1
#define PAD_SPI1_RX           SERCOM_RX_PAD_3
#endif
#define PERIPH_SPI1           sercom5

static const uint8_t SS1   = PIN_SPI1_SS ;	// The SERCOM SS PAD is available on this pin but HW SS isn't used. Set here only for reference.
static const uint8_t MOSI1 = PIN_SPI1_MOSI ;
static const uint8_t MISO1 = PIN_SPI1_MISO ;
static const uint8_t SCK1  = PIN_SPI1_SCK ;


/*
 * Wire Interfaces
 */
#if defined(TWO_WIRE)
#define WIRE_INTERFACES_COUNT 2
#elif defined(THREE_WIRE)
#define WIRE_INTERFACES_COUNT 3
#else
#define WIRE_INTERFACES_COUNT 1
#endif

#define PIN_WIRE_SDA         (16u)
#define PIN_WIRE_SCL         (17u)
#define PERIPH_WIRE          sercom1
#if (SAMD51)
  #define WIRE_STOP_DETECTED_HANDLER      SERCOM1_0_Handler
  #define WIRE_ADDRESS_MATCH_HANDLER      SERCOM1_1_Handler
  #define WIRE_DATA_READY_HANDLER         SERCOM1_2_Handler
#else
  #define WIRE_IT_HANDLER      SERCOM1_Handler
#endif

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define PIN_WIRE1_SDA         (22u)
#define PIN_WIRE1_SCL         (23u)
#define PERIPH_WIRE1          sercom5
#if (SAMD51)
  #define WIRE1_STOP_DETECTED_HANDLER      SERCOM5_0_Handler
  #define WIRE1_ADDRESS_MATCH_HANDLER      SERCOM5_1_Handler
  #define WIRE1_DATA_READY_HANDLER         SERCOM5_2_Handler
#else
  #define WIRE1_IT_HANDLER      SERCOM5_Handler
#endif

static const uint8_t SDA1 = PIN_WIRE1_SDA;
static const uint8_t SCL1 = PIN_WIRE1_SCL;


/*
 * USB - Define PIN_USB_HOST_ENABLE to assert the defined pin to
 * PIN_USB_HOST_ENABLE_VALUE during startup. Leave undefined to disable this pin.
 */
#define PIN_USB_DM                      (24ul)
#define PIN_USB_DP                      (25ul)
//#define PIN_USB_HOST_ENABLE             (19ul)
#define PIN_USB_HOST_ENABLE_VALUE	0

/*
 * I2S Interfaces
 * SAMD51, PDM, and MCLK support will be added hopefully March 2018
 */

#if (SAMD51)
// On the SAMD51, device 0 is TX only using SDO (PIN_I2S_SD), and device 1 is RX only using SDI (PIN_I2S1_SD)
#define I2S_INTERFACES_COUNT  2
#define I2S_DEVICE            0
#define I2S1_DEVICE           1
#define I2S_CLOCK_GENERATOR   7
#define I2S1_CLOCK_GENERATOR  7
// TX
#define PIN_I2S_MCK         (8u)
#define PIN_I2S_SCK         (10u)
#define PIN_I2S_FS          (9u)
#define PIN_I2S_SD          (11u)       // SDO (TX)
// RX (must use clock 0)
#define PIN_I2S1_MCK         (8u)
#define PIN_I2S1_SCK         (10u)
#define PIN_I2S1_FS          (9u)
#define PIN_I2S1_SD          (14u)       // SDI (RX)

#else
#define I2S_INTERFACES_COUNT  1
#define I2S_DEVICE            0
//#define I2S1_DEVICE           1
#define I2S_CLOCK_GENERATOR   7
//#define I2S1_CLOCK_GENERATOR  8
//#define PIN_I2S_MCK         (9u)
#define PIN_I2S_SCK         (20u)
#define PIN_I2S_FS          (21u)
#define PIN_I2S_SD          (19u)
//#define PIN_I2S1_MCK         (10u)
//#define PIN_I2S1_SCK         (11u)
//#define PIN_I2S1_SD          (8u)
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

extern Uart Serial1;
extern Uart Serial2;
#if (SAMD21 || SAMC21 || SAML21)
extern Uart Serial3;
#endif

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
#if (!SAMC21)
  #define SERIAL_PORT_USBVIRTUAL      SerialUSB
#endif
// SERIAL_PORT_MONITOR seems to be used only by the USB Host library (as of 1.6.5).
// It normally allows debugging output on the USB programming port, while the USB host uses the USB native port.
// The programming port is connected to a hardware UART through a USB-Serial bridge (EDBG chip) on the Zero.
// Boards that do not have the EDBG chip will have to connect a USB-TTL serial adapter to 'Serial' to get
// the USB Host debugging output.
#if (SAMC21)
  #define SERIAL_PORT_MONITOR         Serial2
#else
  #define SERIAL_PORT_MONITOR         Serial1
#endif

// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#if (SAMC21)
  #define SERIAL_PORT_HARDWARE        Serial2
  #define SERIAL_PORT_HARDWARE_OPEN   Serial2
#else
  #define SERIAL_PORT_HARDWARE        Serial1
  #define SERIAL_PORT_HARDWARE_OPEN   Serial1
#endif

// The Xeno does not have the EDBG support chip, which provides a USB-UART bridge
// accessible using Serial (the Arduino serial monitor is normally connected to this).
// So, the USB virtual serial port (SerialUSB) must be used to communicate with the host.
// Because most sketches use Serial to print to the monitor, it is aliased to SerialUSB.
// Remember to use while(!Serial); to wait for a connection before Serial printing.
#if (SAMC21)
  #define Serial                      Serial1
#else
  // When USB CDC is enabled, Serial refers to SerialUSB, otherwise it refers to Serial1.
  #if defined(CDC_ONLY) || defined(CDC_HID) || defined(WITH_CDC)
    #define Serial                      SerialUSB
  #else
    #define Serial                      Serial1
  #endif
#endif

#endif /* _VARIANT_MATTAIRTECH_XENO_MINI_ */
