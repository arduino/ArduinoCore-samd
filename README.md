# MattairTech Arduino SAMD Core

This is a fork from arduino/ArduinoCore-samd on GitHub. This will be used to maintain Arduino support
for SAMD boards including the MattairTech MT-D21E and the MT-D11 (see https://www.mattairtech.com/).
It primarily adds support for new devices as well as a more flexible pin configuration / mapping system.
This core is intended to be installed using the Boards Manager (see below).


## Summary
Feature                 |	Value
------------------------|------------------------------------------------------------------------------
Microcontroller		|	ATSAMD21ExxA, 32-Bit ARM Cortex M0+
Clock Speed		|	48 MHz
Flash Memory		|	256 KB (D21E18A) / 128 KB (D21E17A) / 64 KB (D21E16A) / 32 KB (D21E15A)
SRAM			|	32 KB (D21E18A) / 16 KB (D21E17A) / 8 KB (D21E16A) / 4 KB (D21E15A)
EEPROM			|	None (emulation may be available in the future)
Digital Pins		|	22
Analog Input Pins	|	10, 12-bit ADC channels
Analog Output Pins	|	1, 10-bit DAC
PWM Output Pins		|	12
External Interrupts	|	15 (1 NMI)
UART (Serial)		|	2
SPI			|	1
I2C (TWI)		|	1
Operating Voltage	|	3.3V (Do not connect voltages higher than 3.3V!)
DC Current per I/O Pin	|	7 mA


## Pin Configurations

Most pins have multiple configurations available (even analog pins). For example, pin A10 can be an analog input, a
PWM output, Digital I/O, or the TX pin of 'Serial1'. These always reference the pin number printed on the board but
without the 'A' (with the usable pins starting at 2). DO NOT connect voltages higher than 3.3V!

```
============================================ MattairTech MT-D21E (ATsamd21eXXa) =======================================
Comm/Other INT      PWM    Digital    Analog                               Digital   PWM       INT    Comm/other
=======================================================================================================================
                                                  -----------------------
Xin32                                            |   A0            RST   |                            Reset
Xout32                                           |   A1            NC    |
                             2     2 (ADC0, DAC) |   A2            NC    |
                             3     3 (ADC1, REF) |   A3            A31   |   31   TCC1/WO[1]  INT11   Button B / SWD IO
          INT4               4     4 (ADC4)      |   A4            A30   |   30   TCC1/WO[0]  INT10   SWD CLK
          INT5               5     5 (ADC5)      |   A5            NC    |
                             6     6 (ADC6)      |   A6            A28   |   28               INT8    LED
VDIV                         7     7 (ADC7)      |   A7            A27   |   27               INT15   Button A
          INTNMI TCC0/WO[0]  8     8 (ADC16)     |   A8            A23   |   23   TC4/WO[1]   INT7    SPI SS
          INT9   TCC0/WO[1]  9     9 (ADC17)     |   A9            A22   |   22   TC4/WO[0]   INT6    SPI MISO
TX (1)           TCC0/WO[2]  10    10 (ADC18)    |   A10           A19   |   19               INT3    SPI SCK
RX (1)           TCC0/WO[3]  11    11 (ADC19)    |   A11           A18   |   18               INT2    SPI MOSI
TX (2)    INT14  TC3/WO[0]   14                  |   A14           A17   |   17   TCC2/WO[1]  INT1    I2C/SCL
RX (2)           TC3/WO[1]   15                  |   A15           A16   |   16   TCC2/WO[0]  INT0    I2C/SDA
                                                 |   NC            NC    |
                                                 |   NC            NC    |
                                                 |   Vbus          3.3V  |
USB D-                                           |   A24-  _____   Vcc   |
USB D+                                           |   A25+ |     |  Vin   |
                                                 |   Gnd  | USB |  Gnd   |
                                                  -----------------------
```

#### All pins operate at 3.3 volts. DO NOT connect voltages higher than 3.3V!

* Digital: All 22 pins can be used for general purpose I/O (INPUT, OUTPUT, INPUT_PULLUP, and INPUT_PULLDOWN).
  * Each pin can source or sink a maximum of 7 mA (when PER_ATTR_DRIVE_STRONG is set for the pin).
  * Internal pull-up and pull-down resistors of 20-60 Kohms (40Kohm typ., disconnected by default).
  * Use the pinMode(), digitalWrite(), and digitalRead() functions.
* Analog Inputs: 10 pins can be configured as ADC analog inputs.
  * These are available on pins 2 through 11 using the analogRead() function.
  * These pins can be used for GPIO and other digital functions (ie. pwm and serial) as well.
  * Each pin provides 10 bits of resolution (1024 values) by default.
  * 12-bit resolution supported by using the analogReadResolution() function.
  * Each pin measures from ground to 3.3 volts.
  * The upper end of the measurement range can be changed using the AREF pin and the analogReference() function.
* DAC: One analog output is available on pin 2.
  * Provides a 10-bit voltage output with the analogWrite() function.
* PWM: 12 pins can be configured as PWM outputs.
  * Available on pins 8, 9, 10, 11, 14, 15, 16, 17, 22, 23, 30, and 31 using the analogWrite() function.
  * Each pin provides 8 bits of resolution (256 values) by default.
  * 12-bit resolution supported by using the analogWriteResolution() function.
* External Interrupts: 15 pins can be configured with external interrupts.
  * Available on all the pins except pins 2, 3, 6, 7, 10, 11, and 15 using the attachInterrupt() function.
* Serial: 2 pairs of pins can be configured for TTL serial I/O.
  * Serial1: pin 11 (RX) and pin 10 (TX).
  * Serial2: pin 15 (RX) and pin 14 (TX).
* SPI: 3 or 4 pins can be configured for SPI I/O.
  * Pin 18 (MOSI), pin 19 (SCK), pin 22 (MISO), and optionally pin 23 (SS, not currently used).
  * SPI communication using the SPI library.
* TWI (I2C): 2 pins can be configured for TWI I/O.
  * Pin 16 (SDA) and pin 17 (SCL).
  * TWI communication using the Wire library.
* LED: One pin can be configured to light the onboard LED.
  * Pin 28 (LED_BUILTIN). Bring the pin HIGH to turn the LED on. The pullup is disabled on this pin.
* Button: One pin can be configured to read the onboard Button A.
  * Pin 27 (BUTTON_BUILTIN). Pressing the button will bring the pin LOW. The pullup must be enabled first.
  * If the debouncing capacitor is connected, delay reading the pin at least 6ms after turning on the pullup.
* AREF: One pin can be configured as an AREF analog input.
  * The upper end of the analog measurement range can be changed using the analogReference() function.
* Reset: Bring this line LOW to reset the microcontroller.

#### MT-D21E Board Configuration

* The 32.768KHz crystal is used by the Arduino core, so it MUST be connected via the solder jumpers.
* Note that the sketch may still run without the crystal attached, but the clock speed will be very inaccurate.
* The 16MHz crystal is not used. It should be disconnected via the solder jumpers.
* The I2C (TWI) pullup resistors should be enabled via the solder jumpers.
* The LED should be enabled via the solder jumper.
* Button A should be connected via the solder jumper. The debouncing capacitor should also be connected.
* Button B is connected to the Reset pin by default, but can be connected to pin 31 via the solder jumper.
* A reference voltage can be connected to AREF. In this case, the capacitors should be enabled via the solder jumper.


## Serial Monitor

To print to the Serial Monitor over USB, use 'Serial'. Serial points to SerialUSB (Serial1 and Serial2 are UARTS).
Unlike most Arduino boards (ie. Uno), SAMD boards do not automatically reset when the serial monitor is opened.
To see what your sketch outputs to the serial monitor from the beginning, the sketch must wait for the SerialUSB
port to open first. Add the following to setup():

```
while (!Serial) ;
```

Remember that if the sketch needs to run without SerialUSB connected, another approach must be used.

You can also reset the board manually with the Reset button if you wish to restart your sketch. However, pressing
the Reset button will reset the SAMD chip, which in turn will reset USB communication. This interruption means
that if the serial monitor is open, it will be necessary to close and re-open it to restart communication.



## Installation

### Driver Installation

#### Windows

There are two "drivers", a CDC only driver for the bootloader, and a CDC-HID driver for Arduino sketches (optional).
The drivers are signed and support both 32 and 64 bit versions of Windows XP (SP3), Vista, 7, and 8.

1. If you do not already have the SAM-BA bootloader installed, see below.
2. Download https://www.mattairtech.com/software/MattairTech_CDC_Driver_Signed.zip and unzip into any folder.
3. Plug in the board while holding down button A to enter the bootloader. The LED should light.
4. Windows will detect the board. Point the installer to the folder from above to install the bootloader driver.
5. If you don't intend on using Arduino, you can skip the rest of this list. See Using Bossac Standalone below.
6. If you do not already have the test firmware installed, see Using Bossac Standalone below.
7. Press the reset button to run the test firmware (blink sketch with CDC-HID).
8. Windows will detect the board. Point the installer to the folder from above to install the sketch driver.
9. Continue with SAMD Core Installation below.

#### Linux

0. No driver installation is needed.
1. On some distros, you may need to add your user to the same group as the port (ie: dialout) and/or set udev rules.
2. You MAY have to install and use Arduino as the root user in order to get reliable access to the serial port.
   * This is true even when group permissions are set correctly, and it may fail after previously working.
   * You can also create/modify a udev rule to set permissions on the port so *everyone* can read / write.
3. Continue with SAMD Core Installation below.

#### OS X

1. As of this writing, only the 256 KB chip variants work with the OS X version of the upload tool, bossac.
2. First, you will need to open boards.txt and change mattairtech_mt_d21e_bl8k.upload.tool to equal arduino:bossac.
3. No driver installation is needed. You may get a dialog box asking if you wish to open the “Network Preferences”:
   * Click the "Network Preferences..." button, then click "Apply".
   * The board will show up as “Not Configured”, but it will work fine.
4. Continue with SAMD Core Installation below.

### SAMD Core Installation

1. The MattairTech SAMD Core requires Arduino 1.6.5+.
2. In the Arduino IDE 1.6.5+, click File->Preferences.
3. Click the button next to Additional Boards Manager URLs.
4. Add https://www.mattairtech.com/software/arduino/package_MattairTech_index.json.
5. Save preferences, then open the Boards Manager.
6. Install the Arduino SAMD Boards package.
7. Install the MattairTech SAMD Boards package.
8. Close Boards Manager, then click Tools->Board->MattairTech MT-D21E.
9. Select the processor with the now visible Tools->Processor menu.
10. If you do not already have the bootloader or blink sketch installed, see SAM-BA USB CDC Bootloader below.
11. Plug in the board. The blink sketch should be running.
12. Click Tools->Port and choose the COM port.
13. You can now upload your own sketch.



## SAM-BA USB CDC Bootloader (Arduino Zero compatible)

The SAM-BA bootloader has both a CDC USB interface, and a UART interface (TX: pin 10, RX: pin 11). It is
compatible with the Arduino IDE (Zero compatible), or it can be used with the Bossac tool standalone. Under
Arduino, auto-reset is supported (automatically runs the bootloader while the sketch is running) as well as
automatic return freom reset. The SAM-BA bootloader described here adds to the Arduino version, which in
turn is based on the bootloader from Atmel. The Arduino version added several features, including three
new commands (Arduino Extended Capabilities) that increase upload speed. The bootloader normally requires
8 KB FLASH, however, a 4 KB version can be used for the D11 chips.

Bossac is a command line utility for uploading firmware to SAM-BA bootloaders. It runs on Windows. Linux, and OS X.
It is used by Arduino to upload firmware to SAM and SAMD boards. The version Bossac described here adds to the
Arduino version (https://github.com/shumatech/BOSSA, Arduino branch), which in turn is a fork from the original
Bossa (http://www.shumatech.com/web/products/bossa). It adds support for more SAMD chips (both D21 and D11).

Note that only the Arduino or Mattairtech versions of bossac are currently supported for SAMD chips.
Neither the stock bossac (or Bossa) nor the Atmel SAM-BA upload tool will work.

Arduino Extended Capabilities:

   * X: Erase the flash memory starting from ADDR to the end of flash.
   * Y: Write the content of a buffer in SRAM into flash memory.
   * Z: Calculate the CRC for a given area of memory.

The bootloader can be started by:

   * Tapping reset twice in quick succession (BOOT_DOUBLE_TAP).
   * Holding down button A (BOOT_LOAD_PIN) while powering up.
   * Clicking 'Upload Sketch' in the Arduino IDE, which will automatically start the bootloader.
   * If the application (sketch) area is blank, the bootloader will run.

Otherwise, it jumps to application and starts execution from there. The LED will light during bootloader execution.
Note that the 4KB bootloader does not support the Arduino Extended Capabilities or BOOT_DOUBLE_TAP.

When the Arduino IDE initiates the bootloader, the following procedure is used:

1. The IDE opens and closes the USB serial port at a baud rate of 1200bps. This triggers a “soft erase” procedure.
2. The flash memory is erased by the MCU. If it is interrupted for any reason, the erase procedure will likely fail.
3. The board is reset. The bootloader (which always runs first) detects a blank FLASH, so bootloader operation resumes.
4. Opening and closing the port at a baud rate other than 1200bps will not erase or reset the SAMD.

### Bootloader Firmware Installation

#### Bootloader Installation Using the Arduino IDE

1. If you do not already have the MattairTech SAMD core installed, see SAMD Core Installation above.
2. Plug an Atmel ICE into USB, then connect it to the powered SAMD board. A green LED should light on the Atmel ICE.
3. Click Tools->Programmer->Atmel ICE.
4. Click Tools->Board->MattairTech MT-D21E.
5. Click Tools->Burn Bootloader. Ignore any messages about not supporting shutdown or reset.
6. Continue with driver installation above.

#### Bootloader Installation Using Another Tool (ie: Atmel Studio, openocd)

1. Download the bootloader from https://www.mattairtech.com/software/arduino/SAM-BA-bootloaders-zero-mattairtech.zip.
2. Unzip to any directory. Be sure that a bootloader is available for your particular chip.
3. Follow the procedures for your upload tool to upload the firmware.
   * Perform a chip erase first. Be sure no BOOTPROT bits are set.
   * Install the binary file to 0x00000000 of the FLASH.
   * You can optionally set the BOOTPROT bits to 8KB. The Arduino installation method does not set these.
   * You can optionally set the EEPROM bits or anything else. The Arduino installation method uses factory defaults.
4. Continue with driver installation above.

### Using Bossac Standalone

When using Bossac standalone, you will need to ensure that your application starts at 0x00002000 for 8 KB bootloaders,
and 0x00001000 for 4 KB bootloaders. This is because the bootloader resides at 0x00000000. This can be accomplished
by passing the following flag to the linker (typically LDFLAGS in your makefile; adjust for your bootloader size):

```
­Wl,­­section­start=.text=0x2000
```

You may also use a linker script. See the MattairTech SAMD package for examples.
Be sure to generate and use a binary file. Many makefiles are set up to generate an elf, hex, and bin already.

Download Bossac from:

* https://www.mattairtech.com/software/arduino/bossac-1.5-arduino-mattairtech-1-mingw32.zip (Windows 32 bit and 64 bit)
* https://www.mattairtech.com/software/arduino/bossac-1.5-arduino-mattairtech-1-x86_64-linux-gnu.tar.bz2 (Linux 64 bit)
* https://www.mattairtech.com/software/arduino/bossac-1.5-arduino-mattairtech-1-i686-linux-gnu.tar.bz2 (Linux 32 bit)
* Use the bossac command from the Arduino SAMD package for OS X support. Only the 256 KB chip versions are supported

As an example, bossac will be used to upload the test firmware (blink sketch):

1. Download firmware from https://www.mattairtech.com/software/SAM-BA-bootloader-test-firmware.zip and unzip.
2. If you have not already installed the bootloader driver, see Driver Installation above.
3. Be sure there is a binary that matches your chip. On the command line (change the binary to match yours):
4. On Linux --port might be /dev/ttyACM0. If the device is not found, remove the --port argument for auto-detection.

```
bossac.exe -d --port=COM5 -U true -i -e -w -v Blink_Demo_ATSAMD21E18A.bin -R
```
5. See http://manpages.ubuntu.com/manpages/vivid/man1/bossac.1.html for details.
6. Continue with the CDC-HID driver installation above (optional).



## New PinDescription Table

```
/*   The PinDescription table describes how each of the pins can be used by the Arduino core. Each pin can have multiple
 *   functions (ie: ADC input, digital output, PWM, communications, etc.), and the PinDescription table configures
 *   which functions can be used for each pin. This table is mainly accessed by the pinPeripheral function in
 *   wiring_private.c, which is used to attach a pin to a particular peripheral function. The communications drivers
 *   (ie: SPI, I2C, and UART), analogRead(), analogWrite(), analogReference(), attachInterrupt(), and pinMode() all
 *   call pinPeripheral() to verify that the pin can perform the function requested, and to configure the pin for
 *   that function. Most of the contents of pinMode() are now in pinPeripheral().
 * 
 *   Explanation of PinDescription table:
 * 
 *   Port			This is the port (ie: PORTA).
 *   Pin			This is the pin (bit) within the port. Valid values are 0 through 31.
 *   PinType			This indicates what peripheral function the pin can be attached to. In most cases, this
 *                              is PIO_MULTI, which means that the pin can be anything listed in the PinAttribute field.
 *                              It can also be set to a specific peripheral. In this case, any attempt to configure the
 *                              pin (using pinPeripheral or pinMode) as anything else will fail (and pinPeripheral will
 *                              return -1). This can be used to prevent accidental re-configuration of a pin that is
 *                              configured for only one function (ie: USB D- and D+ pins). If a pin is not used or does
 *                              not exist, PIO_NOT_A_PIN must be entered in this field. See WVariant.h for valid
 *                              entries. These entries are also used as a parameter to pinPeripheral() with the
 *                              exception of PIO_NOT_A_PIN and PIO_MULTI. The pinMode function now calls pinPeripheral()
 *                              with the desired mode. Note that this field is not used to select between the two
 *                              peripherals possible with each of the SERCOM and TIMER functions. PeripheralAttribute
 *                              is now used for this.
 *   PeripheralAttribute	This is an 8-bit bitfield used for various peripheral configuration. It is primarily
 *                              used to select between the two peripherals possible with each of the SERCOM and TIMER
 *                              functions. TIMER pins are individual, while SERCOM uses a group of two to four pins.
 *                              This group of pins can span both peripherals. For example, pin 19 (SPI1 SCK) uses
 *                              PER_ATTR_SERCOM_ALT while pin 22 (SPI1 MISO) uses PER_ATTR_SERCOM_STD. Both TIMER and
 *                              SERCOM can exist for each pin. This bitfield is also used to set the pin drive strength.
 *                              In the future, other attributes (like input buffer configuration) may be added.
 *                              See WVariant.h for valid entries.
 *   PinAttribute		This is a 32-bit bitfield used to list all of the valid peripheral functions that a pin
 *                              can attach to. This includes GPIO functions like PIN_ATTR_OUTPUT. Certain attributes
 *                              are shorthand for a combination of other attributes. PIN_ATTR_DIGITAL includes all of
 *                              the GPIO functions, while PIN_ATTR_TIMER includes both PIN_ATTR_TIMER_PWM and
 *                              PIN_ATTR_TIMER_CAPTURE (capture is not used yet). PIN_ATTR_ANALOG is an alias to
 *                              PIN_ATTR_ANALOG_ADC. There is only one DAC channel, so PIN_ATTR_DAC appears only once.
 *                              This bitfield is useful for limiting a pin to only input related functions or output
 *                              functions. This allows a pin to have a more flexible configuration, while restricting
 *                              the direction (ie: to avoid contention). See WVariant.h for valid entries.
 *   TCChannel			This is the TC(C) channel (if any) assigned to the pin. Some TC channels are available
 *                              on multiple pins (ie: TCC0/WO[0] is available on pin A4 or pin A8). In general, only
 *                              one pin should be configured (in the pinDescription table) per TC channel.
 *                              See WVariant.h for valid entries. The tone library uses TC5.
 *   ADCChannelNumber		This is the ADC channel (if any) assigned to the pin. See WVariant.h for valid entries.
 *   ExtInt			This is the interrupt (if any) assigned to the pin. Some interrupt numbers are available
 *                              on multiple pins (ie: EIC/EXTINT[2] is available on pin A2 or pin A18). In general, only
 *                              one pin should be configured (in the pinDescription table) per interrupt number. Thus,
 *                              if an interrupt was needed on pin 2, EXTERNAL_INT_2 can be moved from pin 18.
 *                              See WVariant.h for valid entries.
 */
```
