# MattairTech Arduino SAM M0+ Core

This is a fork from arduino/ArduinoCore-samd on GitHub. This will be used to maintain
Arduino support for SAM M0+ boards including the MattairTech MT-D21E and the MT-D11
(see https://www.mattairtech.com/). It primarily adds support for new devices as well
as a more flexible pin configuration / mapping system. It also adds some size
optimizations (~7.5KB for blink sketch with CDC+HID+UART, ~2.5KB without USB or UART).

This core is intended to be installed using Boards Manager (see below). To update from a
previous version, click on MattairTech SAM M0+ Boards in Boards Manager, then click Update.


## What's New Beta (1.6.8-beta)

1.6.8-beta-b0:
* Added L21 and C21 support. Improved D11D and D11C support.
  * Use Tools->Microcontroller menu to select mcu.
* Both the core and bootloader have added support for:
  * external high-speed crystal (400KHz - 32MHz) using PLL
  * external 32.768KHz crystal using PLL
  * internal oscillator with USB calibration using DFLL
  * internal oscillator using DFLL in open-loop mode (or 48MHz RC oscillator with C21)
  * PLL_FRACTIONAL_ENABLED and PLL_FAST_STARTUP options
  * The clock source is selectable in the Tools->Clock Source menu
* New Tools->Serial Config menu for selecting different combinations of serial peripherals
* New Tools->Bootloader Size menu allows selection of bootloader size
* New Tools->USB Config menu simplifies USB configuration compared to previous core
* Updated variant.cpp table format for future CCL and GCLK use. See VARIANT_COMPLIANCE_CHANGELOG.
* Updated bossac upload tool (fixed support for SAML and SAMC)
* New CMSIS-Atmel package (this is different than from Arduino)
* Merged in all changes from upstream through SAMD CORE 1.6.13 (2017.03.31)


## What's New Release (1.6.6)

* 1.6.6-mt3:
  * Fixes compilation with CDC_UART and CDC_ONLY settings

* 1.6.6-mt2:
  * Changes the default Communication setting to CDC_UART (from CDC_HID_UART)

* 1.6.6-mt1:
  * New documentation section 'Special Notes'. Please read!
  * Updated ASCII pinouts to be more readable and less ambiguous.
  * Updated the Signed driver for Windows (extras directory) (see CHANGELOG for details)
  * Merged in changes from upstream (see CHANGELOG for details)
  * Fix warnings about deprecated recipe.ar.pattern
  * Merged in changes from upstream SAMD CORE 1.6.2 2015.11.03 (see CHANGELOG for details)


## Summary

Feature                 |	MT-D21E										|	MT-D11
------------------------|---------------------------------------------------------------------------------------|------------------------------------------------------
Microcontroller		|	ATSAMD21ExxA, 32-Bit ARM Cortex M0+						|	ATSAMD11D14AM, 32-Bit ARM Cortex M0+
Clock Speed		|	48 MHz										|	48 MHz
Flash Memory		|	256 KB (D21E18A) / 128 KB (D21E17A) / 64 KB (D21E16A) / 32 KB (D21E15A)		|	16 KB (4KB used by USB SAM-BA bootloader)
SRAM			|	32 KB (D21E18A) / 16 KB (D21E17A) / 8 KB (D21E16A) / 4 KB (D21E15A)		|	4 KB
EEPROM			|	None (emulation may be available in the future)					|	None (emulation may be available in the future)
Digital Pins		|	22										|	17
Analog Input Pins	|	10, 12-bit ADC channels								|	10, 12-bit ADC channels
Analog Output Pins	|	1, 10-bit DAC									|	1, 10-bit DAC
PWM Output Pins		|	12										|	8
External Interrupts	|	15 (1 NMI)									|	9 (1 NMI)
USB			|	Device and Host (CDC and HID)							|	Device and Host (CDC and HID)
UART (Serial)		|	2										|	1
SPI			|	1										|	1
I2C (TWI)		|	1										|	1
Operating Voltage	|	3.3V (Do not connect voltages higher than 3.3V!)				|	3.3V (Do not connect voltages higher than 3.3V!)
DC Current per I/O Pin	|	7 mA										|	7 mA


## Special Notes

* **Boards Manager must be opened twice to see some updates**

* **Errors when compiling, uploading, or burning the bootloader**
  * Be sure to install the Arduino samd core before installing the MattairTech sam m0+ core. If you have problems upgrading
  the IDE to 1.6.6, you may need to uninstall both the Arduino and MattairTech cores, then re-install in the proper order.
  Use Arduino core 1.6.2 or above.

* **Tools->Communications menu**
  * Currently, the Tools->Communications menu must be used to select the communications configuration. This configuration
  must match the included libraries. For example, when including the HID and Keyboard libraries, you must select an
  option that includes HID (ie: CDC_HID_UART). This menu is currently needed to select the USB PID that matches the
  USB device configuration (needed for Windows). This may become automatic in a future release.

    * Be sure that the Tools->Communications menu matches the sketch and libraries you are compiling.
    * Different combinations of USB devices will result in different COM port assingments in Windows.

* **Incude platform specific libraries**
  * You may need to manually include platform specific libraries such as SPI.h, Wire.h, and HID.h.

* **Differences from Arduino in versioning**
  * The MattairTech ArduinoCore-samd version currently tracks the IDE version. In some cases, it may indicate the
  minimum IDE version. This is the case for both 1.6.5-mtX and 1.6.6-mtX (which corresponds to SAMD CORE 1.6.2).
  1.6.6-mt1 corresponds to Arduino SAMD CORE 1.6.2 plus some pull requests.


### Additional Differences Between MattairTech and Arduino Cores
* TODO


## Pin Configurations

Most pins have multiple configurations available (even analog pins). For example, pin A10 on the MT-D21E can be an analog
input, a PWM output, Digital I/O, or the TX pin of 'Serial1'. These always reference the pin number printed on the board
but without the 'A' (with the usable pins starting at 2). DO NOT connect voltages higher than 3.3V!


# MattairTech MT-D21E (rev B) (ATSAMx21Exxx)

```
========================== MattairTech MT-D21E rev B (ATSAMx21Exxx) =====================
Other  COM    PWM   Analog  INT  Arduino*             Arduino*  INT   PWM     COM   Other
=========================================================================================
                                      -------------------
XI32(+)                              | A0            RST |                        BOOT(+)
XO32(+)                              | A1            Gnd |
DAC                   *            2 | A2           Vbat |
REFA                  *            3 | A3            A31 | 31    *                IO/B(+)
REFB                  *      *     4 | A4            A30 | 30    *                 CLK(+)
DAC1(L)               *      *     5 | A5            NC  |
LED(+)       TCC10    *            6 | A6       A28 (D/C)| 28    *
VM           TCC11    *            7 | A7            A27 | 27    *               A/CS(+M)
             TCC00    *     NMI    8 | A8            A23 | 23    * TC41/TC01~ SS
             TCC01    *      *     9 | A9            A22 | 22    * TC40/TC00~ MISO(+M)
       TX1   TCC02    *           10 | A10           A19 | 19    *            SCK(+M)
       RX1   TCC03    *           11 | A11           A18 | 18    *            MOSI(+M)
       TX2 TC30/TC40~        *    14 | A14           A17 | 17    *   TCC21    SCL(+)
       RX2 TC31/TC41~             15 | A15           A16 | 16    *   TCC20    SDA(+)
                                     | NC            NC  |
     M=Memory device installed       | NC            NC  | ! Vcc is 3.3V by default.
                                     | Vbus          3.3V|   DO NOT exceed 3.6V on Vcc or
USB D- (D/L)(+), CAN TX (C)  TC50 24 | A24   _____   Vcc |   any IO pin with the D21 or
USB D+ (D/L)(+), CAN RX (C)  TC51 25 | A25  |     |  Vin |   L21 installed. 5V is allowed
                                     | Gnd  | USB |  Gnd |   ONLY with the C21 installed.
           Chip Variant:              -------------------
        D=D21, L=L21, C=C21

* Most pins can be used for more than one function. The same port pin number printed on
  the board is also used in Arduino (without the 'A') for all of the supported functions
  (ie: digitalRead(), analogRead(), analogWrite(), attachInterrupt(), etc.).

+ This alternate function is enabled by default (+M functions enabled only when a memory
  device is installed). Thus, the associated header pin cannot be used. Solder jumpers
  can be used to enable or disable the alternate onboard function.

~ When two timers are shown, the second is for L21/C21. TC5 is TC1 on the L21/C21.

Silkscreen Legend:
  Top: A circled pin means analog function and '*' means alternate function (see + above)
  Bottom: A circled pin means analog function
```


### SAMD11 (MT-D11)

```
=========================== MattairTech MT-D11 (ATsamd11D14AM) ==========================
Other  INT   PWM   Digital  Analog                    Digital   PWM      INT     Other
=========================================================================================
                                    -------------------
DAC                   2   2(ADC0)  | A2   | USB |  Gnd |
REF                   3   3(ADC1)  | A3   |     |  Vcc |
   4(INT4)  4(TCC00)  4   4(ADC2)  | A4    -----   A31 | 31  31(TC21)  31(INT3)  RX/SWDIO
   5(INT5)  5(TCC01)  5   5(ADC3)  | A5            A30 | 30  30(TC20)           TX/SWDCLK
            6(TCC02)  6   6(ADC4)  | A6            A27 | 27            27(INT7)
            7(TCC03)  7   7(ADC5)  | A7            A23 | 23                      SCL
MOSI  10(INT2)        10  10(ADC8) | A10           A22 | 22            22(INT6)  SDA
SCK                   11  11(ADC9) | A11           A17 | 17  17(TC11)
MISO  14(INTNMI)      14  14(ADC6) | A14           A16 | 16  16(TC10)  16(INT0)  LED
BTN/SS  15(INT1)      15  15(ADC7) | A15           RST |                         Reset
                                    -------------------
```


#### All pins operate at 3.3 volts. DO NOT connect voltages higher than 3.3V!

### Pin Capabilities

* **Digital: All pins can be used for general purpose I/O** 
  * Supports INPUT, OUTPUT, INPUT_PULLUP, and INPUT_PULLDOWN.
  * Each pin can source or sink a maximum of 7 mA (when PER_ATTR_DRIVE_STRONG is set for the pin).
  * Internal pull-up and pull-down resistors of 20-60 Kohms (40Kohm typ., disconnected by default).
  * Use the pinMode(), digitalWrite(), and digitalRead() functions.
* **Analog Inputs: 10 pins can be configured as ADC analog inputs.**
  * These are available using the analogRead() function.
  * All pins can be used for GPIO and some pins can be used for other digital functions (ie. pwm or serial).
  * Each pin provides 10 bits of resolution (1024 values) by default.
  * 12-bit resolution supported by using the analogReadResolution() function.
  * Each pin measures from ground to 3.3 volts.
  * The upper end of the measurement range can be changed using the AREF pin and the analogReference() function.
* **DAC: One analog output is available on pin 2.**
  * Provides a 10-bit voltage output with the analogWrite() function.
* **PWM: 12 pins (MT-D21E) or 8 pins (MT-D11) can be configured as PWM outputs.**
  * Available using the analogWrite() function.
  * Each pin provides 8 bits of resolution (256 values) by default.
  * 12-bit resolution supported by using the analogWriteResolution() function.
* **External Interrupts: 15 pins (MT-D21E) or 9 pins (MT-D11) can be configured with external interrupts.**
  * Available using the attachInterrupt() function.
* **Serial: 2 pairs of pins (MT-D21E) or 1 pair (MT-D11) can be configured for TTL serial I/O.**
  * MT-D21E: Serial1: pin 11 (RX) and pin 10 (TX). Serial2: pin 15 (RX) and pin 14 (TX).
  * MT-D11: Serial1: pin 31 (RX) and pin 30 (TX).
* **SPI: 3 or 4 pins can be configured for SPI I/O (SPI).**
  * MT-D21E: Pin 18 (MOSI), pin 19 (SCK), pin 22 (MISO), and optionally pin 23 (SS, not currently used).
  * MT-D11: Pin 10 (MOSI), pin 11 (SCK), pin 14 (MISO), and optionally pin 15 (SS, not currently used).
  * SPI communication using the SPI library.
  * Note that the SPI library will set SS as an output.
  * On the MT-D11, the button must be configured as reset (default) when using SPI.
* **TWI (I2C): 2 pins can be configured for TWI I/O (Wire).**
  * MT-D21E: Pin 16 (SDA) and pin 17 (SCL).
  * MT-D11: Pin 22 (SDA) and pin 23 (SCL).
  * TWI communication using the Wire library.
* **LED: One pin can be configured to light the onboard LED (LED_BUILTIN).**
  * Pin 28 (MT-D21E) or pin 16 (MT-D11). Bring the pin HIGH to turn the LED on. The pullup is disabled on this pin.
* **Button: One pin can be configured to read the onboard Button A (BUTTON_BUILTIN).**
  * Pin 27 (MT-D21E) or pin 15 (MT-D11). Pressing the button will bring the pin LOW. The pullup must be enabled first.
  * If the debouncing capacitor is connected, delay reading the pin at least 6ms after turning on the pullup.
* **AREF: One pin can be configured as an AREF analog input.**
  * The upper end of the analog measurement range can be changed using the analogReference() function.
* **Reset: Bring this line LOW to reset the microcontroller.**

### MT-D21E and MT-D11 Board Configuration Notes

* The 32.768KHz crystal is used by the Arduino core, so it MUST be connected via the solder jumpers.
* The sketch in some cases can run without the crystal attached, but the clock will be very inaccurate.
* The 16MHz crystal is not used. It should be disconnected via the solder jumpers.
* The I2C (TWI) pullup resistors should be enabled via the solder jumpers.
* The LED should be enabled via the solder jumper.
* Button A and the debouncing capacitor should be connected via the solder jumpers.
* Button B (MT-D21E only) is connected to the Reset pin by default, but can be connected to pin 31 via the solder jumper.
* A reference voltage can be connected to AREF. In this case, the capacitors should be enabled via the solder jumpers.
* On the MT-D11, BTN is shared with SPI SS, so the button must be configured as reset (default) when using SPI.


## Serial Monitor

To print to the Serial Monitor over USB, use 'Serial'. Serial points to SerialUSB (Serial1 and Serial2 are UARTs).
Unlike most Arduino boards (ie. Uno), SAM M0+ boards do not automatically reset when the serial monitor is opened.
To see what your sketch outputs to the serial monitor from the beginning, the sketch must wait for the SerialUSB
port to open first. Add the following to setup():

```
while (!Serial) ;
```

Remember that if the sketch needs to run without SerialUSB connected, another approach must be used.
You can also reset the board manually with the Reset button if you wish to restart your sketch. However, pressing
the Reset button will reset the SAM M0+ chip, which in turn will reset USB communication. This interruption means
that if the serial monitor is open, it will be necessary to close and re-open it to restart communication.


## Code Size and RAM Usage (1.6.5-mt2)

Sketch and Configuration    | MT-D21E (Flash + RAM) | MT-D11 (Flash + RAM)
----------------------------|-----------------------|-----------------------
Blink (CDC + HID + UART)    |     7564 + 1524       |     7452 + 1424
Blink (CDC + UART)          |     6588 + 1496       |     6484 + 1396
Blink (CDC Only)            |     5248 + 1304       |     5192 + 1300
Blink (UART Only)           |     3828 + 336        |     3716 + 236
Blink (No USB or UART)      |     2472 + 144        |     2416 + 140
Datalogger (No USB or UART) |     10340 + 948       |     10260 + 944

* 180 bytes of flash can be saved on the MT-D11 by using PIN_MAP_COMPACT (see 'New PinDescription Table' below).
* Datalogger compiled without USB or UART support, but with SPI and SD (with FAT filesystem) support. Serial output was disabled.
* Note that USB CDC is required for auto-reset into the bootloader to work (otherwise, manually press reset twice in quick succession).
* USB uses primarily 3 buffers totaling 1024 bytes. The UART uses a 96 byte buffer. The banzai() function (used for auto-reset) resides in RAM and uses 72 bytes.
* Any combination of CDC, HID, or UART can be used (or no combination), by using the Tools->Communication menu.


### Detailed Memory Usage Output After Compilation

The flash used message at the end of compilation is not correct. The number shown
represents the .text segment only. However, Flash usage = .text + .data segments
(RAM usage = .data + .bss segments). In this release, two programs are run at the
end of compilation to provide more detailed memory usage. To enable this output, go
to File->Preferences and beside "Show verbose output during:", check "compilation".

Just above the normal flash usage message, is the output from the size utility.
However, this output is also incorrect, as it shows .text+.data in the .text field,
but 0 in the .data field. However, the .text field does show the total flash used.
The .data field can be determined by subtracting the value from the normal flash
usage message (.text) from the value in the .text field (.text+.data). The .bss
field is correct.

Above the size utility output is the output from the nm utility. The values on the
left are in bytes. The letters stand for: T(t)=.text, D(d)=.data, B(b)=.bss, and
everything else (ie: W) resides in flash (in most cases).


## Installation

### Driver Installation

#### Windows

Prior to core version 1.6.6-mt1, sketches compiled with both CDC and HID USB code by default, thus requiring a CDC
driver for the bootloader and a CDC-HID driver for sketches. Now that PluggableUSB is supported, sketches compile
with only CDC code by default. Thus, only one driver is needed. Since HID and MIDI are currently supported (and
MSD potentially in the future), driver installation will be required for each different combination of USB devices.
There are currently four USB composite device combinations that include CDC as well as a CDC only device. Each
supported combination has a unique USB VID:PID pair, and these are listed in the .inf file. Once the first device
is installed (the CDC only device), future installations *might* be automatic, otherwise, you may direct the
installer to the same .inf file. The drivers are signed and support both 32 and 64 bit versions of Windows XP(SP3),
Vista, 7, 8, and 10.


1. If you do not already have the SAM-BA bootloader installed, see below.
2. Download https://www.mattairtech.com/software/MattairTech_CDC_Driver_Signed.zip and unzip into any folder.
3. Plug in the board while holding down button A to enter the bootloader. The LED should light.
4. Windows will detect the board. Point the installer to the folder from above to install the bootloader driver.
5. If you don't intend on using Arduino, you can skip the rest of this list. See Using Bossac Standalone below.
6. If you do not already have the test firmware installed (comes preinstalled), see Using Bossac Standalone below.
7. Press the reset button to run the test firmware (blink sketch).
8. Windows will detect the board. Point the installer to the above folder to install the sketch driver (if needed).
9. Continue with SAM M0+ Core Installation below.

#### Linux

0. No driver installation is needed.
1. On some distros, you may need to add your user to the same group as the port (ie: dialout) or set udev rules:
   * See the file https://github.com/mattairtech/ArduinoCore-samd/tree/master/drivers/99-mattairtech-USB-CDC.rules.
2. You MAY have to install and use Arduino as the root user in order to get reliable access to the serial port.
   * This is true even when group permissions are set correctly, and it may fail after previously working.
   * You can also create/modify a udev rule to set permissions on the port so *everyone* can read / write.
3. If you are running modemmanager (ie: Ubuntu), disable it, or use the udev rules file above.
4. Continue with SAM M0+ Core Installation below.

#### OS X

OS X support currently in beta (see below), the following instructions are still for 1.6.6-mtX.
1. Only the 256 KB chip variants work with the OS X version of the upload tool, bossac.
2. First, you will need to open boards.txt and change mattairtech_mt_d21e_bl8k.upload.tool to equal arduino:bossac.
3. Open platform.txt and change tools.bossac.path to equal{runtime.tools.bossac-1.6.1-arduino.path}.
4. No driver installation is needed.
5. Plug in the board. You may get a dialog box asking if you wish to open the “Network Preferences”:
   * Click the "Network Preferences..." button, then click "Apply".
   * The board will show up as “Not Configured”, but it will work fine.
5. Continue with SAM M0+ Core Installation below.


### SAM M0+ Core Installation

* To update from a previous version, click on MattairTech SAM M0+ Boards in Boards Manager, then click Update.
* Boards Manager may require opening twice (with possibly a delay in between) to see some updates.

1. The MattairTech SAM M0+ Core requires Arduino 1.6.7 (1.6.5-mtX required IDE 1.6.5).
2. In the Arduino IDE, click File->Preferences.
3. Click the button next to Additional Boards Manager URLs.
4. Add https://www.mattairtech.com/software/arduino/package_MattairTech_index.json.
5. Save preferences, then open the Boards Manager.
6. Install the Arduino SAM M0+ Boards package. Use version 1.6.2 or higher.
7. Install the MattairTech SAM M0+ Boards package (1.6.7).
8. Close Boards Manager, then click Tools->Board->MattairTech MT-D21E (or MT-D11).
9. Select the processor with the now visible Tools->Processor menu.
10. If you do not already have the bootloader or blink sketch installed, see SAM-BA USB CDC Bootloader below.
11. Plug in the board. The blink sketch should be running.
12. Click Tools->Port and choose the COM port.
13. You can now upload your own sketch.


### Uploading the First Sketch

1. In the Arduino IDE 1.6.7 (or above), open File->Examples->01.Basics->Blink.
2. Change the three instances of '13' to 'LED_BUILTIN'.
3. Be sure the correct options are selected in the Tools menu (see AVR Core Installation above).
4. With the board plugged in, select the correct port from Tools->Port.
5. Click the Upload button. After compiling, the sketch should be transferred to the board.
6. Once the bootloader exits, the blink sketch should be running.


## SAM-BA USB CDC Bootloader (Arduino compatible)

The SAM-BA bootloader has both a CDC USB interface, and a UART interface (TX: pin 10, RX: pin 11, 8N1).
It is compatible with the Arduino IDE (Zero compatible), or it can be used with the Bossac tool standalone. Under
Arduino, auto-reset is supported (automatically runs the bootloader while the sketch is running) as well as
automatic return from reset. The SAM-BA bootloader described here adds to the Arduino version, which in
turn is based on the bootloader from Atmel. The Arduino version added several features, including three
new commands (Arduino Extended Capabilities) that increase upload speed. The bootloader normally requires
8 KB FLASH, however, a 4 KB version can be used for the D11 chips.

Bossac is a command line utility for uploading firmware to SAM-BA bootloaders. It runs on Windows. Linux, and OS X.
It is used by Arduino to upload firmware to SAM and SAM M0+ boards. The version Bossac described here adds to the
Arduino version (https://github.com/shumatech/BOSSA, Arduino branch), which in turn is a fork from the original
Bossa (http://www.shumatech.com/web/products/bossa). It adds support for more SAM M0+ chips (D21, L21, C21, and D11).

Note that only the Arduino or Mattairtech versions of bossac are currently supported for SAM M0+ chips.
Neither the stock bossac (or Bossa) nor the Atmel SAM-BA upload tool will work.

Arduino Extended Capabilities:

   * X: Erase the flash memory starting from ADDR to the end of flash.
   * Y: Write the content of a buffer in SRAM into flash memory.
   * Z: Calculate the CRC for a given area of memory.

The bootloader can be started by:

   * Tapping reset twice in quick succession (BOOT_DOUBLE_TAP).
   * Holding down button A (BOOT_LOAD_PIN) while powering up.
   * Clicking 'Upload Sketch' in the Arduino IDE, which will automatically start the bootloader (when CDC is enabled).
   * If the application (sketch) area is blank, the bootloader will run.

Otherwise, it jumps to application and starts execution from there. The LED will light during bootloader execution.
Note that the 4KB bootloader does not support the Arduino Extended Capabilities or BOOT_DOUBLE_TAP.
However, BOOT_DOUBLE_TAP does fit into the SAMD11 4KB bootloader.

When the Arduino IDE initiates the bootloader, the following procedure is used:

1. The IDE opens and closes the USB serial port at a baud rate of 1200bps. This triggers a “soft erase” procedure.
2. The first row of application section flash memory is erased by the MCU. If it is interrupted for any reason, the erase procedure will likely fail.
3. The board is reset. The bootloader (which always runs first) detects the blank flah row, so bootloader operation resumes.
4. Opening and closing the port at a baud rate other than 1200bps will not erase or reset the SAM M0+.


### Bootloader Firmware Installation

#### Bootloader Installation Using the Arduino IDE

1. If you do not already have the MattairTech SAM M0+ core installed, see SAM M0+ Core Installation above.
2. Plug in the SAM M0+ board. The bootloader must be running to (press reset twice within 500ms).
3. Plug an Atmel ICE into USB, then connect it to the powered SAM M0+ board. A green LED should light on the Atmel ICE.
4. Click Tools->Programmer->Atmel ICE.
5. Click Tools->Board->MattairTech MT-D21E (or MT-D11).
6. Click Tools->Processor and select your chip.
7. Click Tools->Burn Bootloader. Ignore any messages about not supporting shutdown or reset.
8. Continue with driver installation above.

A running sketch may interfere with the bootloader installation process. Be sure you are running the existing bootloader or using a blank chip.

#### Bootloader Installation Using Another Tool (ie: Atmel Studio, openocd)

1. Download the bootloader from https://www.mattairtech.com/software/arduino/SAM-BA-bootloaders-zero-mattairtech.zip.
2. Unzip to any directory. Be sure that a bootloader is available for your particular chip.
3. Follow the procedures for your upload tool to upload the firmware.
   * Perform a chip erase first. Be sure no BOOTPROT bits are set.
   * Install the binary file to 0x00000000 of the FLASH.
   * You can optionally set the BOOTPROT bits to 8KB (or 4KB for the MT-D11). The Arduino installation method does not set these.
   * You can optionally set the EEPROM bits or anything else. The Arduino installation method uses factory defaults.
4. Continue with driver installation above.


### Using Bossac Standalone

When using Bossac standalone, you will need to ensure that your application starts at 0x00002000 for 8 KB bootloaders,
and 0x00001000 for 4 KB bootloaders. This is because the bootloader resides at 0x00000000. This can be accomplished
by passing the following flag to the linker (typically LDFLAGS in your makefile; adjust for your bootloader size):

```
Wl,sectionstart=.text=0x2000
```

You may also use a linker script. See the MattairTech SAM M0+ package for examples.
Be sure to generate and use a binary file. Many makefiles are set up to generate an elf, hex, and bin already.

Download Bossac from:

* https://www.mattairtech.com/software/arduino/bossac-1.6.1-arduino-mattairtech-1-mingw32.tar.bz2 (Windows 32 bit and 64 bit)
* https://www.mattairtech.com/software/arduino/bossac-1.6.1-arduino-mattairtech-1-x86_64-linux-gnu.tar.bz2 (Linux 64 bit)
* https://www.mattairtech.com/software/arduino/bossac-1.6.1-arduino-mattairtech-1-i686-linux-gnu.tar.bz2 (Linux 32 bit)
* https://www.mattairtech.com/software/arduino/bossac-1.6.1-arduino-mattairtech-1-x86_64-apple-darwin.tar.bz2 (OS X 64 bit)

Linux 64 bit users can also download Bossa (GUI) and bossash (shell) from:

* https://www.mattairtech.com/software/arduino/Bossa-1.6.1-arduino-mattairtech-1-x86_64-linux-gnu.tar.bz2 (Linux 64 bit)

As an example, bossac will be used to upload the test firmware (blink sketch):

1. Download firmware from https://www.mattairtech.com/software/SAM-BA-bootloader-test-firmware.zip and unzip.
2. If you have not already installed the bootloader driver, see Driver Installation above.
3. Be sure there is a binary that matches your chip. On the command line (change the binary to match yours):

```
bossac.exe -d --port=COM5 -U true -i -e -w -v Blink_Demo_ATSAMD21E18A.bin -R
```
4. On Linux --port might be /dev/ttyACM0. If the device is not found, remove the --port argument for auto-detection.
5. See http://manpages.ubuntu.com/manpages/vivid/man1/bossac.1.html for details.
6. The board should reset automatically and the sketch should be running.



## New PinDescription Table

### Note that a new column (GCLKCCL) was added for 1.6.8-beta-b0.
MATTAIRTECH_ARDUINO_SAMD_VARIANT_COMPLIANCE in variant.h is used to track versions.
If using board variant files with the old format, the new core will still read the
table the old way, losing any new features introduced by the new column. Additionally,
new definitions have been added for L21 and C21 support.

### Each pin can have multiple functions.
The PinDescription table describes how each of the pins can be used by the Arduino
core. Each pin can have multiple functions (ie: ADC input, digital output, PWM,
communications, etc.), and the PinDescription table configures which functions can
be used for each pin. This table is mainly accessed by the pinPeripheral function in
wiring_private.c, which is used to attach a pin to a particular peripheral function.
The communications drivers (ie: SPI, I2C, and UART), analogRead(), analogWrite(),
analogReference(), attachInterrupt(), and pinMode() all call pinPeripheral() to
verify that the pin can perform the function requested, and to configure the pin for
that function. Most of the contents of pinMode() are now in pinPeripheral().

### Pin Mapping
There are different ways that pins can be mapped. Typically, there is no relation
between the arduino pin number used, and the actual port pin designator. Thus, the 
pcb must be printed with the arduino numbering, otherwise, if the port pin is printed,
a cross reference table is needed to find the arduino pin number. However, this results
in the least amount of space used by the table. Another method, used by default by the
MT-D21E and MT-D11, maps Arduino pin numbers to the actual port pin number (ie: Arduino
pin 28 = Port A28). This works well when there is only one port (or if the PORTB pins
are used for onboard functions and not broken out). PIO_NOT_A_PIN entries must be added
for pins that are used for other purposes or for pins that do not exist (especially the
D11), so some FLASH space may be wasted. For an example of both types, see variant.cpp
from the MT-D11 variant. The MT-D21J combines both methods, using the actual port pin 
designators from both PORTA and PORTB for arduino numbers 0-31 (ie: B1=1, A2=2), then
using arduino numbering only above 31. For 0-31 only one pin from PORTA or PORTB can be
used, leaving the other pin for some number above 31.

### See WVariant.h in cores/arduino for the definitions used in the table.

#### Port:
This is the port (ie: PORTA).

#### Pin:
This is the pin (bit) within the port. Valid values are 0-31.

#### PinType:
This indicates what peripheral function the pin can be attached to. In most cases,
this is PIO_MULTI, which means that the pin can be anything listed in the PinAttribute
field. It can also be set to a specific peripheral. In this case, any attempt to
configure the pin (using pinPeripheral or pinMode) as anything else will fail (and
pinPeripheral will return -1). This can be used to prevent accidental re-configuration
of a pin that is configured for only one function (ie: USB D- and D+ pins). If a pin
is not used or does not exist, PIO_NOT_A_PIN must be entered in this field. See
WVariant.h for valid entries. These entries are also used as a parameter to
pinPeripheral() with the exception of PIO_NOT_A_PIN and PIO_MULTI. The pinMode function
now calls pinPeripheral() with the desired mode. Note that this field is not used to
select between the two peripherals possible with each of the SERCOM and TIMER functions.
PeripheralAttribute is now used for this.

#### PeripheralAttribute:
This is an 8-bit bitfield used for various peripheral configuration. It is primarily
used to select between the two peripherals possible with each of the SERCOM and TIMER
functions. TIMER pins are individual, while SERCOM uses a group of two to four pins.
This group of pins can span both peripherals. For example, pin 19 (SPI1 SCK) on the
MT-D21E uses PER_ATTR_SERCOM_ALT while pin 22 (SPI1 MISO) uses PER_ATTR_SERCOM_STD.
Both TIMER and SERCOM can exist for each pin. This bitfield is also used to set the
pin drive strength. In the future, other attributes (like input buffer configuration)
may be added. Starting with 1.6.8, the ADC instance on the C21 (there are two) is also
selected here. See WVariant.h for valid entries.

#### PinAttribute
This is a 32-bit bitfield used to list all of the valid peripheral functions that a
pin can attach to. This includes GPIO functions like PIN_ATTR_OUTPUT. Certain
attributes are shorthand for a combination of other attributes. PIN_ATTR_DIGITAL
includes all of the GPIO functions, while PIN_ATTR_TIMER includes both
PIN_ATTR_TIMER_PWM and PIN_ATTR_TIMER_CAPTURE (capture is not used yet).
PIN_ATTR_ANALOG is an alias to PIN_ATTR_ANALOG_ADC. This bitfield is useful for
limiting a pin to only input related functions or output functions. This allows a pin
to have a more flexible configuration, while restricting the direction (ie: to avoid
contention). See WVariant.h for valid entries.

#### TCChannel
This is the TC/TCC channel (if any) assigned to the pin. Some TC channels are available
on multiple pins. In general, only one pin should be configured in the pinDescription
table per TC channel. Starting with 1.6.8, the timer type is now encoded in this column
to support the L21 and C21, which use TC numbers starting at 0 (rather than 3 as on the
D21). See WVariant.h for valid entries.

#### ADCChannelNumber
This is the ADC channel (if any) assigned to the pin. The C21 has two ADC instances,
which is selected in the PeripheralAttribute column. See WVariant.h for valid entries.

#### ExtInt
This is the interrupt (if any) assigned to the pin. Some interrupt numbers are
available on multiple pins. In general, only one pin should be configured in the
pinDescription table per interrupt number. Thus, for example, if an interrupt was
needed on pin 2, EXTERNAL_INT_2 can be moved from pin 18. See WVariant.h for valid
entries.

#### GCLKCCL
This column was added in 1.6.8-beta-b0. It is not yet used. It will eventually support
the Analog Comparators (AC), the Configurable Custom Logic (CCL) units of the L21 and
C21, and the GCLK outputs (inputs).


## Possible Future Additions

* SAML21 and C21 support is currently under development
* Timer library is currently under development (like TimerOne, plus input capture, plus ??)
* OS X support currently in beta testing

* USB Host mode CDC ACM (partially complete; BSD-like license?)
* Features for lower power consumption (library?)
* Enhanced SD card library
* Optional use of single on-board LED as USB activity LED
* MSC (Mass Storage) USB Device Class
* Polyphonic tone
* Wired-AND, Wired-OR for port pins
* High-speed port pin access (IOBUS)
* Libraries for some hardware I plan on using:
  * TFT LCD (CFAF128128B-0145T)
  * Motor controller (LV8711T)
  * IR decoder
  * I2S DAC/AMP and I2S MEMS microphone
  * Battery management IC
  * XBee/Xbee Pro devices
  * RS485
  * Several I2C (Wire) sensor devices: 
    * Accelerometer/magnetometer (LSM303CTR)
    * Barometer/altimeter (LPS22HBTR)
    * Humidity/temperature
    * Light/color sensor


## ChangeLog

The Changelog has moved to a separate file named CHANGELOG. The most recent changes are still in the 'What's New' section above.


## Bugs or Issues

If you find a bug you can submit an issue here on github:

https://github.com/mattairtech/ArduinoCore-samd/issues

Before posting a new issue, please check if the same problem has been already reported by someone else to avoid duplicates.


## Contributions

Contributions are always welcome. The preferred way to receive code cotribution is by submitting a Pull Request on github.


## Beta builds

Periodically, a beta is released for testing.

The beta builds are available through Boards Manager. If you want to install them:
  1. Open the **Preferences** of the Arduino IDE.
  2. Add this URL `https://www.mattairtech.com/software/arduino/beta/package_MattairTech_index.json` in the **Additional Boards Manager URLs** field, and click OK.
  3. Open the **Boards Manager** (menu Tools->Board->Board Manager...)
  4. Install **MattairTech SAM M0+ Boards - Beta build**
  5. Select one of the boards under **MattairTech SAM M0+ Beta Build XX** in Tools->Board menu
  6. Compile/Upload as usual

The Arduino IDE will notify the user if an update to the beta is available, which can then be installed automatically.
Alternatively, if a particular beta is needed, replace the url in step 2 with:
  `https://www.mattairtech.com/software/arduino/beta/package_MattairTech_sam_m0p-${VERSION}-beta-b${BUILD_NUMBER}_index.json`
where ${VERSION} and ${BUILD_NUMBER} match the beta name as shown in the CHANGELOG (ie: package_MattairTech_sam_m0p-1.6.7-beta-b0_index.json).
In this case, the IDE will not notify the user of updates.


## License and credits

This core has been developed by Arduino LLC in collaboration with Atmel.
This fork developed by Justin Mattair of MattairTech LLC.

```
  Copyright (c) 2015 Arduino LLC.  All right reserved.

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
```
