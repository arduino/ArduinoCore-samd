# Arduino Core for SAM D21/R21 (E variant) CPU

This repository containts the source code and configuration files of the Arduino Core
for Atmel's SAM D21 and R21 (E variant) processor (used on the FemtoUSB and FemtoBeacon board).

## Installation on Arduino IDE

Please use Arduino IDE **1.8.1 or higher**. Install the Arduino SAMD Boards (32-bits ARM Cortex-M0+) board package first.

Add the following package URL to the Additional Boards Manager URLs field in your Arduino IDE via **File > Preferences** (Settings tab).

The stable release URL is:

```
https://downloads.femtoduino.com/ArduinoCore-atsamd21e18a/package_atsamd21e18a-release-build_index.json
```
The hourly build URL is:

```
https://downloads.femtoduino.com/ArduinoCore-atsamd21e18a/package_atsamd21e18a-hourly-build_index.json
```

This core is now available as a package in the Arduino IDE cores manager.
Just open the **Boards Manager** and install the package called:

"Atmel SAM D21/R21 core (ATSAMD21E18A/ATSAMR21E18A)" by Femtoduino



For SAM R21 support (ATMEL LwMesh), you will need our fork of `library-atmel-lwmesh` library, "at86rf233" branch. Simply add it to your Arduino library folder as "at86rf233"

See https://github.com/femtoduino/library-atmel-lwm/tree/at86rf233

You will also need the FreeIMU libraries (except "MotionDriver") from our fork of FreeIMU-Updates ("FemtoBeacon" branch). Remember to configure FreeIMU/FreeIMU.h

See https://github.com/femtoduino/FreeIMU-Updates


## Support

This package is derrived from the Arduino Zero bootloader and core. It has been modified to work with the E variant of the SAM D21 chips.

There is a dedicated section of the Arduino Forum for general discussion and project assistance:

http://forum.arduino.cc/index.php?board=98.0

## Bugs or Issues

If you find a bug with the SAM D21/R21 core, you can submit an issue here on github:

https://github.com/femtoduino/ArduinoCore-atsamd21e18a/issues

Before posting a new issue, please check if the same problem has been already reported by someone else
to avoid duplicates.

## Contributions

Contributions are always welcome. The preferred way to receive code cotribution is by submitting a 
Pull Request on github.


## License and credits

The original Arduino Zero core has been developed by Arduino LLC in collaboration with Atmel.

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
