# Arduino Core for SAMD21 CPU

This repository contains the source code and configuration files of the Arduino Core
for Atmel's SAMD21 processor (used on the Arduino/Genuino Zero, MKR1000 and MKRZero boards).

## My mods contained in 'wire_timeout' branch

This version of the core has the following changes:

1. A modified version of jrowberg's excellent i2c wire timeout implementation. See pull request [#439](https://github.com/arduino/ArduinoCore-samd/pull/439). By the way I've made the default timeout 25ms. An example modification of Adafruits SSD1306 library for a 128x64 OLED is [here](https://github.com/acicuc/Adafruit_SSD1306/tree/wire_timeout). Albeit old, it still demonstrates one use for the i2c timeout implementation.
2. A fix for the infamous infinite recursion SERCOM issue, see pull request [#535](https://github.com/arduino/ArduinoCore-samd/pull/535).
3. The Serial.flush() blocks forever fix, see issue [#597](https://github.com/arduino/ArduinoCore-samd/issues/597).

These are currently the main issues I've struck while working with the SAMD21 in the 'real' world. In particular 1&2 above have allowed for hot-swap and plug&play for devices on the i2c bus, and allows for complete device failure without locking up your entire system or blowing the stack! I'll add others as they raise their ugly heads within my projects.

## Installation on Arduino IDE

This core is available as a package in the Arduino IDE cores manager.
Just open the "Boards Manager" and install the package called:

"Arduino SAMD Boards (32-bit ARM Cortex-M0+)"

## Support

There is a dedicated section of the Arduino Forum for general discussion and project assistance:

http://forum.arduino.cc/index.php?board=98.0

## Bugs or Issues

If you find a bug you can submit an issue here on github:

https://github.com/arduino/ArduinoCore-samd/issues

Before posting a new issue, please check if the same problem has been already reported by someone else
to avoid duplicates.

## Contributions

Contributions are always welcome. The preferred way to receive code cotribution is by submitting a 
Pull Request on github.

## Hourly builds

This repository is under a Continuous Integration system that every hour checks if there are updates and
builds a release for testing (the so called "Hourly builds").

The hourly builds are available through Boards Manager. If you want to install them:
  1. Open the **Preferences** of the Arduino IDE.
  2. Add this URL `http://downloads.arduino.cc/Hourly/samd/package_samd-hourly-build_index.json` in the **Additional Boards Manager URLs** field, and click OK.
  3. Open the **Boards Manager** (menu Tools->Board->Board Manager...)
  4. Install **Arduino SAMD core - Hourly build**
  5. Select one of the boards under **SAMD Hourly build XX** in Tools->Board menu
  6. Compile/Upload as usual

If you already installed an hourly build and you want to update it with the latest:
  1. Open the **Boards Manager** (menu Tools->Board->Board Manager...)
  2. Remove **Arduino SAMD core - Hourly build**
  3. Install again **Arduino SAMD core - Hourly build**, the Board Manager will download the latest build replacing the old one.

## License and credits

This core has been developed by Arduino LLC in collaboration with Atmel.

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
