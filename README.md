# Arduino Core for SAMD21 CPU

This repository contains the source code and configuration files of the Arduino Core
for Atmel's SAMD21 processor (used on the Arduino Zero board and JACK board)

## Installation on Arduino IDE

The Arduino Zero core is available as a package in the Arduino IDE cores manager.
Just open the "Boards Manager" and install the package called:

"Arduino SAMD Boards (32-bit ARM Cortex-M0+)"

Next, clone this repository and move the following files/folders to C:\Users\mperla.CORP\AppData\Roaming\Arduino15\packages\arduino\hardware\samd\1.6.1 :

- bootloaders\jack
- cores\jack
- variants\jack
- boards.txt

## Contributions

Contributions are always welcome. The preferred way to receive code cotribution is by submitting a 
Pull Request on github.

## License and credits

The Arduino Zero core has been developed by Arduino LLC in collaboration with Atmel.

The JACK core has been developed by Michele Perla on top of the Arduino one, so I suppose
their GLPL applies to it as well.

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

