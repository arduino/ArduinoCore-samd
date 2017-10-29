# Arduino Core for SAMD21 and SAMD51 CPU

## Known Issues:
* gcc optimizations must be off for sketches to work consistently. We are working to track this down.
* AREF must be tied to 3.3V for dac to work. This is a bug in the SAMD51 silicon.
* USB host mode doesn't work yet
* I2S doesn't work yet

This repository contains the source code and configuration files of the Arduino Core
for Atmel's SAMD21 and SAMD51 processor (used on the Arduino/Genuino Zero, MKR1000 and MKRZero boards).

In particular, this adds support for the Adafruit SAMD Boards such as the Feather M0

## Installation on Arduino IDE

This is a detailed process for now!

1. Install Arduino IDE 1.8.5 or greater
2. Remove the Adafruit SAMD board support package if you have installed it
3. make a folder in your ArduinoSketchFolder called "hardware", then make a folder "hardware/Adafruit"
4. in "hardware/Adafruit" git clone https://github.com/adafruit/ArduinoCore-samd, and rename the folder "samd"
5. Check you now have "ArduinoSketcheFolderName/hardware/Adafruit/samd/boards.txt"
6. git checkout samd51 from within the cloned repo, and check boards.txt contains "# Adafruit Metro M4 (SAMD51)"
7. Now we need to update CMSIS! Download https://github.com/adafruit/ArduinoModule-CMSIS-Atmel/archive/master.zip and unzip
8. Find your Arduino CMSIS, in Mac its "~/Library/Arduino15/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS" version # may vary, you'll need "Go to folder..." feature in the Finder to get to " ~/Library/Arduino15" as it is hidden. In Windows it is "C:\Users\yourusername\AppData\Local\Arduino15\packages\arduino\tools\CMSIS-Atmel\1.1.0\CMSIS" Inside is a folder named Device

For Linux: the Arduino stuff is located in ~/.arduino15/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS

9. Inside the downloaded CMSIS zip, go into the CMSIS-Atmel/CMSIS folder, you should see a folder named Device. *DRAG THE DEVICE FOLDER ONLY* from the zip to your arduino Library folder so that Device is merged with Device. It will replace a bunch of files.

For Linux, it may be better to just delete the original Device Folder then copy over the new one - the drag/drop worked ok on MACOS, but not so well on my Linux box.  

10. You will also need to replace bossac. Go here https://github.com/adafruit/BOSSA/releases and download either windows exe or mac app of latest bossac. unzip

For Linux users it is best to build a copy of bossac locally - 
clone the Adafruit Repostitory to somewhere on your local machine.
```
git clone https://github.com/adafruit/BOSSA.git
change to the arduino branch
git checkout arduino
Follow the instructions to build bossac.
```
11. Replace the binary at ".../Library/Arduino15/packages/arduino/tools/bossac/1.7.0/bossac" or "...\AppData\Local\Arduino15\packages\arduino\tools\bossac\1.7.0\bossac"
12. On Windows 7 you will also need to install the updated serial driver, download https://github.com/adafruit/Adafruit_Windows_Drivers/archive/master.zip to get all the drivers, and point the Device Manager to the unzipped folder. It isn't signed, so just approve the installation manually.
13. That's it!

## Bugs or Issues

If you find a bug you can submit an issue here on github:

https://github.com/adafruit/ArduinoCore-samd

or if it is an issue with the upstream:

https://github.com/arduino/ArduinoCore-samd/issues

Before posting a new issue, please check if the same problem has been already reported by someone else
to avoid duplicates.

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
