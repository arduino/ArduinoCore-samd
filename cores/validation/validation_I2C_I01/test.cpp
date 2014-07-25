/*
  Copyright (c) 2014 Arduino.  All right reserved.

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

#include "Arduino.h"
#include <Wire.h>

const uint8_t addressTemp = 0x4Ful;
uint16_t temp = 0;
uint8_t a, b;

void setup()
{
  Serial5.begin( 115200 );
  Serial5.println("Wire init");
  Wire.begin();
}

void loop()
{
  Wire.beginTransmission(addressTemp);
  Wire.write((uint8_t) 0x00);
  Wire.endTransmission();

  delay(100);

  Wire.requestFrom(addressTemp, 2);
  Serial5.print((char)13);
  Serial5.print("Temperature : ");

  a = Wire.read();
  b = Wire.read();

  temp = b << 7;
  temp |= a;
  temp >>= 7;

  Serial5.print(temp);
}
