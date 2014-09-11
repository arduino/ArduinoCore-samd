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

#include <Arduino.h>
#include <Wire.h>

uint8_t accx,accy,zbut,cbut ;

void setup()
{
  SERIAL_PORT_MONITOR.begin( 115200 ) ;
  SERIAL_PORT_MONITOR.println("nunchuk init");

  Wire.begin(); // join i2c bus as master
  Wire.beginTransmission(0x52);// transmit to device 0x52
    Wire.write((uint8_t)0xF0);// sends sent a zero.
    Wire.write((uint8_t)0x55);// sends sent a zero.
    Wire.write((uint8_t)0xFB);// sends sent a zero.
    Wire.write((uint8_t)0x00);// sends sent a zero.
  Wire.endTransmission();// stop transmitting

  SERIAL_PORT_MONITOR.println( "WiiChukDemo ready" ) ;
  delay(100);
}

void loop()
{
  Wire.requestFrom(0x52, 6);

  uint8_t jX =   Wire.read();
  uint8_t jY =   Wire.read();
  uint8_t accX = Wire.read();
  uint8_t accY = Wire.read();
  uint8_t accZ = Wire.read();
  uint8_t misc = Wire.read();

  SERIAL_PORT_MONITOR.print("Joy : ");
  SERIAL_PORT_MONITOR.print(jX);
  SERIAL_PORT_MONITOR.print(", ");
  SERIAL_PORT_MONITOR.print(jY);

  SERIAL_PORT_MONITOR.print("\tAcc : ");
  SERIAL_PORT_MONITOR.print(accX);
  SERIAL_PORT_MONITOR.print(", ");
  SERIAL_PORT_MONITOR.print(accY);
  SERIAL_PORT_MONITOR.print(", ");
  SERIAL_PORT_MONITOR.print(accZ);

  SERIAL_PORT_MONITOR.print("\tBtn : ");
  SERIAL_PORT_MONITOR.print(" [");
  SERIAL_PORT_MONITOR.print(misc);
  SERIAL_PORT_MONITOR.print("] ");

  switch(misc & 0x3ul)
  {
    case 0x0ul:
      SERIAL_PORT_MONITOR.println("Z");
      break;

    case 0x1ul:
      SERIAL_PORT_MONITOR.println("C");
      break;

    case 0x2ul:
      SERIAL_PORT_MONITOR.println("C + Z");
      break;

    case 0x3ul:
      SERIAL_PORT_MONITOR.println("No key");
      break;

    default:
      break;
  }

  Wire.beginTransmission(0x52);// transmit to device 0x52
  Wire.write((uint8_t)0x00);// sends sent a zero.
  Wire.endTransmission();// stop transmitting

  delay(100);

}
