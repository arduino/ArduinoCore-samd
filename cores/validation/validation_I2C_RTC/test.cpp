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

const uint8_t address = 0x68;

uint8_t decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
uint8_t bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}

void setup()
{
  SERIAL_PORT_MONITOR.begin(9600);
  SERIAL_PORT_MONITOR.println("Wire init");
  Wire.begin();

  // Setting the date
  /*
  Wire.beginTransmission(address);
    Wire.write((uint8_t)0x00);  // Init register

    Wire.write(decToBcd(0));  // Second
    Wire.write(decToBcd(39));  // Minute
    Wire.write(decToBcd(11));  // Hours

    Wire.write(decToBcd(4));  // Day of week

    Wire.write(decToBcd(21));  // Day of Month
    Wire.write(decToBcd(5));  // Month
    Wire.write(decToBcd(14));  // Year
  Wire.endTransmission();
  */
}

void loop()
{

  Wire.beginTransmission(address);
    Wire.write((uint8_t)0x3F);
  Wire.endTransmission();

  delay(10);

  Wire.requestFrom(address, 7);
/*  while(Wire.available())
  {
    SERIAL_PORT_MONITOR.print(bcdToDec(Wire.read()));
    SERIAL_PORT_MONITOR.print(" ");
  }
  SERIAL_PORT_MONITOR.println();*/


   int second     = bcdToDec(Wire.read() & 0x7f);
   int minute     = bcdToDec(Wire.read());
   int hour       = bcdToDec(Wire.read() & 0x3f);  // Need to change this if 12 hour am/pm
   int dayOfWeek  = bcdToDec(Wire.read());
   int dayOfMonth = bcdToDec(Wire.read());
   int month      = bcdToDec(Wire.read());
   int year       = bcdToDec(Wire.read());

   SERIAL_PORT_MONITOR.print(hour, DEC);
   SERIAL_PORT_MONITOR.print(":");
   SERIAL_PORT_MONITOR.print(minute, DEC);
   SERIAL_PORT_MONITOR.print(":");
   SERIAL_PORT_MONITOR.print(second, DEC);
   SERIAL_PORT_MONITOR.print("  ");
   SERIAL_PORT_MONITOR.print(month, DEC);
   SERIAL_PORT_MONITOR.print("/");
   SERIAL_PORT_MONITOR.print(dayOfMonth, DEC);
   SERIAL_PORT_MONITOR.print("/");
   SERIAL_PORT_MONITOR.print(year,DEC);
   SERIAL_PORT_MONITOR.print("  ");
   SERIAL_PORT_MONITOR.println();

  delay(990);
}
