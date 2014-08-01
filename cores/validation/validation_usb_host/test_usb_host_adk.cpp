/*
  Copyright (c) 2012 Arduino.  All right reserved.

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

#define ARDUINO_MAIN
//#include "variant.h"
#include "Arduino.h" 
#include <stdio.h>
#include <adk.h>


USBHost usb;
ADK adk(&usb,"Arduino SA",
            "Arduino_Terminal",
            "Arduino Due X",
            "1.0",
            "http://labs.arduino.cc/uploads/ADK/ArduinoTerminal/ThibaultTerminal_ICS_0001.apk",
            "1");

void setup(void)
{
  Serial5.begin( 115200 );
  while (!Serial5); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  Serial5.println("\r\nADK demo start");

  if (usb.Init() == -1)
	Serial5.println("OSC did not start.");

  delay(20);
}

#define RCVSIZE 128

void loop(void)
{
	uint8_t buf[RCVSIZE];
	uint32_t nbread = 0;
	char helloworld[] = "Hello World!\r\n";

	usb.Task();

	if( adk.isReady() == false ) {
		return;
	}
	/* Write hello string to ADK */
	adk.SndData(strlen(helloworld), (uint8_t *)helloworld);

	delay(1000);

	/* Read data from ADK and print to UART */
	adk.RcvData((uint8_t *)&nbread, buf);
	if (nbread > 0)
	{
		Serial5.print("RCV: ");
		for (uint32_t i = 0; i < nbread; ++i)
		{
			Serial5.print((char)buf[i]);
		}
		Serial5.print("\r\n");
	}	
}
