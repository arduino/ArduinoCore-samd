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

#include "variant.h"
#include <stdio.h>
#include <adk.h>

// Accessory descriptor. It's how Arduino identifies itself to Android.
char applicationName[] = "Arduino_Terminal"; // the app on your phone
char accessoryName[] = "Arduino Due X"; // your Arduino board
char companyName[] = "Arduino SA";

// Make up anything you want for these
char versionNumber[] = "1.0";
char serialNumber[] = "1";
char url[] = "http://labs.arduino.cc/uploads/ADK/ArduinoTerminal/ThibaultTerminal_ICS_0001.apk";

USBHost Usb;
ADK adk(&Usb, companyName, applicationName, accessoryName,versionNumber,url,serialNumber);

void setup()
{
	cpu_irq_enable();
	printf("\r\nADK demo start\r\n");
	delay(200);
}

#define RCVSIZE 128

void loop()
{
	uint8_t buf[RCVSIZE];
	uint32_t nbread = 0;
	char helloworld[] = "Hello World!\r\n";

	Usb.Task();

	if (adk.isReady())
	{
		/* Write hello string to ADK */
		adk.write(strlen(helloworld), (uint8_t *)helloworld);

		delay(1000);

		/* Read data from ADK and print to UART */
		adk.read(&nbread, RCVSIZE, buf);
		if (nbread > 0)
		{
			printf("RCV: ");
			for (uint32_t i = 0; i < nbread; ++i)
			{
				printf("%c", (char)buf[i]);
			}
			printf("\r\n");
		}
	}
}
