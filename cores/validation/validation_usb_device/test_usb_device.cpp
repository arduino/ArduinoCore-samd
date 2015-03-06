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

#define ARDUINO_MAIN
#include "Arduino.h"

#ifdef HID_ENABLED
	const int buttonPin = 4;          // input pin for pushbutton
	int previousButtonState = HIGH;   // for checking the state of a pushButton
	int counter = 0;                  // button push counter
#endif

void setup(void)
{
  SERIAL_PORT_MONITOR.begin( 115200 );
  while (!SERIAL_PORT_MONITOR); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  SERIAL_PORT_MONITOR.println("Start USB device test");
 
#ifdef HID_ENABLED
	Mouse.begin();

	// make the pushButton pin an input:
	pinMode(buttonPin, INPUT);
	// initialize control over the keyboard:
	Keyboard.begin();
#endif

#ifdef CDC_ENABLED
	SERIAL_PORT_USBVIRTUAL.begin(115200);
#endif
}

char testchar[10];
void loop(void)
{
#ifdef HID_ENABLED
	Mouse.move(1, 0, 0);

	// read the pushbutton:
	int buttonState = digitalRead(buttonPin);
	// if the button state has changed, and it's currently pressed:
	if ((buttonState != previousButtonState) && (buttonState == HIGH))
	{
		// increment the button counter
		counter++;
		// type out a message
		Keyboard.print("You pressed the button ");
		Keyboard.print(counter);
		Keyboard.println(" times.");
	}
	// save the current button state for comparison next time:
	previousButtonState = buttonState;
#endif

#ifdef CDC_ENABLED
	if (SERIAL_PORT_USBVIRTUAL.available() > 0)
	{
		char inChar;
		
		//while( -1 == (inChar = SERIAL_PORT_USBVIRTUAL.read()));
		inChar = SERIAL_PORT_USBVIRTUAL.read();
		
		SERIAL_PORT_USBVIRTUAL.print(inChar);
	}

	delay(10);
#endif
}

