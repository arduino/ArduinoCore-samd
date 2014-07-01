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
#include "Arduino.h" 

#ifdef HID_ENABLED
	const int buttonPin = 4;          // input pin for pushbutton
	int previousButtonState = HIGH;   // for checking the state of a pushButton
	int counter = 0;                  // button push counter
#endif

void setup(void)
{	
#ifdef HID_ENABLED
	Mouse.begin();

	// make the pushButton pin an input:
	pinMode(buttonPin, INPUT);
	// initialize control over the keyboard:
	Keyboard.begin();
#endif

#ifdef CDC_ENABLED
	SerialUSB.begin(115200);
#endif
}


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
	if (SerialUSB.available() > 0)
	{
		char inChar;
		while( -1 == (inChar = SerialUSB.read()));
		SerialUSB.print(inChar);
	}

	delay(10);
#endif
}

