/*
 Keyboard Controller Example

 Shows the output of a USB Keyboard connected to
 the Native USB port on an Arduino Due Board.

 created 8 Oct 2012
 by Cristian Maglie

 http://arduino.cc/en/Tutorial/KeyboardController

 This sample code is part of the public domain.
 */

// Require keyboard control library
#include <KeyboardController.h>

// Initialize USB Controller
USBHost usb;

// Attach keyboard controller to USB
KeyboardController keyboard(usb);

void printKey();

// This function intercepts key press
void keyPressed() {
  Serial5.print("Pressed:  ");
  printKey();
}

// This function intercepts key release
void keyReleased() {
  Serial5.print("Released: ");
  printKey();
}

void printKey() {
  // getOemKey() returns the OEM-code associated with the key
  Serial5.print(" key:");
  Serial5.print(keyboard.getOemKey());

  // getModifiers() returns a bits field with the modifiers-keys
  int mod = keyboard.getModifiers();
  Serial5.print(" mod:");
  Serial5.print(mod);

  Serial5.print(" => ");

  if (mod & LeftCtrl)
    Serial5.print("L-Ctrl ");
  if (mod & LeftShift)
    Serial5.print("L-Shift ");
  if (mod & Alt)
    Serial5.print("Alt ");
  if (mod & LeftCmd)
    Serial5.print("L-Cmd ");
  if (mod & RightCtrl)
    Serial5.print("R-Ctrl ");
  if (mod & RightShift)
    Serial5.print("R-Shift ");
  if (mod & AltGr)
    Serial5.print("AltGr ");
  if (mod & RightCmd)
    Serial5.print("R-Cmd ");

  // getKey() returns the ASCII translation of OEM key
  // combined with modifiers.
  Serial5.write(keyboard.getKey());
  Serial5.println();
}

void setup()
{
  Serial5.begin( 115200 );
  while (!Serial5); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  Serial5.println("Keyboard Controller Program started");

  if (usb.Init() == -1)
	  Serial5.println("OSC did not start.");
  
  delay( 20 );
}

void loop()
{
  // Process USB tasks
  usb.Task();
}
