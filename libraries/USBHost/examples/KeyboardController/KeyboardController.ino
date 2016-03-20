/*
 Keyboard Controller Example

 Shows the output of a USB Keyboard connected to
 the Native USB port on an Arduino Zero

 created 8 Oct 2012
 by Cristian Maglie

 http://arduino.cc/en/Tutorial/KeyboardController

 This sample code is part of the public domain.
 */

// Require keyboard control library
#include <KeyboardController.h>


// on a zero with debug port, use debug port
//#define SerialDebug Serial

// on a feather or non-debug Zero, use Serial1 (since USB is taken!)
#define SerialDebug Serial1

// Initialize USB Controller
USBHost usb;

// Attach keyboard controller to USB
KeyboardController keyboard(usb);

void printKey();

// This function intercepts key press
void keyPressed() {
  SerialDebug.print("Pressed:  ");
  printKey();
}

// This function intercepts key release
void keyReleased() {
  SerialDebug.print("Released: ");
  printKey();
}

void printKey() {
  // getOemKey() returns the OEM-code associated with the key
  SerialDebug.print(" key:");
  SerialDebug.print(keyboard.getOemKey());

  // getModifiers() returns a bits field with the modifiers-keys
  int mod = keyboard.getModifiers();
  SerialDebug.print(" mod:");
  SerialDebug.print(mod);

  SerialDebug.print(" => ");

  if (mod & LeftCtrl)
    SerialDebug.print("L-Ctrl ");
  if (mod & LeftShift)
    SerialDebug.print("L-Shift ");
  if (mod & Alt)
    SerialDebug.print("Alt ");
  if (mod & LeftCmd)
    SerialDebug.print("L-Cmd ");
  if (mod & RightCtrl)
    SerialDebug.print("R-Ctrl ");
  if (mod & RightShift)
    SerialDebug.print("R-Shift ");
  if (mod & AltGr)
    SerialDebug.print("AltGr ");
  if (mod & RightCmd)
    SerialDebug.print("R-Cmd ");

  // getKey() returns the ASCII translation of OEM key
  // combined with modifiers.
  SerialDebug.write(keyboard.getKey());
  SerialDebug.println();
}

uint32_t lastUSBstate = 0;

void setup()
{
  SerialDebug.begin( 115200 );
  SerialDebug.println("USB Host Keyboard Controller Program started");

  if (usb.Init() == -1)
	  SerialDebug.println("USB Host did not start.");

  SerialDebug.println("USB Host started");
  delay( 20 );
}

void loop()
{
  // Process USB tasks
  usb.Task();

  uint32_t currentUSBstate = usb.getUsbTaskState();
  if (lastUSBstate != currentUSBstate) {
    SerialDebug.print("USB state changed: 0x"); 
    SerialDebug.print(lastUSBstate, HEX); 
    SerialDebug.print(" -> 0x"); 
    SerialDebug.println(currentUSBstate, HEX);
    switch (currentUSBstate) {
      case USB_ATTACHED_SUBSTATE_SETTLE: SerialDebug.println("Device Attached"); break;
      case USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE: SerialDebug.println("Detached, waiting for Device"); break;
      case USB_ATTACHED_SUBSTATE_RESET_DEVICE: SerialDebug.println("Resetting Device"); break;
      case USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE: SerialDebug.println("Reset complete"); break;
      case USB_STATE_CONFIGURING: SerialDebug.println("USB Configuring"); break;
      case USB_STATE_RUNNING: SerialDebug.println("USB Running"); break;
    }
    lastUSBstate = currentUSBstate;
  }
}
