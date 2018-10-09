/*
 Mouse Controller Example

 Shows the output of a USB Mouse connected to
 the Native USB port on an Arduino Due Board.

 created 8 Oct 2012
 by Cristian Maglie

 http://arduino.cc/en/Tutorial/MouseController

 This sample code is part of the public domain.
 */

// Require mouse control library
#include <MouseController.h>

// on a zero with debug port, use debug port
//#define SerialDebug Serial

// on a feather or non-debug Zero, use Serial1 (since USB is taken!)
#define SerialDebug Serial1

uint32_t lastUSBstate = 0;

// Initialize USB Controller
USBHost usb;

// Attach mouse controller to USB
MouseController mouse(usb);

// variables for mouse button states
bool leftButton = false;
bool middleButton = false;
bool rightButton = false;

// This function intercepts mouse movements
void mouseMoved() {
  SerialDebug.print("Move: ");
  SerialDebug.print(mouse.getXChange());
  SerialDebug.print(", ");
  SerialDebug.println(mouse.getYChange());
}

// This function intercepts mouse movements while a button is pressed
void mouseDragged() {
  SerialDebug.print("Drag: ");
  SerialDebug.print(mouse.getXChange());
  SerialDebug.print(", ");
  SerialDebug.println(mouse.getYChange());
}

// This function intercepts mouse button press
void mousePressed() {
  SerialDebug.print("Pressed: ");
  if (mouse.getButton(LEFT_BUTTON)) {
    SerialDebug.print("L");
    leftButton = true;
  }
  if (mouse.getButton(MIDDLE_BUTTON)) {
    SerialDebug.print("M");
    middleButton = true;
  }
  if (mouse.getButton(RIGHT_BUTTON)) {
    SerialDebug.print("R");
    rightButton = true;
  }
  SerialDebug.println();
}

// This function intercepts mouse button release
void mouseReleased() {
  SerialDebug.print("Released: ");
  if (!mouse.getButton(LEFT_BUTTON) && leftButton == true) {
    SerialDebug.print("L");
    leftButton = false;
  }
  if (!mouse.getButton(MIDDLE_BUTTON) && middleButton == true) {
    SerialDebug.print("M");
    middleButton = false;
  }
  if (!mouse.getButton(RIGHT_BUTTON) && rightButton == true) {
    SerialDebug.print("R");
    rightButton = false;
  }
  SerialDebug.println();
}

void setup()
{
  SerialDebug.begin( 115200 );
  SerialDebug.println("USB Host Mouse Controller Program started");

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
