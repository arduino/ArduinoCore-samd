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

// Initialize USB Controller
USBHost usb;

// Attach mouse controller to USB
MouseController mouse(usb);

// variables for mouse button states
boolean leftButton = false;
boolean middleButton = false;
boolean rightButton = false;

// This function intercepts mouse movements
void mouseMoved() {
  Serial5.print("Move: ");
  Serial5.print(mouse.getXChange());
  Serial5.print(", ");
  Serial5.println(mouse.getYChange());
}

// This function intercepts mouse movements while a button is pressed
void mouseDragged() {
  Serial5.print("DRAG: ");
  Serial5.print(mouse.getXChange());
  Serial5.print(", ");
  Serial5.println(mouse.getYChange());
}

// This function intercepts mouse button press
void mousePressed() {
  Serial5.print("Pressed: ");
  if (mouse.getButton(LEFT_BUTTON)) {
    Serial5.print("L");
    leftButton = true;
  }
  if (mouse.getButton(MIDDLE_BUTTON)) {
    Serial5.print("M");
    middleButton = true;
  }
  if (mouse.getButton(RIGHT_BUTTON)) {
    Serial5.print("R");
    Serial5.println();
    rightButton = true;
  }
}

// This function intercepts mouse button release
void mouseReleased() {
  Serial5.print("Released: ");
  if (!mouse.getButton(LEFT_BUTTON) && leftButton == true) {
    Serial5.print("L");
    leftButton = false;
  }
  if (!mouse.getButton(MIDDLE_BUTTON) && middleButton == true) {
    Serial5.print("M");
    middleButton = false;
  }
  if (!mouse.getButton(RIGHT_BUTTON) && rightButton == true) {
    Serial5.print("R");
    rightButton = false;
  }
  Serial5.println();
}

void setup()
{
  Serial5.begin( 115200 );
  while (!Serial5); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  Serial5.println("Mouse Controller Program started");

  if (usb.Init() == -1)
      Serial5.println("OSC did not start.");

  delay( 20 );
}

void loop()
{
  // Process USB tasks
  usb.Task();
}
