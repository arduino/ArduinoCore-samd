/*
  Serial Hello

  Upload to your ATSAMD21E18A (ARM) board via the native USB port.
  Once upload is done, open up the Serial Window. 
  The word "Hello" should be output every second.

  Your FemtoUSB (ATSAMD21E18A based) requires the Arduino Zero bootloader.

  This example code is in the public domain.

  modified 11 February 2016
  by Alejandro Albino

  See https://github.com/femtoio/femto-usb
 */


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  while(!Serial);
  SerialUSB.begin(9600);
  
}

// the loop function runs over and over again forever
void loop() {
  SerialUSB.println("Hello!");
  delay(1000);
}
