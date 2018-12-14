/*
 Usage
 This example demonstrates how to use the SAMD SFU library to update a
 sketch on an Arduino/Genuino Zero, MKRZero or MKR1000 board using an
 SPI Flash memory chip. It prints out the date and time the sketch was compiled at
 to both Serial and Serial1.

 Circuit:
 * Arduino MKR board with MKR MEM Shield (https://store.arduino.cc/arduino-mkr-mem-shield)

 Steps to update sketch via SerialFlash:

 1) Upload this sketch or another sketch that includes the SFU library
    via #include <SFU.h>

 2) Update the sketch as desired. For this example the sketch prints out
    the compiled date and time so no updates are needed.

 3) In the IDE select: Sketch -> Export compiled Binary

 4) Write the .bin file to the flash chip in the way you prefer (eg. using https://github.com/PaulStoffregen/SerialFlash/blob/master/examples/CopyFromSerial/CopyFromSerial.ino). The filename MUST be "UPDATE.bin".

 5) Reboot the board; the new binary will be automatically flashed to the board.

 created 12 March 2018
 by Martino Facchin
*/

/*
 Include the SFU library 
 
 This will add some code to the sketch before setup() is called
 to check if a serial flash is present and UPDATE.bin exists on it.

 If UPDATE.bin is present, the file is used to update the sketch
 running on the board. After this UPDATE.bin is deleted from flash.
*/
#include <SFU.h>

String message;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  // wait a bit
  delay(1000);

  message += "Sketch compile date and time: ";
  message += __DATE__;
  message += " ";
  message += __TIME__;

  // print out the sketch compile date and time on the serial port
  Serial.println(message);
  Serial1.println(message);
}

void loop() {
  // add you own code here
}

