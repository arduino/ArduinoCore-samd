/*
 Usage
 This example demonstrates how to use the SAMD SDU library to update a 
 sketch on an Arduino/Genuino Zero, MKRZero or MKR1000 board using an
 SD card. It prints out the date and time the sketch was compiled at 
 to both Serial and Serial1.

 Circuit:
 * Arduino MKRZero board with SD card
 OR
 * Arduino/Genuino Zero or MKR1000 board
 * SD shield or breakout connected with CS pin of 4
 * SD card

 Non-Arduino/Genuino Zero, MKRZero or MKR1000 board are NOT supported.

 Steps to update sketch via SD card:

 1) Upload this sketch or another sketch that includes the SDU library
    via #include <SDU.h>

 2) Update the sketch as desired. For this example the sketch prints out
    the compiled date and time so no updates are needed.

 3) In the IDE select: Sketch -> Export compiled Binary

 4) Copy the .bin file from the sketch's folder to the SD card and rename
    the file to UPDATE.bin. Eject the SD card from your PC.

 5) Insert the SD card into the board, shield or breakout and press the
    reset button or power cycle the board. The SDU library will then update
    the sketch on the board with the contents of UPDATE.bin

 created 23 March 2017
 by Sandeep Mistry
*/

/*
 Include the SDU library 
 
 This will add some code to the sketch before setup() is called
 to check if an SD card is present and UPDATE.bin exists on the
 SD card.
 
 If UPDATE.bin is present, the file is used to update the sketch
 running on the board. After this UPDATE.bin is deleted from the
 SD card.
*/
#include <SDU.h>

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

