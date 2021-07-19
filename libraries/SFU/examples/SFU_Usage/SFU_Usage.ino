/*
 Usage
 This example demonstrates how to use the SAMD SFU library to update a 
 sketch on any Arduino MKR board connected to a MKRMEM Shield. This sketch
 prints out the date and time the sketch was compiled.

 Steps to update sketch via MKRMEM shield:

 1) Upload this sketch or another sketch that includes the SFU library
    via #include <SFU.h>

 2) Update the sketch as desired. For this example the sketch prints out
    the compiled date and time so no updates are needed.

 3) In the IDE select: Sketch -> Export compiled Binary

 4) Open the location of the sketch and convert the .bin file to a C byte array.
      cat SKETCH.bin | xxd --include > Binary.h

 5) Copy Binary.h file from the sketch's folder to the SFU_LoadBinary sketch
    and load it to the MKRMEM via SFU_LoadBinary sketch.
*/

/*
 Include the SFU library 
 
 This will add some code to the sketch before setup() is called
 to check if UPDATE.bin is present on the flash chip of the MKRMEM
 shield. If this theck is positive the file is used to update the sketch
 running on the board. After this UPDATE.BIN is deleted from the flash.
*/

#include <SFU.h>

void setup() {
  Serial.begin(9600);
  while(!Serial) { }

  // wait a bit
  delay(1000);

  String message;
  message += "Sketch compile date and time: ";
  message += __DATE__;
  message += " ";
  message += __TIME__;

  // print out the sketch compile date and time on the serial port
  Serial.println(message);
}

void loop() {
  // add you own code here
}
