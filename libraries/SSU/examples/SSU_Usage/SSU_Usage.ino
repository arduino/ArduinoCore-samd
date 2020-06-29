/*
 Usage
 This example demonstrates how to use the SAMD SSU library to update a 
 sketch on any Arduino MKR board via the storage on the SARA U-201 GSM module.
 This sketch prints out the date and time the sketch was compiled.
 Steps to update sketch:
 1) Upload this sketch or another sketch that includes the SSU library
    via #include <SSU.h>
 2) Update the sketch as desired. For this example the sketch prints out
    the compiled date and time so no updates are needed.
 3) In the IDE select: Sketch -> Export compiled Binary
 4) Open the location of the sketch and convert the .bin file to a C byte array.
      cat SKETCH.bin | xxd --include > Binary.h
 5) Copy Binary.h file from the sketch's folder to the SSU_LoadBinary sketch
    and load it to the U-201 via SSU_LoadBinary sketch.
*/

/*
 Include the SSU library
 
 This will add some code to the sketch before setup() is called
 to check if UPDATE.BIN and UPDATE.OK are present on the storage of
 the U-201 module. If this theck is positive UPDATE.BIN is used to update
 the sketch running on the board.
 After this UPDATE.BIN and UPDATE.OK are deleted from the flash.
*/


#include <SSU.h>

void setup()
{
  Serial.begin(9600);
  while (!Serial) { }
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

void loop()
{
  // add you own code here
}