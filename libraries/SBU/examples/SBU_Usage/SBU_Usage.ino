/*
 Usage
 This example demonstrates how to use the SAMD SBU library to update a 
 sketch on any Arduino MKR board via the storage on the SARA-R410M module.
 This sketch prints out the current date and time.
 Steps to update sketch:
 1) Upload this sketch or another sketch that includes the SBU library
 2) Update the sketch as desired. For this example the sketch prints out
    the compiled date and time.
 3) In the IDE select: Sketch -> Export compiled Binary
 4) Open the location of the sketch and convert the .bin file to a C byte array.
      cat SKETCH.bin | xxd --include > Binary.h
 5) Copy Binary.h file from the sketch's folder to the SBU_LoadBinary sketch
    and load it to the SARA-R410M via SBU_LoadBinary sketch.
*/

/*
 Include the SBU library
 
 This will add some code to the sketch before setup() is called
 to check if UPDATE.BIN and UPDATE.OK are present on the storage of
 the SARA-R410M module. If this check is positive UPDATE.BIN is used to update
 the sketch running on the board.
 After this UPDATE.BIN and UPDATE.OK are deleted from the flash.
*/


#include <SBU.h>

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
