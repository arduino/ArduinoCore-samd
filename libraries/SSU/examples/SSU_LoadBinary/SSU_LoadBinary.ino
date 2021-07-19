/**************************************************************************************
 * INCLUDE
 **************************************************************************************/


#include <MKRGSM.h>


/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static char const BINARY[] =

{
  #include "Binary.h"
};


GSMFileUtils fileUtils;


/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup() {
  Serial.begin(9600);

  unsigned long const start = millis();
  for(unsigned long now = millis(); !Serial && ((now - start) < 5000); now = millis()) { };

  Serial.print("Accessing SARA U-201 Filesystem... ");
  if(!fileUtils.begin()) {
    Serial.println("failed.");
    return;

  }
  Serial.println("OK");
  Serial.print("Writing \"UPDATE.BIN\" ... ");

  uint32_t bytes_to_write = sizeof(BINARY);
  auto bytes_written = fileUtils.downloadFile("UPDATE.BIN", BINARY, bytes_to_write);

  if(bytes_written != bytes_to_write) {
    Serial.println("downloadFile failed.");return;

  } else {
    Serial.print("OK (");
    Serial.print(bytes_written);
    Serial.println(" bytes written)");
  }
}

void loop() {

}
