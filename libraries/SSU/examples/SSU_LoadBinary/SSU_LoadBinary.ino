/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Arduino_MKRMEM.h>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint8_t const BINARY[] =
{
  #include "Binary.h"
};

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup() {
  Serial.begin(9600);

  unsigned long const start = millis();
  for(unsigned long now = millis(); !Serial && ((now - start) < 5000); now = millis()) { };
  
  flash.begin();

  Serial.print("Mounting ... ");
  if(SPIFFS_OK != filesystem.mount()) {
    Serial.println("mount() failed with error code "); Serial.println(filesystem.err()); return;
  }
  Serial.println("OK");


  Serial.print("Checking ... ");
  if(SPIFFS_OK != filesystem.check()) {
    Serial.println("check() failed with error code "); Serial.println(filesystem.err()); return;
  }
  Serial.println("OK");


  Serial.print("Writing \"UPDATE.BIN\" ... ");
  File file = filesystem.open("UPDATE.BIN", CREATE | READ_WRITE| TRUNCATE);

  int const bytes_to_write = sizeof(BINARY);
  int const bytes_written = file.write((void *)BINARY, bytes_to_write);
  
  if(bytes_written != bytes_to_write) {
    Serial.println("write() failed with error code "); Serial.println(filesystem.err()); return;
  } else {
    Serial.print("OK (");
    Serial.print(bytes_written);
    Serial.println(" bytes written)");
  }

  Serial.print("Unmounting ... ");
  filesystem.unmount();
  Serial.println("OK");
}

void loop() {

}
