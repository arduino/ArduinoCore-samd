#include <MKRNB.h>
#include <SBU.h>

static char const BINARY[] =
{
  #include "Binary.h"
};

static char const CHECK_FILE[] =
{
  "OK" 
};

static constexpr char CHECK_FILE_NAME[] = "UPDATE.OK";

NBFileUtils fileUtils;
bool update_available = false;

void setup() {
  Serial.begin(9600);
  while(!Serial) { }

  unsigned long const start = millis();
  for(unsigned long now = millis(); !Serial && ((now - start) < 5000); now = millis()) { };

  Serial.print("Accessing SARA Filesystem... ");
  if(!fileUtils.begin(false)) {
    Serial.println("failed.");
    return;

  }
  Serial.println("OK");
  Serial.print("Writing \"UPDATE.BIN\" ... ");
  
  uint32_t bytes_to_write = sizeof(BINARY);
  Serial.print("Size of BINARY: ");
  Serial.println(bytes_to_write);
  int index = 0;
  bool append = false;
  int new_bytes = 0;
  //int bytes_written = 0;
  
  for (int i=0; i<(bytes_to_write/512); i++) {
    auto new_bytes = fileUtils.downloadFile("UPDATE.BIN", BINARY+index, 512, append);
    if (new_bytes != 512) {
      Serial.print("New_bytes = ");
      Serial.print(new_bytes);
      Serial.println(" != 512");
    }
    index = index + new_bytes;
    append = true;
  }
  if ((bytes_to_write%512)!=0) {  
    auto new_bytes = fileUtils.downloadFile("UPDATE.BIN", BINARY+index, bytes_to_write%512, append);
    if (new_bytes != bytes_to_write%512) {
      Serial.print("Last bytes read = ");
      Serial.print(new_bytes);
      Serial.print(". They should have been ");
      Serial.println(bytes_to_write%512);
    }
    index = index + new_bytes;
  }

  if(index != bytes_to_write) {
    Serial.print("Written only ");
    Serial.println(index); //bytes_written
    Serial.print(bytes_to_write);
    Serial.println(" should have been written. System is restarting...");
    delay(100);
    NVIC_SystemReset();

  } else {
    Serial.print("Download complete! ");
    Serial.print(index);
    Serial.println(" bytes written");

    auto status = 0;
    while (status != 2) {
      status = fileUtils.createFile(CHECK_FILE_NAME, CHECK_FILE, 2);
      delay(100);
    }
    
    Serial.println("Please type \"restart\" to apply the update");
    update_available = true;
  }
  
}

void loop() {
  if (update_available == true) {
    String command = Serial.readStringUntil('\n');
    if (command.indexOf("restart") >= 0) {
      NVIC_SystemReset();
    }
  }
}
