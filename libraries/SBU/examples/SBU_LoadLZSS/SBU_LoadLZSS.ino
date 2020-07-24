#include <MKRNB.h>
#include <SBU.h>

#include "lzssEncode.h"

static char const BINARY[] =
{
#include "Binary.h"
};

static char const CHECK_FILE[] =
{
  "OK"
};

static constexpr char CHECK_FILE_NAME[] = "UPDATE.OK";
const char * UPDATE_FILE_NAME_LZSS = "UPDATE.BIN.LZSS";

NBFileUtils fileUtils;
bool update_available = false;

void setup() {
  Serial.begin(9600);
  while (!Serial) { }

  unsigned long const start = millis();
  for (unsigned long now = millis(); !Serial && ((now - start) < 5000); now = millis()) { };

  Serial.print("Accessing SARA Filesystem... ");
  if (!fileUtils.begin(false)) {
    Serial.println("failed.");
    return;

  }
  Serial.println("OK");

  uint32_t bytes_to_write = sizeof(BINARY);
  Serial.print("Size of BINARY.H: ");
  Serial.println(bytes_to_write);

  Serial.print("Encoding \"BINARY.H\" into \"UPDATE.BIN.LZSS\" and writing it into the Sara-R410M module ... ");
  
  //Encode into .lzss and write to the Sara modem
  int bytes_written = lzss_encode(BINARY, bytes_to_write);

  if (bytes_written == 0) {
    Serial.println("something went wrong!");
  } else {
    Serial.println("OK!");
  }

  Serial.print("Size of UPDATE.BIN.LZSS: ");
  Serial.println(bytes_written);

  auto status = 0;
  while (status != 2) {
    status = fileUtils.createFile(CHECK_FILE_NAME, CHECK_FILE, 2);
    delay(100);
  }

  Serial.println("Please type \"restart\" to apply the update");
  update_available = true;
}


void loop() {
  if (update_available == true) {
    String command = Serial.readStringUntil('\n');
    if (command.indexOf("restart") >= 0) {
      NVIC_SystemReset();
    }
  }
}
