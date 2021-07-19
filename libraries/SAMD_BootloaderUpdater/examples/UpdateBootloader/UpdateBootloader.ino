/*
  Update Bootloader

  This sketch checks if your Arduino SAMD board is running the latest bootloader.

  If it is not, it prompts you to update it.

  Circuit:
  - MKR Vidor 4000

  This example code is in the public domain.
*/


#include <SAMD_BootloaderUpdater.h>

void setup() {

  Serial.begin(9600);
  while (!Serial);

  Serial.println("Welcome to the Arduino SAMD bootloader updater");
  Serial.println("----------------------------------------------");
  Serial.println();

retry:
  Serial.print("Checking if bootloader requires an update ... ");
  if (SAMD_BootloaderUpdater.needsUpdate()) {
    Serial.println("bootloader is already the latest version");
    Serial.println();
    Serial.println("Update is not required :)");

    while (1);
  }

  Serial.println("bootloader is NOT running the latest");
  Serial.println();
  Serial.println();
  Serial.println("Would you like to proceed with updating it? (y/N)");
  Serial.println();


  while (Serial.available() == 0) {
    delay(100);
  }
  char in = Serial.read();
  if (in != 'y' && in != 'Y') {
    Serial.println("That's all folks!");
    while (1);
  }

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("WARNING: DO NOT UNPLUG the USB cable during the update!!!");
  Serial.println();
  Serial.println("Updating bootloader ...");
  Serial.println();

  if (!SAMD_BootloaderUpdater.update(onUpdateProgress)) {
    Serial.println("oh no! the bootloader failed to update :(");
    Serial.println();
    goto retry;
  }

  Serial.println();
  Serial.println("The bootloader was successfully updated \\o/");
  Serial.println();
  Serial.println("Your board will now start blinking in joy :)");
}

void onUpdateProgress(float percentage)
{
  // toggle the LED
  digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ? LOW : HIGH);

  // print out the percentage
  Serial.print(percentage);
  Serial.println("%");
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);

  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

