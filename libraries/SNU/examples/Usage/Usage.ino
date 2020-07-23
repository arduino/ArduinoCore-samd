/*
  Usage
  This example demonstrates how to use the SAMD SNU library to update a
  sketch on an Arduino MKRWiFi1010 board using the board and nothing else.
  It prints out the date and time the sketch was compiled at
  to both Serial and Serial1.

   Arduino MKRWiFi1010 board

  Steps to update sketch via NINA WiFi/BT module:

  1) Upload this sketch or another sketch that includes the SNU library
    via #include <SNU.h>

  2) Update the sketch as desired. For this example the sketch prints out
    the compiled date and time so no updates are needed.

  3) In the IDE select: Sketch -> Export compiled Binary

  4) Use WiFiStorage.download(url, "UPDATE.BIN") function to download the
    new binary from a remote webserver.

  5) Reboot the board; the update will be applied seamlessly

  created 14 December 2018
  by Martino Facchin
*/

/*
  Include the SNU library

  This will add some code to the sketch before setup() is called
  to check if the WiFi module is present and UPDATE.bin exists.

  If UPDATE.bin is present, the file is used to update the sketch
  running on the board. After this UPDATE.bin is deleted from NINA memory.
*/
#include <SNU.h>
#include <WiFiNINA.h>

#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
/////// Wifi Settings ///////
char ssid[] = SECRET_SSID;      // your network SSID (name)
char pass[] = SECRET_PASS;   // your network password
char url[] = SECRET_OTA_URL;

int status = WL_IDLE_STATUS;

String message;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
  }

  // wait a bit
  delay(1000);

  message += "Sketch compile date and time: ";
  message += __DATE__;
  message += " ";
  message += __TIME__;

  // print out the sketch compile date and time on the serial port
  Serial.println(message);
  Serial1.println(message);

  Serial.println("Type \"download\" in the Serial Monitor to start downloading the update");
}

void loop() {
  String command = Serial.readStringUntil('\n');

  if (command.indexOf("download") >= 0) {

    Serial.println("Downloading update file");
    WiFiStorage.download(url, "UPDATE.BIN");

    WiFiStorageFile update = WiFiStorage.open("/fs/UPDATE.BIN");
    if (update.available()) {
      Serial.println("Download complete, please restart or type \"restart\" to apply the update");
      Serial.println("Filesize: " + String(update.available()));
    } else {
      Serial.println("Download failed, please retry :(");
    }
  }

  if (command.indexOf("restart") >= 0) {
      NVIC_SystemReset();
  }
}
