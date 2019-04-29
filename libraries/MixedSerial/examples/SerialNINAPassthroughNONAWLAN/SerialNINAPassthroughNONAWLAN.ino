/*
  SerialNINAPassthrough - Use esptool to flash the u-blox NINA (ESP32) module
  Arduino MKR WiFi 1010, Arduino MKR Vidor 4000, and Arduino UNO WiFi Rev.2.

  Copyright (c) 2018 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifdef ARDUINO_SAMD_MKRVIDOR4000
#include <VidorPeripherals.h>

unsigned long baud = 119400;
#else
unsigned long baud = 115200;
#endif

int rts = -1;
int dtr = -1;

#include <MixedSerial.h>

#undef SerialNina
//SoftwareSerial SerialNina(29, 30);
MixedSerial SerialNina(Serial2, 30);

void setup() {
  Serial.begin(baud);
  while (!Serial) {}

#ifdef ARDUINO_SAMD_MKRVIDOR4000
  FPGA.begin();
#endif

  SerialNina.begin(baud);

#ifdef ARDUINO_SAMD_MKRVIDOR4000
  FPGA.pinMode(FPGA_NINA_GPIO0, OUTPUT);
  FPGA.pinMode(FPGA_SPIWIFI_RESET, OUTPUT);
#else
  pinMode(NINA_GPIO0, OUTPUT);
  pinMode(NINA_RESETN, OUTPUT);
#endif

  digitalWrite(NINA_GPIO0, HIGH);

  digitalWrite(NINA_RESETN, HIGH);
  delay(100);
  digitalWrite(NINA_RESETN, LOW);
  delay(100);
  digitalWrite(NINA_RESETN, HIGH);

#ifdef ARDUINO_AVR_UNO_WIFI_REV2
  // manually put the NINA in upload mode
  digitalWrite(NINA_GPIO0, LOW);

  digitalWrite(NINA_RESETN, LOW);
  delay(100);
  digitalWrite(NINA_RESETN, HIGH);
  delay(100);
  digitalWrite(NINA_RESETN, LOW);
#endif
}

long last = 0;

void loop() {

/*
  if (last - millis() > 10000) {
    // manually put the NINA in upload mode
    digitalWrite(NINA_GPIO0, HIGH);

    digitalWrite(NINA_RESETN, HIGH);
    delay(100);
    digitalWrite(NINA_RESETN, LOW);
    delay(100);
    digitalWrite(NINA_RESETN, HIGH);
    last = millis();
  }
*/
#ifndef ARDUINO_AVR_UNO_WIFI_REV2
  if (rts != Serial.rts()) {
#ifdef ARDUINO_SAMD_MKRVIDOR4000
    FPGA.digitalWrite(FPGA_SPIWIFI_RESET, (Serial.rts() == 1) ? LOW : HIGH);
#else
    digitalWrite(NINA_RESETN, Serial.rts()  ? LOW : HIGH);
#endif
    rts = Serial.rts();
  }

  if (dtr != Serial.dtr()) {
#ifdef ARDUINO_SAMD_MKRVIDOR4000
    FPGA.digitalWrite(FPGA_NINA_GPIO0, (Serial.dtr() == 1) ? HIGH : LOW);
#else
    digitalWrite(NINA_GPIO0, (Serial.dtr() == 0) ? HIGH : LOW);
#endif
    dtr = Serial.dtr();
  }
#endif

  if (Serial.available()) {
    SerialNina.write(Serial.read());
  }

  if (SerialNina.available()) {
    Serial.write(SerialNina.read());
  }

#ifndef ARDUINO_AVR_UNO_WIFI_REV2
  // check if the USB virtual serial wants a new baud rate
  if (Serial.baud() != baud) {
    rts = -1;
    dtr = -1;

    baud = Serial.baud();
#ifndef ARDUINO_SAMD_MKRVIDOR4000
    SerialNina.begin(baud);
#endif
  }
#endif
}
