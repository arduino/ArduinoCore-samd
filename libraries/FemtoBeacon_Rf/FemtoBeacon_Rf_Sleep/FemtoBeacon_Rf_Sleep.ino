#include <Arduino.h>
#include <Wire.h>

//#define Serial SERIAL_PORT_USBVIRTUAL

#include <RTCZero.h>
RTCZero rtc;
volatile bool shouldBeSleeping = false;
volatile int theSeconds = 0;
volatile int awakeCalls = 0;

const byte seconds = 0;
const byte minutes = 00;
const byte hours = 17;

const byte day = 17;
const byte month = 11;
const byte year = 15;

void setup() {
  SYSCTRL->VREG.bit.RUNSTDBY = 1; // Regulator, run in normal mode when standby mode is activated.
  SYSCTRL->DFLLCTRL.bit.RUNSTDBY = 1; // Enable the DFLL48M clock in standby mode!

  
  
  // put your setup code here, to run once:
  rtc.begin();
  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(day, month, year);

  rtc.setAlarmTime(17, 00, 10);

  rtc.attachInterrupt(alarmMatch);
  while (!SerialUSB);

  SerialUSB.begin(115200);

  
  SerialUSB.println("Ready!");
  SerialUSB.end();

//  rtc.standbyMode();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  SerialUSB.print("...Going into Standby Mode. Awake calls:");
  SerialUSB.println(awakeCalls);
  SerialUSB.end();
  
  
  USBDevice.detach();
  delay(1000);

  theSeconds = rtc.getSeconds() + 5;
  theSeconds = theSeconds % 60;
//
  rtc.attachInterrupt(alarmMatch);
  rtc.setAlarmSeconds(theSeconds);
  rtc.enableAlarm(rtc.MATCH_SS);
  
  rtc.standbyMode();
//  delay(5000);

  USBDevice.attach();

  delay(1000);
  
  while(!SerialUSB);
  Serial.begin(115200);

  SerialUSB.print("...Completed standby. Awake calls: ");
  SerialUSB.println(awakeCalls);
  SerialUSB.end();
}

void alarmMatch() {
  // Do something!
  ++awakeCalls;
//  USBDevice.attach();
//
//  while(!SerialUSB);
//  SerialUSB.println("Alarm Match!");
//  SerialUSB.end();
  
}
