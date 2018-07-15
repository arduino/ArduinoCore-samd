/**
 * FreeIMU library serial communication protocol
*/
//These are optional depending on your IMU configuration


#include <HMC58X3.h>
#include <MS561101BA.h> //Comment out for APM 2.5
#include <I2Cdev.h>
#include <MPU60X0.h>
#include <AK8975.h>
#include <AK8963.h>

//#include <AP_Baro_MS5611.h>  //Uncomment for APM2.5


//These are mandatory
#include <AP_Math_freeimu.h>
#include <Butter.h>    // Butterworth filter
#include <iCompass.h>
#include <MovingAvarageFilter.h>

#include <Wire.h>
#include <SPI.h>

#if defined(__AVR__)
	#include <EEPROM.h>
#endif

//#define DEBUG
//#include "DebugUtils.h"
#include "CommunicationUtils.h"
#include "FreeIMU_6050.h"
#include "DCM.h"
#include "FilteringScheme.h"
#include "RunningAverage.h"

//int raw_values[11];
//char str[512];
float q[4];
float ypr[3]; // yaw pitch roll
float val[12];

// Set the FreeIMU object
FreeIMU_6050 my3IMU = FreeIMU_6050();

void setup() {
  Serial.begin(57600);
  Wire.begin();
  
  delay(5);
  my3IMU.init(true); // the parameter enable or disable fast mode
  delay(5);
  my3IMU.initGyros();
}

void loop() { 

  my3IMU.getEuler360degAttitude(ypr, q, val);
  Serial.print("Yaw: ");
  Serial.print(ypr[0] );
  Serial.print(" Pitch: ");
  Serial.print(ypr[1] );
  Serial.print(" Roll: ");
  Serial.print(ypr[2]);
  
  Serial.print(" MagHead: ");
  Serial.print(val[9]);
  Serial.print(" Altitude: ");
  Serial.print(val[10]);
  
  Serial.println("");
  
  delay(10);
}

