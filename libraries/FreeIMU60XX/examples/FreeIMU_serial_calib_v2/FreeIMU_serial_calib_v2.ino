//These are mandatory
#include <AP_Math_freeimu.h>
#include <Butter.h>    // Butterworth filter
#include <iCompass.h>
#include <MovingAvarageFilter.h>

/**
 * FreeIMU library serial communication protocol
*/

#include <HMC58X3.h>
#include <MS561101BA.h>
#include <I2Cdev.h>
#include <MPU60X0.h>
#include <AK8975.h>
#include <AK8963.h>

#include <Wire.h>
#include <SPI.h>

//#define DEBUG
#include "DebugUtils.h"
#include "CommunicationUtils.h"
#include "FreeIMU_6050.h"
#include "DCM.h"
#include "FilteringScheme.h"
#include "RunningAverage.h"


int16_t raw_values[11];
char str[128];

// Set the FreeIMU object and LSM303 Compass
FreeIMU_6050 my3IMU = FreeIMU_6050();

//The command from the PC
char cmd, tempCorr;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  //#if HAS_MPU6050()
  //    my3IMU.RESET();
  //#endif
	
  my3IMU.init(true);

 
  // LED
  pinMode(13, OUTPUT);
}

void loop() {
  if(Serial.available()) {
    cmd = Serial.read();
    if(cmd=='v') {
      //sprintf(str, "FreeIMU library by FREQ: LIB_VERSION: %s", FREEIMU_LIB_VERSION);
      Serial.print("OK....");
      Serial.print('\n');
    }
    /*else if(cmd=='r') {
      uint8_t count = serial_busy_wait();
      for(uint8_t i=0; i<count; i++) {
        //my3IMU.getUnfilteredRawValues(raw_values);
        my3IMU.getRawValues(raw_values);
        sprintf(str, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,", raw_values[0], raw_values[1], raw_values[2], raw_values[3], raw_values[4], raw_values[5], raw_values[6], raw_values[7], raw_values[8], raw_values[9]);
        Serial.print(str);
        //#if (HAS_MS5611() )
        //  Serial.print(my3IMU.getBaroTemperature()); Serial.print(",");
        //  Serial.print(my3IMU.getBaroPressure()); Serial.print(",");
        //#endif
        Serial.print(millis()); Serial.print(",");
        Serial.println("\r\n");
     }
    } */
    else if(cmd=='b') {
      uint8_t count = serial_busy_wait();
      for(uint8_t i=0; i<count; i++) {
      #if HAS_MPU9150()  || HAS_MPU9250()
          my3IMU.getRawValues(raw_values);
          writeArr(raw_values, 9, sizeof(int)); // writes accelerometer, gyro values & mag if 9150
        #elif HAS_MPU6050() || HAS_MPU6000()   // MPU6050
          //my3IMU.accgyro.getMotion6(&raw_values[0], &raw_values[1], &raw_values[2], &raw_values[3], &raw_values[4], &raw_values[5]);
          my3IMU.getRawValues(raw_values);
          writeArr(raw_values, 6, sizeof(int)); // writes accelerometer, gyro values & mag if 9150
        #endif
        //writeArr(raw_values, 6, sizeof(int)); // writes accelerometer, gyro values & mag if 9150
        
        #if IS_9DOM() && (!HAS_MPU9150()  && !HAS_MPU9250() )
          my3IMU.magn.getValues(&raw_values[0], &raw_values[1], &raw_values[2]);
          writeArr(raw_values, 3, sizeof(int));
        #endif
        Serial.println();
      }
    }

  }
}

char serial_busy_wait() {
  while(!Serial.available()) {
    ; // do nothing until ready
  }
  return Serial.read();
}




