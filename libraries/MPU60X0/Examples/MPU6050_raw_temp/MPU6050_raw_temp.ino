// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2011-10-07 - initial release
//     2013-09-21 - modified by Mike Smorto to experiment with Motion sesnor
//					and temperature compensation.
//     2013-10-15 - deleted motion sensor to concentrate on temperature compensation only
//                - added magnetometer raw values

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"
#include "SPI.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU60X0.h"
#include "HMC58X3.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU60X0 accelgyro;
HMC58X3 magn;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t rawTemp;
float temp;

#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    //Serial.println("Initializing I2C devices...");
	//#define MPU60X0_GYRO_FS_250         0x00
	//#define MPU60X0_GYRO_FS_500         0x01
	//#define MPU60X0_GYRO_FS_1000        0x02
	//#define MPU60X0_GYRO_FS_2000        0x03
	//----------------------------------------------
	//#define MPU60X0_ACCEL_FS_2          0x00
	//#define MPU60X0_ACCEL_FS_4          0x01
	//#define MPU60X0_ACCEL_FS_8          0x02
	//#define MPU60X0_ACCEL_FS_16         0x03
	//-----------------------------------------------
	/**  Gryro Specs
	*	FS_SEL	Full Scale Range	LSB Sensitivity
    *	  0			± 250 °/s		 131   LSB/°/s
	*	  1			± 500 °/s		  65.5 LSB/°/s
	*	  2			± 1000 °/s		  32.8 LSB/°/s
	*	  3			± 2000 °/s		  16.4 LSB/°/s
	
	*	    Accel Specs	
	*	AFS_SEL  Full Scale Range	LSB Sensitivity
	*	  0			±2g					16384 LSB/g
	*	  1			±4g					 8192 LSB/g
	*	  2			±8g					 4096 LSB/g
	*	  3			±16g				 2048 LSB/g
	**/
	//-------------------------------------------------
    accelgyro.initialize();
	accelgyro.setFullScaleGyroRange(0x03);      //for freeIMU its set to 0x3
	delay(5);
	//accelgyro.setFullScaleAccelRange(0x02);
	//delay(5);
	
    // verify connection
    //Serial.println("Testing device connections...");
    //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	//Setup magnetometer
    magn.init(false); // Don't set mode yet, we'll do that later on.
    // Calibrate HMC using self test, not recommended to change the gain after calibration.
    magn.calibrate(1); // Use gain 1=default, valid 0-7, 7 not recommended.
    // Single mode conversion was used in calibration, now set continuous mode
    magn.setMode(0);
    delay(10);
    magn.setDOR(B110);
	
    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
}

void loop() {

    int ix,iy,iz;
	
    // read raw accel/gyro measurements from device
    //Serial.println("Getting raw accwl/gyro measurements");
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    /** Get current internal temperature.
    * @return Temperature reading in 16-bit 2's complement format
    * @see MPU60X0_RA_TEMP_OUT_H
    */
    //Serial.println("Getting Die Temperature");
    rawTemp = accelgyro.getTemperature();	
    temp=(rawTemp/340.)+36.53;
	
	//Get magnetometer readings
	magn.getValues(&ix,&iy,&iz);

    /*  The accelerometer and gyroscope measurements are explained in the MPU-6050 
    * datasheet in the GYRO_CONFIG and ACCEL_CONFIG register descriptions (sections 4.4 
    * and 4.5 on pages 14 and 15). The scale of each depends on the sensitivity settings 
    * chosen, which can be one of +/- 2, 4, 8, or 16g for the accelerometer and one of 
    * +/- 250, 500, 1000, or 2000 deg/sec for the gyroscope. The accelerometer produces data 
    * in units of acceleration (distance over time2), and the gyroscope produces data in units
    * of rotational velocity (rotation distance over time).
    * 
    * The output scale for any setting is [-32768, +32767] for each of the six axes. The default 
    * setting in the I2Cdevlib class is +/- 2g for the accel and +/- 250 deg/sec for the gyro. If 
    * the device is perfectly level and not moving, then:
    * 		X/Y accel axes should read 0
    * 		Z accel axis should read 1g, which is +16384 at a sensitivity of 2g
    * 		X/Y/Z gyro axes should read 0
    * 
    * In reality, the accel axes won't read exactly 0 since it is difficult to be perfectly level 
    * and there is some noise/error, and the gyros will also not read exactly 0 for the same reason
    * (noise/error). - quote from Jeff Rowberg extracted from his forum
    */
	
    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
    //16384 accel conversion, 131.072 gyro conversion for default setting of 2g,
	Serial.print(temp);Serial.print(",");
	Serial.print(rawTemp);Serial.print(",");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.print(gz); Serial.print(",");
    Serial.print(ix); Serial.print(",");	
    Serial.print(iy); Serial.print(",");	
    Serial.print(iz); Serial.print(",");	
	Serial.print(millis());Serial.println(",\r\n");
	
    delay(100);
	
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}