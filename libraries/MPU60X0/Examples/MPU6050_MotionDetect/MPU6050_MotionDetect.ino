// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2011-10-07 - initial release
//     2013-09-21 - modified by Mike Smorto to experiment with Motion sesnor
//					and temperature compensation.

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

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU60X0 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int8_t threshold, count; 
float temp;
bool zero_detect; 
bool TurnOnZI = false;

bool XnegMD, XposMD, YnegMD, YposMD, ZnegMD, ZposMD;

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
    *	  0			± 250 °/s		 131 LSB/°/s
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
	accelgyro.setFullScaleGyroRange(MPU60X0_GYRO_FS_2000); //for freeIMU its set to 0x3
	dealy(5)
	accelgyro.setFullScaleAccelRange(MPU60X0_ACCEL_FS_2);
	delay(5)
	
    // verify connection
    //Serial.println("Testing device connections...");
    //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    //Set up zero motion
	
    /** Get accelerometer power-on delay.
    * The accelerometer data path provides samples to the sensor registers, Motion
    * detection, Zero Motion detection, and Free Fall detection modules. The
    * signal path contains filters which must be flushed on wake-up with new
    * samples before the detection modules begin operations. The default wake-up
    * delay, of 4ms can be lengthened by up to 3ms. This additional delay is
    * specified in ACCEL_ON_DELAY in units of 1 LSB = 1 ms. The user may select
    * any value above zero unless instructed otherwise by InvenSense. Please refer
    * to Section 8 of the MPU-6000/MPU-6050 Product Specification document for
    * further information regarding the detection modules.
    * @return Current accelerometer power-on delay
    * @see MPU60X0_RA_MOT_DETECT_CTRL
    * @see MPU60X0_DETECT_ACCEL_ON_DELAY_BIT
    */	
    accelgyro.setAccelerometerPowerOnDelay(3);
	

    /** Get Zero Motion Detection interrupt enabled status.
    * Will be set 0 for disabled, 1 for enabled.
    * @return Current interrupt enabled status
    * @see MPU60X0_RA_INT_ENABLE
    * @see MPU60X0_INTERRUPT_ZMOT_BIT
    **/	
    accelgyro.setIntZeroMotionEnabled(TurnOnZI);
	

    /** Get the high-pass filter configuration.
    * The DHPF is a filter module in the path leading to motion detectors (Free
    * Fall, Motion threshold, and Zero Motion). The high pass filter output is not
    * available to the data registers (see Figure in Section 8 of the MPU-6000/
    * MPU-6050 Product Specification document).
    * 
    * The high pass filter has three modes:
    *    Reset: The filter output settles to zero within one sample. This
    *           effectively disables the high pass filter. This mode may be toggled
    *           to quickly settle the filter.
    *
    *    On:    The high pass filter will pass signals above the cut off frequency.
    *
    *    Hold:  When triggered, the filter holds the present sample. The filter
    *           output will be the difference between the input sample and the held
    *           sample.
    *
    * ACCEL_HPF | Filter Mode | Cut-off Frequency
    * ----------+-------------+------------------
    * 0         | Reset       | None
    * 1         | On          | 5Hz
    * 2         | On          | 2.5Hz
    * 3         | On          | 1.25Hz
    * 4         | On          | 0.63Hz
    * 7         | Hold        | None
    * </pre>
    * 
    * @return Current high-pass filter configuration
    * @see MPU60X0_DHPF_RESET
    * @see MPU60X0_RA_ACCEL_CONFIG
    */	
    //DEBUG_PRINTLN("Setting DHPF bandwidth to 5Hz...");
    accelgyro.setDHPFMode(1);

	
    /** Get motion detection event acceleration threshold.
    * This register configures the detection threshold for Motion interrupt
    * generation. The unit of MOT_THR is 1LSB = 2mg. Motion is detected when the
    * absolute value of any of the accelerometer measurements exceeds this Motion
    * detection threshold. This condition increments the Motion detection duration
    * counter (Register 32). The Motion detection interrupt is triggered when the
    * Motion Detection counter reaches the time count specified in MOT_DUR
    * (Register 32).
    * 
    * The Motion interrupt will indicate the axis and polarity of detected motion
    * in MOT_DETECT_STATUS (Register 97).
    * 
    * For more details on the Motion detection interrupt, see Section 8.3 of the
    * MPU-6000/MPU-6050 Product Specification document as well as Registers 56 and
    * 58 of this document.
    *
    * @return Current motion detection acceleration threshold value (LSB = 2mg)
    * @see MPU60X0_RA_MOT_THR
    */	
    //Serial.println("Setting motion detection threshold to 2...");
    accelgyro.setMotionDetectionThreshold(2);


    /** Get zero motion detection event acceleration threshold.
    * This register configures the detection threshold for Zero Motion interrupt
    * generation. The unit of ZRMOT_THR is 1LSB = 2mg. Zero Motion is detected when
    * the absolute value of the accelerometer measurements for the 3 axes are each
    * less than the detection threshold. This condition increments the Zero Motion
    * duration counter (Register 34). The Zero Motion interrupt is triggered when
    * the Zero Motion duration counter reaches the time count specified in
    * ZRMOT_DUR (Register 34).
    * 
    * Unlike Free Fall or Motion detection, Zero Motion detection triggers an
    * interrupt both when Zero Motion is first detected and when Zero Motion is no
    * longer detected.
    * 
    * When a zero motion event is detected, a Zero Motion Status will be indicated
    * in the MOT_DETECT_STATUS register (Register 97). When a motion-to-zero-motion
    * condition is detected, the status bit is set to 1. When a zero-motion-to-
    * motion condition is detected, the status bit is set to 0.
    * 
    * For more details on the Zero Motion detection interrupt, see Section 8.4 of
    * the MPU-6000/MPU-6050 Product Specification document as well as Registers 56
    * and 58 of this document.
    * 
    * @return Current zero motion detection acceleration threshold value (LSB = 2mg)
    * @see MPU60X0_RA_ZRMOT_THR
    */	
    //Serial.println("Setting zero-motion detection threshold to 156...");
    accelgyro.setZeroMotionDetectionThreshold(2);


    /** Get motion detection event duration threshold.
    * This register configures the duration counter threshold for Motion interrupt
    * generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit
    * of 1LSB = 1ms. The Motion detection duration counter increments when the
    * absolute value of any of the accelerometer measurements exceeds the Motion
    * detection threshold (Register 31). The Motion detection interrupt is
    * triggered when the Motion detection counter reaches the time count specified
    * in this register.
    * 
    * For more details on the Motion detection interrupt, see Section 8.3 of the
    * MPU-6000/MPU-6050 Product Specification document.
    * 
    * @return Current motion detection duration threshold value (LSB = 1ms)
    * @see MPU60X0_RA_MOT_DUR
    */
    //Serial.println("Setting motion detection duration to 80/120...");
    accelgyro.setMotionDetectionDuration(0x78);


    /** Get zero motion detection event duration threshold.
    * This register configures the duration counter threshold for Zero Motion
    * interrupt generation. The duration counter ticks at 16 Hz, therefore
    * ZRMOT_DUR has a unit of 1 LSB = 64 ms. The Zero Motion duration counter
    * increments while the absolute value of the accelerometer measurements are
    * each less than the detection threshold (Register 33). The Zero Motion
    * interrupt is triggered when the Zero Motion duration counter reaches the time
    * count specified in this register.
    * 
    * For more details on the Zero Motion detection interrupt, see Section 8.4 of
    * the MPU-6000/MPU-6050 Product Specification document, as well as Registers 56
    * and 58 of this document.
    * 
    * @return Current zero motion detection duration threshold value (LSB = 64ms)
    * @see MPU60X0_RA_ZRMOT_DUR
    */	
    //Serial.println("Setting zero-motion detection duration to 0...");
    accelgyro.setZeroMotionDetectionDuration(0);	
	

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    // read raw accel/gyro measurements from device
    //Serial.println("Getting raw accwl/gyro measurements");
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    //Serial.println("Getting Motion indicators, count and threshold");
	
    /*XnegMD = accelgyro.getXNegMotionDetected();
    XposMD = accelgyro.getXPosMotionDetected();
    YnegMD = accelgyro.getYNegMotionDetected();
    YposMD = accelgyro.getYPosMotionDetected();
    ZnegMD = accelgyro.getZNegMotionDetected();
    ZposMD = accelgyro.getZPosMotionDetected();
	
    zero_detect = accelgyro.getIntMotionStatus();
    threshold = accelgyro.getZeroMotionDetectionThreshold();
	*/
    
    //Serial.println("Got to count");
    //count = accelgyro.getMotionDetectionCounterDecrement(); 
	
    /** Get current internal temperature.
    * @return Temperature reading in 16-bit 2's complement format
    * @see MPU60X0_RA_TEMP_OUT_H
    */
    //Serial.println("Getting Die Temperature");	
    temp=(accelgyro.getTemperature()/340.)+36.53;


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
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.print(gz); Serial.print(",");
    Serial.print(zero_detect); Serial.print(",");
	Serial.print(millis());Serial.println(",\r\n");
	
	//Serial.print(XnegMD); Serial.print(",");
	//Serial.println(XposMD);
	
    // display tab-separated accel/gyro x/y/z values
    /*
    Serial.print("a/g:\t");
    Serial.print(ax/16384.); Serial.print("\t");
    Serial.print(ay/16384.); Serial.print("\t");
    Serial.print(az/16384.); Serial.print("\t");
    Serial.print(gx/131.072); Serial.print("\t");
    Serial.print(gy/131.072); Serial.print("\t");
    Serial.println(gz/131.072);
	
    Serial.print("DieTemp:\t");Serial.println(temp);
	
    Serial.print("ZeroMotion(97):\t");	
    Serial.print(zero_detect); Serial.print("\t");
    Serial.print("Count: \t");Serial.print(count); Serial.print("\t");
    Serial.print(XnegMD); Serial.print("\t");
    Serial.print(XposMD); Serial.print("\t");
    Serial.print(YnegMD); Serial.print("\t");
    Serial.print(YposMD); Serial.print("\t");
    Serial.print(ZnegMD); Serial.print("\t");
    Serial.println(ZposMD);
    */	

    delay(80);
	
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}