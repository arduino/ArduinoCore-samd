/*
FreeIMU.cpp - A libre and easy to use orientation sensing library for Arduino
Copyright (C) 2011-2012 Fabio Varesano <fabio at varesano dot net>

Development of this code has been supported by the Department of Computer Science,
Universita' degli Studi di Torino, Italy within the Piemonte Project
http://www.piemonte.di.unito.it/


This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
-----------------------------------------------------------------------------------------------
Below changes were made by Michael J Smorto, above license still applies. 
All changes to this program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

10-27-2013   1. Modified incorporate temperature correction for gyro and accelerometer
			 2. Corrected __AVR__ to __AVR_ATmega128__
             3. Added call to get temperature from MPU-6050 in getRawValues
10-29-2013	Corrected temp coefficient array for temp
11-01-2013  Added new inverse sqrt code obtained from articles:
			http://pizer.wordpress.com/2008/10/12/fast-inverse-square-root/ and
			http://www.diydrones.com/forum/topics/madgwick-imu-ahrs-and-fast-inverse-square-root
11-05-13	1. Add code to reset MPU-6050 on start-up, implemented in the Arduino sketch as well as
			option to manually reset in processing sketch.
			2. Added LPF code line, currently set to MPU60X0_DLPF_BW_42
11-23-13	1. Changed twoKidef from 0.1 to 0.0f -
				12-12 => twoKiDef changed from 0f to 0.0025f based on article in DIYDrones
			2. Added bypass caps (0.1 and 0.01 ceramic caps on the power pins of the freeIMU
			2a. Added a 10 uF tantalum on the power bus
			3. Yaw drift stabilized very nicely
			4. Updated processing sketch to calculate heading and display based on the freescale 
			   application note and as implemented:
			   http://diydrones.com/profiles/blogs/tiltcompensated-heading
            5. update RESET function to zero out initial values
12-06-13	1. added LSM303D Polulo board instead of  calculating heading from FreeIMU
			2. added option to turn on/off temp correction, temp_corr_on
			3. added variable to set temperature break where calibration turns off, temp_break,
			note: temp_break in units directly from MPU-6050
01-02-14 to
  01-05-14  1. Recalibrated 6050 using Madgewick automated calibration rig - see paper
			2. Corrected temperature calibration coefficients to ensure that gyros are zeroed for whole
			   temp range when at rest - deleted necessary to run zerogyro routine.
			3. Acceleration coefficients adjusted so Ax = 0, Ay = 0 and Az = 1g when at rest
			4. Had to tune Ki to reduce drift - now set to 0.165
			5. Changed Kp to 1.0 - drift almost zero
01-08-14	1. Completed coding ZARA ZUPT for Zero velocity detemination - added code to the
			   processing sketch
01-09-14	1. Added a Reset Quaternion matrix only - option 2 in the sketch - to allow reset of matrix
			   after rapid movement.
11-02-2013  Todo list:
			2. add Yaw Drift correction - noticed that if the temp is above the temp cut-off
			   can get what I would consider unacceptable drift. Will also check temp calibration
			   above cut-off (high temp range)
			   
02-27-14	Fixed code issues with generic MPU6050 output for raw values.
03-02-14	1. Default to temp correction off.
			2. Updating code as temp correction only applicable to MPU-6050 or 9150 at this point.
			3. Clarified GenMPU6050 and DFROBOT defines
			4. Added code for BMP085 altimeter
03-04-14	Fixed issue with increased drift when magnetometer enabled by updating Fabio's code to the Madgwick
			code posted on his X-IO website.
03-09-14	1. With the magnetometer active Kp is better at 0.5 instead of 0.75. Will adjust as appropriate.
03-15-14	Tested code with magnetometer enabled for both Freeimu and DFROBOT. For the DFRobot which had the
			ADXL345 and ITG3200 Kp and Ki had to be adjusted to 0.15 and 0.000002 respectfully. This stabilized
			the drift but still more than the Freeimu. Code added to auto-adjust Kp and Ki depending on the board.
03-16-14	Added LPF to accelerometer and gyro
03-22-14	1. Added ArduPilot Gyro calibration code and modified AP_Math library calling the modifications
			AP_Math_freeimu to identify that changes were made
			2. Added initGyros option "g" to zero gyros from arduino code and processing sketches
			3. Changes resulted in allowing me to put back Kp and Ki to 0.75 and 0.1625 respectfully.
03-24-14	1. Added ArduPilot Filter library so I could implement a median filter on both the accelerometer
			and the gyroscope when using a ADXL345 and ITG3200. Using a 10 point median filter.
			2. Retuned Kp and Ki to 0.05 and 0.000002f, still have a bit of yaw drift. 
			3. Added a LPF filter per guidance at http://philstech.blogspot.com/2012/04/quadcopter-accelerometer-data-filtering.html.
03-25-14	Further tweaked filter factors and Kp/Ki for DFROBOT
				Median Filter = 7 point
				LPF factor = 0.6
03-31-14	Deleted all filtering and adjusted filters for DFROBOT and GENERAL MPU06050 (no mag) - had.
			little impact. Can not get the ADXL345/ITG3200 combo to stabilize as much as the MPU-6050.
04-03-14	Fixed cal curves - don't need gyro temp cal for ITG3200 since internal cal is used - seems to
			to work fine. Need to add back in filter for accelerometer when using ITG3200. 
04-09-14	1. Deleted Fabio's implementation of the AHRS filter and inserted the complete version from
			Seb Madgwick's X-io website. Seemed to fix the mag issue dramatically.
			2. Put median filter back in using a 5 point filter instead of 7
			3. Fixed zero gyro issue when using the ADXL345 chip
04-18-14
			Added 9150 library - 6050 library with getMotion9 function fully operational
			Fixed heading function so heading is aligned with the IMU axis alignment
			Put median filter on mag values
04-19-14
			Added support for GPS in Arduino and Processing Sketch using TinyGPS++ and AltSerialSoft libraries.
			TinyGPS++ : http://arduiniana.org/libraries/tinygpsplus/
			AltSoftwareSerial : https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
04-21-14	1. Added tilt compensated compass just using the HMC5883L or the 9150 on chip magnetomer
			2. Added GPS support using the TinyGPS++ and the AltSoftwareSerial libraries - modified processing
			GUI to support GPS.
04-23-14	Incorporated Butterworth filter from AP_Filter
04-28-14	Modified LS303D Pololu library for use with any magnetometer and accelerometer.
05-13-14
-------- Deleted 9150 library and added support for MPU-9150/9250 using Jeff Rowbergs AK8975 library for the magnetometer and maintained integrity of original 60X0 library
-------- Replaced Median Filter with Butterworh filters
-------- Added Filter library from ArduIMU (nice addition)
-------- Reduced size of serial sketch to fit on arduino uno for calibration
-------- Added azimuth code for heading when magnetometer and accelerometer available by extracting the code segment from the Pololu LSM303D library. Made a library called iCompass so it can be used elsewhere.
-------- In process of modifying code for use with Pololu AltIMU-10 v3.
---------------------------------------------------------------------------
05-17-14
-------- updated FreeIMU examples to work with current library
-------- Added support for the Pololu AltIMU-10 v3 IMU
--------------------------------------------------------------------------
05-24-14
-------- Corrected test for AltIMU-10 v3 and updated Kp/Ki
-------- Created a new Serial Example for when using a Arduino Due - AltSoftSerial does not work for Due.
-------- Minor correction to Processing sketch
--------------------------------------------------------------------------
06-20-14 
-------- Based on Kris Winer's work on the MPU-9250 and the AK8963 I was able to create 
-------- a basic AK8963 library modeled after that of the AK8975 library from Jeff Rowberg. 
-------- This was incorporated into FreeIMU library.
-------------------------------------------------------------------------
08-29-14
-------- incorporated changes proposed by Dan Barzilay which incorporates the Altitude Complimentary 
-------- filter and the Heading calcs that was in the Procressing sketch directly into the library.
-------- Arduino serial sketches were updated to output the heading and altimeter accordingly. As a 
-------- result of the changes the Processing sketch was simplified and the changes incorporated into 
-------- the FreeIMU_cube_Odo_Exp_v2 Gui in the experimental folder. 
------------------------------------------------------------------------
09-03-14
-------- Initial release:  Implemented the MadgwickAHRS (MARG) filter.  Option available to use either the Mahoney
-------- AHRS or the MARG from FreeIMU.h.  
------------------------------------------------------------------------
09-19-14
-------- Set up altitude to accept new sea level pressure from processing as option pxxxxxx, where xxxxxx
-------- is an the SL pressure in millibars * 100, so 1015.45 would read 101545
-------- Changed tilt compensation routine: deleting iCompass
------------------------------------------------------------------------
09-20-14
-------- Deleted iCompass and RunningAverage from FreeIMU. Compass averaging is done in the Processing 
-------- sketch using Yarmartino compass averaging method which is more address issue of discontinuity 
-------- at 0/360 degrees. 
-------- Tests on the Altimu10 v3 are much more stable. The processing code was adapted from 
-------- Christopher Baker's code at  https://github.com/SAIC-ATS/Algorithms.git. Will leave 
-------- runningaveage library in case anyone wishes to use it for other reasons.
-------- iCompass deleted as tilt compensation was not working properly. Tilt compensation is still 
-------- done from within FreeIMU.cpp using a different method.
-------- FreeIMU_serial.ino updated to reflect changes to implementation of altitude complimentary 
-------- filter from within the library as opposed to calling from the serial sketch.
------------------------------------------------------------------------
09-22-14
-------- Changed LSM303 library default sampling rate to 100hz for accel and magnetometer to adjust 
-------- for use with Teensy 3.1
-------- Updated (int) to (int16_t) in LPS library for getTemperatureRaw.
-------- FreeIMU examples updated accordingly based on compass averaging change.
------------------------------------------------------------------------
10-03-14 
-------- Updated FreeIMU.cpp to allow easier access to change accelerometer/gyro/magneter default
-------- range settings to match description in wiki page
------------------------------------------------------------------------
10-12-14 
-------- Updated library for support to APM 2.5 (Arduimu).  This change should also work on the 
-------- APM 2.6 board as well.  MS5611 SPI library added from Ardupilot library (saved alot of 
-------- time - thanks guys).  Put back iCompass module for aligned axis.  Had typo - now fixed 
-------- so put it back in.
--------
-------- This SPI code for the MS5611 is based on the example sketch by JP B, "APM 2.6 and 
-------- MPU6000 gyro - any custom firmware out there for this?" found at
-------- https://groups.google.com/forum/#!topic/diyrovers/thIohPCNyZA. Also see 
--------  http://forum.arduino.cc/index.php?PHPSESSID=6mia4mslt5dalc1fi4ad8rl2g1&topic=125623.0
------------------------------------------------------------------------
10-19-14
-------- Several updates/corrections made to library: Fixed APM 2.5 axis alignment, deleted a 
-------- 3g library that was not there, more tweaks to beta terms, uploaded missing TinyGPS++ 
-------- library and moved Baudrate setting to beginning of sketches.
--------
-------- Incorporated MPL3115A2 pressure sensor support by Mario Cannistra 
-------- (https://github.com/mariocannistra) as well as an example using a MPU-9250 
-------- using the MPL3115A2.
------------------------------------------------------------------------
10-24-14
-------- Implemented a third version of the Madgwick filter (Gradient Descent). This version 
-------- is directly from his original paper.  Beta and Zeta have been left at their default 
-------- values in the paper.
------------------------------------------------------------------------
10-25-14
-------- Fixed several bugs that would cause the new AHRS filter to crash on compile.
-------- Jitting is still there and its a lot of fun watching the algorithm work with the 
-------- default settings for the MPU9250.  Have fun tweaking the settings.
------------------------------------------------------------------------
11-04-14
-------- Minor update. Moved motion detect algorithm to FreeIMU library proper.  Created 
-------- FreeIMU_cube_Odo_Exp_v3.pde to take motion detect from library.
------------------------------------------------------------------------
11-05-14
-------- Thanks goes to duguyiqiu for pointing out that the Arduino Due Freq was not printing 
-------- correctly. As a result I found that the legacy code did not address the I2C bus speed 
-------- for the Due or the Mega 2560 to 400hz.  This is fixed with this update.
------------------------------------------------------------------------
11-08-14
-------- Fixed getQ_simple (q's not correctly identified), added a motion transition test and
-------- and called getQ_simple to get convergence on yaw faster (used heading)
------------------------------------------------------------------------
11-12-14
-------- Incorporated central definition of sensor order & signs for MARG calls in FreeIMU.hPa
-------- getQ_simple now works for all 9dof sensors whether sensors are aligned or not.
------------------------------------------------------------------------
11-13-14
-------- Setup a gyro_sensitivity variable defined at the same time you set up your gryo
-------- Made a minor change to the MPU60X0.cpp library for MPU6000 SPI
------------------------------------------------------------------------
11-19-14
-------- Combined running average filter and simple moving average filter into the AP_Filter library.
-------- Added a capability to the Arduino serial sketch to load a permanent GPS config file from a 
-------- attached I2C eeprom.  For my setup default baud is 57600.
------------------------------------------------------------------------
11-28-14
-------- Have incorporated the XBee-Arduino library by Andrew Rapp into an example sketch 
-------- (FreeIMU_serial_ARM_CPU_XBEE.ino).  Did make a modification to it so that it would send packets 
-------- without escapes.  So please use the one in the library folder otherwise it will not work.  
-------- Incorporated the Digi Xbee Java Library into the processing GUI (FreeIMU_cube_Odo_Exp_Digi2) in 
-------- the Experimental folder.  This code has only been tested with the Mega with XBee 900HPs.
-------- NOTE: To use the Xbee-Arduino with its examples skecthes you will need to edit the Xbee.h file 
-------- and set ATAP=2 (Escpaes sent), currently set to ATAP=1 (No Escapes).
------------------------------------------------------------------------
01-01-15
-------- Merged pull request by Mauro Mombelli who fixed AHRS.h for readability and merged ARHSupdateIMU 
-------- with AHRSupdate.  Updated FreeIMU.h accordingly.
------------------------------------------------------------------------
01-09-15
-------- Updated xbee-arduino library to add new function: setAPImode(MODE). Where MODE would be set equal 
-------- to 1 or 2 to correspond to the API mode that you selected for you Xbees.  Defaults to API Mode 2.
-------------------------------------------------------------------------
03-25-15 Incorporated DCM option based on the 9 Degree of Measurement Attitude and Heading Reference System 
-------- for Sparkfun 9DOF Razor IMU (SEN-10125 and SEN-10736) and SparkFun 9DOF Sensor Stick (SEN-10183, 
-------- SEN-10321 and SEN-10724), https://github.com/ptrbrtz/razor-9dof-ahrs.git.  The DCM, MATH and 
-------- Razor_AHRS.ino files were combined into a DCM library for use with the FreeIMU library.
-------- DCM implementation was based on the original code (http://code.google.com/p/sf9domahrs/) 
-------- by Doug Weibel and Jose Julio, based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio
-------- Additional functions added to get Euler angles directly from the DCM class as well as a function
-------- to convert the DCM matrix to quaternions which is used in the example sketch to interface with
-------- with existing Processing sketches.
-------- An example file has been created to show the DCM implementation with euler angle output.
-------- Use existing Serial sketches for use with processing. No other changes are needed.
--------------------------------------------------------------------------
07-18-15 Added two Kalman filter sketches using the output of the FreeIMU library. The first is the Kalman 
-------- implementation by Kristian Lauszus, TKJ Eectronics, http://www.tkjelectronics.com, using his modified sketch 
-------- that will compare the FreeIMU filter selected to that of the Kalman filter. The Arduino sketch is 
-------- FreeIMU_KalmanV1.ino. The sketch requires the StandardCplusplus lib - please use the version on my github 
-------- page as I had to make changes to it to work with the mega and other arduino boards per a issue description 
-------- from the library.
-------- 
-------- The second implementaion uses the Kalman class from Picopter - Matthew Watson authored on Apr 29 2013,
--------  https://github.com/big5824/Picopter.git. The sketch uses the ArduinoEigen Library and the stlport library 
-------- which I hava also uploaded so please refresh your download. The sketch for this implementation is 
-------- FreeIMU_EKF2. I also uploaded a processing sketch from Adafruit (Bunnyrotate.pde) for their AHRS 
-------- implementation demo. It uses the Saito object loader. I did make a few modifications including getting 
-------- rid of the bunny and use the Cassini model that comes with Saito library. This I also uploaded in the 
-------- Experimental directory.
--------------------------------------------------------------------------
07-24-15 Completed adding support for the LSM9DS0 iNemo IMU. Made use of the Sparkfun LSM9DS0 library. Thanks 
-------- to the guys at Sparkfun, I owe you guys at SFE a few beers. I did have to make a minor modification to the 
-------- library to make it callable from the FreeIMU class. 
---------------------------------------------------------------------------
08-24-15 Added support for the MS5637 Pressure Sensor using a slightly modified version of the Freetronics
-------- library.  All sketches were updated according.
---------------------------------------------------------------------------
08-25-15 Added 360 degree Euler angle rotation function. Also fixed an issue with val array out of bounds
------- preventing correct angles being returned when angle functions are returned.
---------------------------------------------------------------------------
11-28-15 Replace LPS331 library by LPS libray for compatibility with ALTIMU10 v3 (LPS331AP) and AltIMU-10 v4 (LPS25H).
------- https://github.com/pololu/lps-arduino
------- Add support for the Dadafruit 10-DOF IMU (L3GD20H / LSM303 / BMP180).
---------------------------------------------------------------------------
02-01-16 Adding support for CurieIMU - BMI160
----------------------------------------------------------------------------
03-29-16 Added support for the LSM9DS1 IMU with and without a co MS5611
----------------------------------------------------------------------------

*/

#include "Arduino.h"
#include <inttypes.h>

//#define DEBUG
#include "FreeIMU.h"

//#include "WireUtils.h"
//#include "DebugUtils.h"
#include <Filter.h>             // Filter library
#include <Butter.h>

#if(MARG == 0)
	#include "AHRS.h"
#elif(MARG == 1)
	#include "MadgwickAHRS.h"
#elif(MARG == 3)
	#include "MARGUpdateFilter.h"
#else
	#include "DCM.h"
	//dcm = DCM(); This has to be defined later otherwise compiler complains
#endif

//#include "vector_math.h"

//initialize temperature calibrations, mjs, 10/24/13 - udapted 4-7-14
//comment out ifs and leave float 0's only if you don't temp cal boards
#if defined(FREEIMU_v04)
	//calibration for MPU-6050 at default scales of freeIMU
	//need to change if you use different MPU-6050 or change scales.
	//Mag cal not used - TBD
	//MPU-6050 Cal
	float c3[9] = {           0.,            0., -1.618180e-09,            0.,          0.,          0.,     0., 0.,  0.};
	float c2[9] = {4.798083e-07 ,-7.104300e-08 , -1.899410e-05, -4.387634e-08, -1.779335e-08,  4.216745e-09, 0., 0., 0. };
	float c1[9] = {1.801522e-02 ,-5.200081e-03 , -1.462879e-01, -5.878346e-04,  1.172002e-03, -6.897733e-05, 0., 0., 0. };
	float c0[9] = {      -45.61 ,	     -45.24,       -305.58,  6.699801e+00,  8.341212e+00,	-2.171155e+01, 0., 0., 0. };
#elif defined(DFROBOT)
	// DFROBOT
	float c3[9] = {          0.,             0.,           0.,          0.,            0.,           0., 0., 0., 0.};
	float c2[9] = {0.007829365 , -0.0009776705 ,   0.01271437,  -0.01214285,   0.00615132,  0.002638248, 0., 0., 0.};
	float c1[9] = {-0.330832467, -0.0945206152 ,  -0.59609025,  -2.14580824,  -4.68951187, -2.832623092, 0., 0., 0. };
	float c0[9] = {   +5.648888, +9.784001     ,     2.775708,  60.71009177, 109.32876618, 85.639595449, 0., 0., 0. };
#else
	float c3[9] = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
	float c2[9] = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
	float c1[9] = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
	float c0[9] = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
#endif

enum Mscale {
  MFS_14BITS = 0, 	// 0.6 mG per LSB
  MFS_16BITS = 1    // 0.15 mG per LSB
};

//Setup accelerometer filter
//butter50hz2_0 mfilter_accx;
//butter50hz2_0 mfilter_accy;
//butter50hz2_0 mfilter_accz;
butter10hz0_3 mfilter_accx;
butter10hz0_3 mfilter_accy;
butter10hz0_3 mfilter_accz;

//#if HAS_MPU9150() || HAS_MPU9250()
//Set up Butterworth Filter for 9150 mag - more noisy than HMC5883L
//was butter50hz2_0, new values base on Mario Cannistrà suggestion
//he also used same filter on the gryo's which I am not using rigth now
butter100hz2_0  mfilter_mx;
butter100hz2_0  mfilter_my;
butter100hz2_0  mfilter_mz;
//#endif

//Setup Motion Detect Averages
MovingAvarageFilter accnorm_avg(5);
MovingAvarageFilter accnorm_test_avg(7);
MovingAvarageFilter accnorm_var(7);	
MovingAvarageFilter motion_detect_ma(7);

//Set-up constants for gyro calibration
uint8_t num_gyros = 1;
uint8_t INS_MAX_INSTANCES = 2;

#define M_PI 3.14159265359

FreeIMU::FreeIMU() {

  //pinMode(12,OUTPUT);
  
  #if HAS_ADXL345()
    acc = ADXL345();
  #elif HAS_BMA180()
    acc = BMA180();
  #endif
  
  #if HAS_HMC5883L()
    magn = HMC58X3();
    maghead = iCompass(MAG_DEC, WINDOW_SIZE, SAMPLE_SIZE);
  #endif
  
  #if HAS_LSM303()
    compass = LSM303();
    maghead = iCompass(MAG_DEC, WINDOW_SIZE, SAMPLE_SIZE);	
  #endif
  
  #if HAS_ITG3200()
    gyro = ITG3200();
  #elif HAS_L3D20()
    gyro = L3G();
  #elif HAS_MPU6050()
    accgyro = MPU60X0(); // I2C
  #elif HAS_MPU6000()
    accgyro = MPU60X0(); // SPI for Arduimu v3
  #elif HAS_MPU9150()
    accgyro = MPU60X0();
    mag = AK8975();
    maghead = iCompass(MAG_DEC, WINDOW_SIZE, SAMPLE_SIZE);
  #elif HAS_MPU9250()
    accgyro = MPU60X0();
    mag = AK8963();
    maghead = iCompass(MAG_DEC, WINDOW_SIZE, SAMPLE_SIZE);
  #elif HAS_LSM9DS0()
    //lsm = LSM9DS0(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
	#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
	#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
    lsm = LSM9DS0(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
    maghead = iCompass(MAG_DEC, WINDOW_SIZE, SAMPLE_SIZE);
  #elif HAS_LSM9DS1()
    //lsm = LSM9DS1(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
	lsm.settings.device.commInterface = IMU_MODE_I2C;
	lsm.settings.device.mAddress = LSM9DS1_XM;
	lsm.settings.device.agAddress = LSM9DS1_G;
    maghead = iCompass(MAG_DEC, WINDOW_SIZE, SAMPLE_SIZE);	
  #elif HAS_CURIE()
    accgyro = CurieImuClass(); 
  #elif HAS_TPS()
	amgData = PSMotionSense();
    maghead = iCompass(MAG_DEC, WINDOW_SIZE, SAMPLE_SIZE);	
  #endif
    
  #if HAS_PRESS()
    kPress.KalmanInit(0.0000005,0.01,1.0,0);
    #if HAS_MS5611()
	    #if HAS_APM25()
		    baro = AP_Baro_MS5611();
	    #else
		    baro = MS561101BA();
	    #endif
    #elif HAS_BMP085()
        baro085 = BMP085();
    #elif HAS_LPS()
        baroLPS = LPS();
    #elif HAS_MPL3115A2()
        baro3115 = MPL3115A2();
	#elif HAS_MS5637()
	    baro5637 = BaroSensorClass();
    #endif
  #endif
  
  // initialize quaternion
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
  q3old = 0.0f;
  exInt = 0.0;
  eyInt = 0.0;
  ezInt = 0.0;
  twoKp = twoKpDef;
  twoKi = twoKiDef;
  beta = betaDef;
  integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
  lastUpdate = 0;
  now = 0;

  //temp_break = -1000;	  //original temp_break = -4300;
  //senTemp_break = 32.;
  temp_corr_on = temp_corr_on_default;
  //nsamples = 75;
  //instability_fix = 1;
  
  //initialize gyro offsets
  gyro_off_x = 0.;
  gyro_off_y = 0.;  
  gyro_off_z = 0.;
  
  #ifndef CALIBRATION_H
    // initialize scale factors to neutral values
	  acc_scale_x = 1;
	  acc_scale_y = 1;
	  acc_scale_z = 1;
	  magn_scale_x = 1;
	  magn_scale_y = 1;
	  magn_scale_z = 1;
  #else
	  // get values from global variables of same name defined in calibration.h
	  acc_off_x = ::acc_off_x;
	  acc_off_y = ::acc_off_y;
	  acc_off_z = ::acc_off_z;
	  acc_scale_x = ::acc_scale_x;
	  acc_scale_y = ::acc_scale_y;
	  acc_scale_z = ::acc_scale_z;
	  magn_off_x = ::magn_off_x;
	  magn_off_y = ::magn_off_y;
 	  magn_off_z = ::magn_off_z;
	  magn_scale_x = ::magn_scale_x;
	  magn_scale_y = ::magn_scale_y;
	  magn_scale_z = ::magn_scale_z;
  #endif
}

void FreeIMU::init() {
  #if HAS_ITG3200()
    init(FIMU_ACC_ADDR, FIMU_ITG3200_DEF_ADDR, false);
  #elif HAS_ALTIMU10()
    init0(false);
  #elif HAS_APM25()
    //As per APM standard code, stop the barometer from holding the SPI bus
    pinMode(40, OUTPUT);
    digitalWrite(40, HIGH);
    init(53, false);
  #elif HAS_LSM9DS0()
    init0(true);
  #elif HAS_LSM9DS1()
    init0(true);
  #elif HAS_ADA_10_DOF()
    init0(false);
  #elif HAS_CURIE()
	init0(false);
  #elif HAS_TPS()
	init0(true);
  #else
    init(FIMU_ACCGYRO_ADDR, false);
  #endif
}

void FreeIMU::init(bool fastmode) {
  #if HAS_ITG3200()
    init(FIMU_ACC_ADDR, FIMU_ITG3200_DEF_ADDR, fastmode);
  #elif HAS_ALTIMU10() || HAS_LSM9DS0() || HAS_ADA_10_DOF() || HAS_LSM9DS1()
    init0(fastmode);
  #elif HAS_APM25()
    //As per APM standard code, stop the barometer from holding the SPI bus
    pinMode(40, OUTPUT);
    digitalWrite(40, HIGH);
    init(53, fastmode);
  #elif HAS_CURIE()
    init0(fastmode);
  #elif HAS_TPS()
	init0(fastmode);
  #else
    init(FIMU_ACCGYRO_ADDR, fastmode);
  #endif
}

void FreeIMU::RESET() {
	#if (HAS_MPU6050() || HAS_MPU6000() || HAS_MPU9150() || HAS_MPU9250)
		accgyro.reset();
	#endif
	
	delay(50);
	//reset matrix
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	exInt = 0.0;
	eyInt = 0.0;
	ezInt = 0.0;
	twoKp = twoKpDef;
	twoKi = twoKiDef;
	// beta = betaDef;
	integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
	//lastUpdate = 0;
	//now = 0;
}

void FreeIMU::RESET_Q() {
	//reset matrix
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	exInt = 0.0;
	eyInt = 0.0;
	ezInt = 0.0;
	twoKp = twoKpDef;
	twoKi = twoKiDef;
	// beta = betaDef;
	integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
	//lastUpdate = 0;
	//now = 0;
}


/**
 * Initialize the FreeIMU I2C bus, sensors and performs gyro offsets calibration
*/
#if HAS_ITG3200()
	void FreeIMU::init(int acc_addr, int gyro_addr, bool fastmode) {
#elif HAS_ALTIMU10() || HAS_LSM9DS0() || HAS_ADA_10_DOF() || HAS_CURIE() || HAS_LSM9DS1() \
	|| HAS_TPS()
	void FreeIMU::init0(bool fastmode) {
#else
	void FreeIMU::init(int accgyro_addr, bool fastmode) {
#endif
  delay(5);
  
  // disable internal pullups of the ATMEGA which Wire enable by default
  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
    // deactivate internal pull-ups for twi
    // as per note from atmega8 manual pg167
    cbi(PORTC, 4);
    cbi(PORTC, 5);
  #elif defined(__AVR_ATmega128__)
    // deactivate internal pull-ups for twi
    // as per note from atmega128 manual pg204
    cbi(PORTD, 0);
    cbi(PORTD, 1);
  #endif

  #if defined(__AVR_ATmega128) || defined( __AVR_ATmega2560__ ) // only valid on AVR, not on 32bit platforms (eg: Arduino 2, Teensy 3.0)
    if(fastmode) { // switch to 400KHz I2C - eheheh
      TWBR = ((F_CPU / 400000L) - 16) / 2; // see twi_init in Wire/utility/twi.c
    }
  #elif defined(__arm__)
    if(fastmode) {
      #if defined(CORE_TEENSY) && F_BUS == 48000000
        I2C0_F = 0x1A;  // Teensy 3.0 at 48 or 96 MHz
        I2C0_FLT = 2;
      #elif defined(CORE_TEENSY) && F_BUS == 24000000
        I2C0_F = 0x45;  // Teensy 3.0 at 24 MHz
        I2C0_FLT = 1;
      #endif
    }
  #elif defined(__SAM3X8E__) //Arduino Due
	 if(fastmode) { // switch to 400KHz I2C - eheheh
      TWBR = ((F_CPU / 400000L) - 16) / 2; 
    }
  #endif

 
  #if HAS_ADXL345()
    // init ADXL345
    acc.init(acc_addr);
    acc.set_bw(ADXL345_BW_100);
    acc.setRangeSetting(2);
  #elif HAS_BMA180()
    // init BMA180
    acc.setAddress(acc_addr);
    acc.SoftReset();
    acc.enableWrite();
    acc.SetFilter(acc.F10HZ);
    acc.setGSensitivty(acc.G2); 
    acc.SetSMPSkip();
    acc.SetISRMode();
    acc.disableWrite();
  #endif

  #if HAS_ITG3200()
    // init ITG3200
    gyro.init(gyro_addr);
    delay(1000);
    gyro.setFilterBW(BW020_SR1);
    delay(1000);
    // calibrate the ITG3200
    gyro.zeroCalibrate(128,5);
    gyro_sensitivity = 14.375f;
  #endif
  
   /* 
   * For Invensense MPUs:
   *
   * FS_SEL | Full Scale Range   | LSB Sensitivity
   * -------+--------------------+----------------
   * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
   * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
   * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
   * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
   */
  #if HAS_MPU6050()
    accgyro = MPU60X0(false, accgyro_addr);
    accgyro.initialize();
    accgyro.setDLPFMode(MPU60X0_DLPF_BW_20); 
    accgyro.setI2CMasterModeEnabled(0);
    accgyro.setI2CBypassEnabled(1);
    accgyro.setRate(0x13);			//Sets sample rate to 8000/1+7 = 1000Hz
    accgyro.setFullScaleGyroRange(MPU60X0_GYRO_FS_2000);
    accgyro.setFullScaleAccelRange(MPU60X0_ACCEL_FS_2);
    gyro_sensitivity = 16.4f;
    delay(5);
    delay(30);
  #elif HAS_MPU9150()
    //initialize accelerometer and gyroscope
    accgyro = MPU60X0(false, accgyro_addr);
    #if HAS_MPU9250()
	    accgyro.initialize9250();
	    accgyro.setDLPFMode(MPU60X0_DLPF_BW_98);
	#else
	  accgyro.initialize();
	  accgyro.setDLPFMode(MPU60X0_DLPF_BW_20);
	#endif
	
    accgyro.setI2CMasterModeEnabled(0);
    accgyro.setI2CBypassEnabled(1);
    accgyro.setFullScaleGyroRange(MPU60X0_GYRO_FS_2000);
	accgyro.setFullScaleAccelRange(MPU60X0_ACCEL_FS_2);
	gyro_sensitivity = 16.4f;
	delay(100);
	//initialize magnetometer
	mag = AK8975(false, AK8975_DEFAULT_ADDRESS);
	mag.initialize();
  #elif HAS_MPU9250()
	//initialize accelerometer and gyroscope
	accgyro = MPU60X0(false, accgyro_addr);
	accgyro.initialize9250();
	accgyro.setDLPFMode(MPU60X0_DLPF_BW_20); 
	accgyro.setI2CMasterModeEnabled(0);
	accgyro.setI2CBypassEnabled(1);
	accgyro.setFullScaleGyroRange(MPU60X0_GYRO_FS_2000);
	accgyro.setFullScaleAccelRange(MPU60X0_ACCEL_FS_2);
	gyro_sensitivity = 16.4f;
	delay(100);
	//initialize magnetometer
	mag = AK8963(false, AK8963_DEFAULT_ADDRESS);
	mag.initialize();  
	mag.setModeRes(AK8963_MODE_CONT2, MFS_16BITS);
  #elif HAS_MPU6000()
	accgyro = MPU60X0(true, accgyro_addr);
	accgyro.initialize();
	accgyro.setRate(19);
	accgyro.setFullScaleGyroRange(MPU60X0_GYRO_FS_2000);
	accgyro.setFullScaleAccelRange(MPU60X0_ACCEL_FS_2);
	accgyro.setDLPFMode(MPU60X0_DLPF_BW_20); 
	gyro_sensitivity = 16.4f;
	delay(5);
  #endif 
  
  #if HAS_LSM9DS0()
    // Use the begin() function to initialize the LSM9DS0 library.
	// You can either call it with no parameters (the easy way):
	uint16_t status = lsm.begin(lsm.G_SCALE_2000DPS, lsm.A_SCALE_2G );
	// Or call it with declarations for sensor scales and data rates:  
	//uint16_t status = lsm.begin(lsm.G_SCALE_2000DPS, 
	//                            lsm.A_SCALE_6G, lsm.M_SCALE_2GS);
	//Angular rate FS = ±245 dps, gyro_sensitivity = 8.75
	//Angular rate FS = ±500 dps, gyro_sensitivity =  17.50
	//Angular rate FS = ±2000 dps, gyro_sensitivity =  70
	gyro_sensitivity = 70.0f;
  #endif
  
  #if HAS_LSM9DS1()
    // [commInterface] determines whether we'll use I2C or SPI
	// to communicate with the LSM9DS1.
	// Use either IMU_MODE_I2C or IMU_MODE_SPI
	lsm.settings.device.commInterface = IMU_MODE_I2C;
	// [mAddress] sets the I2C address or SPI CS pin of the
	// LSM9DS1's magnetometer.
	lsm.settings.device.mAddress = 0x1E; // Use I2C addres 0x1E
	// [agAddress] sets the I2C address or SPI CS pin of the
	// LSM9DS1's accelerometer/gyroscope.
	lsm.settings.device.agAddress = 0x6B; // I2C address 0x6B
  
	// Setup Gyro
	// [enabled] turns the gyro on or off.
	lsm.settings.gyro.enabled = true;  // Enable the gyro

    // [lowPowerEnable] turns low-power mode on or off.
    lsm.settings.gyro.lowPowerEnable = false; // LP mode off
	
	// [scale] sets the full-scale range of the gyroscope.
	// scale can be set to either 245, 500, or 2000
	lsm.settings.gyro.scale = 245; // Set scale to +/-245dps
	
	// [sampleRate] sets the output data rate (ODR) of the gyro
	// sampleRate can be set between 1-6
	// 1 = 14.9    4 = 238
	// 2 = 59.5    5 = 476
	// 3 = 119     6 = 952
	lsm.settings.gyro.sampleRate = 4; // 59.5Hz ODR
	
	// [bandwidth] can set the cutoff frequency of the gyro.
	// Allowed values: 0-3. Actual value of cutoff frequency
	// depends on the sample rate. (Datasheet section 7.12)
	lsm.settings.gyro.bandwidth = 1;
	
	// [HPFEnable] enables or disables the high-pass filter
	lsm.settings.gyro.HPFEnable = false; // HPF disabled
	
	// [HPFCutoff] sets the HPF cutoff frequency (if enabled)
	// Allowable values are 0-9. Value depends on ODR.
	// (Datasheet section 7.14)
	lsm.settings.gyro.HPFCutoff = 1; // HPF cutoff = 4Hz
	
	//Angular rate FS = ±245 dps, gyro_sensitivity = 8.75
	//Angular rate FS = ±500 dps, gyro_sensitivity =  17.50
	//Angular rate FS = ±2000 dps, gyro_sensitivity =  70
	gyro_sensitivity = 8.75f;
	
	// Setup Accelerometer -----------------------------------
	// [scale] sets the full-scale range of the accelerometer.
	// accel scale can be 2, 4, 8, or 16
	lsm.settings.accel.scale = 2; // Set accel scale to +/-8g.
	
	// [sampleRate] sets the output data rate (ODR) of the
	// accelerometer. ONLY APPLICABLE WHEN THE GYROSCOPE IS
	// DISABLED! Otherwise accel sample rate = gyro sample rate.
	// accel sample rate can be 1-6
	// 1 = 10 Hz    4 = 238 Hz
	// 2 = 50 Hz    5 = 476 Hz
	// 3 = 119 Hz   6 = 952 Hz
	lsm.settings.accel.sampleRate = 4; // Set accel to 10Hz.
	
	// [bandwidth] sets the anti-aliasing filter bandwidth.
	// Accel cutoff freqeuncy can be any value between -1 - 3. 
	// -1 = bandwidth determined by sample rate
	// 0 = 408 Hz   2 = 105 Hz
	// 1 = 211 Hz   3 = 50 Hz
	lsm.settings.accel.bandwidth = 3; // BW = 408Hz
	
	// [highResEnable] enables or disables high resolution 
	// mode for the acclerometer.
	lsm.settings.accel.highResEnable = false; // Disable HR
	
	// [highResBandwidth] sets the LP cutoff frequency of
	// the accelerometer if it's in high-res mode.
	// can be any value between 0-3
	// LP cutoff is set to a factor of sample rate
	// 0 = ODR/50    2 = ODR/9
	// 1 = ODR/100   3 = ODR/400
	lsm.settings.accel.highResBandwidth = 0;  
	
	// Setup Magnetometer ---------------------------------------
	// [enabled] turns the magnetometer on or off.
	lsm.settings.mag.enabled = true; // Enable magnetometer
	
	// [scale] sets the full-scale range of the magnetometer
	// mag scale can be 4, 8, 12, or 16
	lsm.settings.mag.scale = 4; // Set mag scale to +/-12 Gs
	
	// [sampleRate] sets the output data rate (ODR) of the
	// magnetometer.
	// mag data rate can be 0-7:
	// 0 = 0.625 Hz  4 = 10 Hz
	// 1 = 1.25 Hz   5 = 20 Hz
	// 2 = 2.5 Hz    6 = 40 Hz
	// 3 = 5 Hz      7 = 80 Hz
	lsm.settings.mag.sampleRate = 4; // Set OD rate to 20Hz
	
	// [tempCompensationEnable] enables or disables 
	// temperature compensation of the magnetometer.
	lsm.settings.mag.tempCompensationEnable = false;
	
	// [XYPerformance] sets the x and y-axis performance of the
	// magnetometer to either:
	// 0 = Low power mode      2 = high performance
	// 1 = medium performance  3 = ultra-high performance
	lsm.settings.mag.XYPerformance = 2; // Ultra-high perform.
	
	// [ZPerformance] does the same thing, but only for the z
	lsm.settings.mag.ZPerformance = 2; // Ultra-high perform.
	
	// [lowPowerEnable] enables or disables low power mode in
	// the magnetometer.
	lsm.settings.mag.lowPowerEnable = false;
	
	// [operatingMode] sets the operating mode of the
	// magnetometer. operatingMode can be 0-2:
	// 0 = continuous conversion
	// 1 = single-conversion
	// 2 = power down
	lsm.settings.mag.operatingMode = 0; // Continuous mode
	
    // Use the begin() function to initialize the LSM9DS1 library.
	// You can either call it with no parameters (the easy way):
	uint16_t status = lsm.begin();
  #endif
  
  #if HAS_CURIE()
	accgyro.initialize();
	accgyro.setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);
	/** Set accelerometer output data rate.
	* The acc_odr parameter allows setting the output data rate of the accelerometer
	* as described in the table below.
	*
	* <pre>
	*  5 =  25/2Hz
	*  6 =    25Hz
	*  7 =    50Hz
	*  8 =   100Hz
	*  9 =   200Hz
	* 10 =   400Hz
	* 11 =   800Hz
	* 12 =  1600Hz
	* 13 =  3200Hz
	* </pre>
	*
	*/
	accgyro.setAccelRate(BMI160_ACCEL_RATE_100HZ);

	/** Get accelerometer digital low-pass filter mode.
	* The acc_bwp parameter sets the accelerometer digital low pass filter configuration.
	*
	* When the filter mode is set to Normal (@see BMI160_DLPF_MODE_NORM), the filter
	* bandwidth for each respective accelerometer output data rates is shown in the table below:
	*
	* <pre>
	* ODR     | 3dB cut-off
	* --------+--------------
	*  12.5Hz |  5.06Hz
	*    25Hz | 10.12Hz
	*    50Hz | 20.25Hz
	*   100Hz | 40.5Hz
	*   200Hz | 80Hz
	*   400Hz | 162Hz (155Hz for Z axis)
	*   800Hz | 324Hz (262Hz for Z axis)
	*  1600Hz | 684Hz (353Hz for Z axis)
	* </pre>
	*
	* When the filter mode is set to OSR2 (@see BMI160_DLPF_MODE_OSR2), the filter
	* bandwidths above are approximately halved.
	*
	* When the filter mode is set to OSR4 (@see BMI160_DLPF_MODE_OSR4), the filter
	* bandwidths above are approximately 4 times smaller.
	*/
	//accgyro.setAccelDLPFMode(BMI160_DLPF_MODE_OSR2);

	accgyro.setFullScaleGyroRange(BMI160_GYRO_RANGE_2000);
	/** Get gyroscope output data rate.
	* The gyr_odr parameter allows setting the output data rate of the gyroscope
	* as described in the table below.
	*
	* <pre>
	*  6 =   25Hz
	*  7 =   50Hz
	*  8 =  100Hz
	*  9 =  200Hz
	* 10 =  400Hz
	* 11 =  800Hz
	* 12 = 1600Hz
	* 13 = 3200Hz
	* </pre>
	*/
	accgyro.setGyroRate(BMI160_GYRO_RATE_100HZ);
	  
	/** Get gyroscope digital low-pass filter mode.
	* The gyro_bwp parameter sets the gyroscope digital low pass filter configuration.
	*
	* When the filter mode is set to Normal (@see BMI160_DLPF_MODE_NORM), the filter
	* bandwidth for each respective gyroscope output data rates is shown in the table below:
	*
	* <pre>
	* ODR     | 3dB cut-off
	* --------+------------
	*    25Hz | 10.7Hz
	*    50Hz | 20.8Hz
	*   100Hz | 39.9Hz
	*   200Hz | 74.6Hz
	*   400Hz | 136.6Hz
	*   800Hz | 254.6Hz
	*  1600Hz | 523.9Hz
	*  3200Hz | 890Hz
	* </pre>
	*
	* When the filter mode is set to OSR2 (@see BMI160_DLPF_MODE_OSR2), the filter
	* bandwidths above are approximately halved.
	*
	* When the filter mode is set to OSR4 (@see BMI160_DLPF_MODE_OSR4), the filter
	* bandwidths above are approximately 4 times smaller.
	*/
	//accgyro.setGyroDLPFMode(BMI160_DLPF_MODE_OSR2);

	  
	/*
	* | Full Scale Range   | LSB Sensitivity
	* +--------------------+----------------
	* | +/- 125 degrees/s  | 262.4 LSB/deg/s
	* | +/- 250 degrees/s  | 131.2 LSB/deg/s
	* | +/- 500 degrees/s  | 65.6 LSB/deg/s
	* | +/- 1000 degrees/s | 32.8 LSB/deg/s
	* | +/- 2000 degrees/s | 16.4 LSB/deg/s // use 20.5
	*/
	gyro_sensitivity = 20.5f;
	delay(100);
  #endif
  
  #if HAS_TPS()
	amgData.begin();
	/*
	* | Full Scale Range   | LSB Sensitivity
	* +--------------------+----------------
	* | +/- 250 degrees/s  | 7.8125 LSB/deg/s
	* | +/- 500 degrees/s  | 15.625 LSB/deg/s
	* | +/- 1000 degrees/s | 31.25 LSB/deg/s
	* | +/- 2000 degrees/s | 62.5 LSB/deg/s // use 20.5
	*/
	gyro_sensitivity = 62.5f;
	delay(100);
  #endif
  
  #if HAS_HMC5883L()
	  // init HMC5843
	  magn.init(false); // Don't set mode yet, we'll do that later on.
	  // Calibrate HMC using self test, not recommended to change the gain after calibration.
	  magn.calibrate(1); // Use gain 1=default, valid 0-7, 7 not recommended.
	  // Single mode conversion was used in calibration, now set continuous mode
	  magn.setMode(0);
	  delay(10);
	  magn.setDOR(B110);
  #endif
  
  
  #if HAS_MS5611()
	#if HAS_APM25()
		pinMode(63, OUTPUT);
		digitalWrite(63, HIGH);
		SPI.begin();
		SPI.setClockDivider(SPI_CLOCK_DIV16); // 32 = 500khz for debugging, increase later
		baro.init();		
	#else
		baro.init(FIMU_BARO_ADDR);
	#endif
  #endif

  #if HAS_MPL3115A2()
    baro3115.begin();
	baro3115.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
	// Set the # of samples from 1 to 128. See datasheet.
	// Integer values between 0 < n < 7 give oversample ratios 2^n and 
	// sampling intervals of 0=6 ms , 1=10, 2=18, 3=34, 4=66, 5=130, 6=258, and 7=512 
	baro3115.setOversampleRate(4); // Set Oversample to the recommended 128 --> 512ms
	baro3115.enableEventFlags(); // Enable all three pressure and temp event flags 
  #endif  
   
  #if HAS_BMP085()
    // 19.8 meters for my location, true = using meter units
    // this initialization is useful if current altitude is known,
    // pressure will be calculated based on TruePressure and known altitude.	
    //baro085.init(3, 1981.6469, true);  
    //Changed Init to default pressure
    baro085.init();
  #endif
  
  #if HAS_MS5637()
    baro5637.begin();
  #endif
  
  //ALTIMU or Adafruit 10-DOF SENSOR INIT
  #if HAS_L3D20()
    gyro.init();
    //gyro.enableDefault();
    //					Sensitivity
    //+/-245:        0x00	8.75	
    //+/-500:        0x10	17.5
    //+/- 2000:      0x20	70
    gyro.writeReg(0x23, 0x20);
    gyro_sensitivity = 70.0f;
  #endif
  
  #if HAS_LSM303()
    compass.init();
    compass.enableDefault();
    deviceType = compass.getDeviceType();

	if((int16_t) deviceType == 3) {	// LSM303D
		// Accelerometer
		//0x00:+/-2g,  0x08:+/- 4g,  0x10: +/- 6g, 0x18: +/- 8g,  0x20: +/- 16g
		compass.writeReg(LSM303::CTRL2, 0x00);

		// Magnetometer
		// MFS = 01 (+/- 4 gauss full scale),  MFS = 00(+/- 2 gauss full scale)
		// MFS = 10 (+/- 8 gauss full scale),  MFS = 11(+/- 12 gauss full scale)
		compass.writeReg(LSM303::CTRL6, 0x00);
	}
	else if((int16_t) deviceType == 2) {  // LSM303DLHC
		// Accelerometer
		// FS = 00  (+/- 2 g full scale); HR = 1 (high resolution enable)
		//                                  FS HR
		//    equates to 0b00 00  1    00 0
		// FS = 0x01  (+/- 4 g),  0x10: +/- 8 g, 0x11: +/- 16 g 
		//  or: 0x08, 0x18, 0x28, 0x38 respectfully
		compass.writeAccReg(LSM303::CTRL_REG4_A, 0x08);
	
		// Magnetometer Settings
		// ±1.3ga	(0x20)
		// ±1.9ga	(0x40)
		// ±2.5ga	(0x60)
		// ±4.0ga	(0x80)	
		// ±4.7ga	(0xA0)	
		// ±5.6ga	(0xC0)
		// ±8.1ga	(0xE0)	
		compass.writeMagReg(LSM303::CRB_REG_M, 0x20);
	}
	else if((int16_t) deviceType == 0 || (int16_t) deviceType == 1) {  // LSM303DLH or LSM303DLM
		// Accelerometer
		// 0x00 = 0b00000000
		// FS = 00 (+/- 2 g full scale), 01 (+/- 4g), 11} (+/- 8g)
		// 0x00, 0x01, 0x03 respectfully
		compass.writeAccReg(LSM303::CTRL_REG4_A, 0x00);

		// Magnetometer Settings
		// ±1.3ga	(0x20)
		// ±1.9ga	(0x40)
		// ±2.5ga	(0x60)
		// ±4.0ga	(0x80)	
		// ±4.7ga	(0xA0)	
		// ±5.6ga	(0xC0)
		// ±8.1ga	(0xE0)	
		compass.writeMagReg(LSM303::CRB_REG_M, 0x20);
	}	

  #endif
  
  #if HAS_LPS()
    baroLPS.init();
    baroLPS.enableDefault();
  #endif
  
  // zero gyro
  //zeroGyro();
  //if(temp_corr_on == 0) {
  //digitalWrite(12,HIGH);
  
  initGyros(); //}

  //digitalWrite(12,LOW);
	
  #ifndef CALIBRATION_H
	  // load calibration from eeprom
	  calLoad();
  #endif

  
  RESET_Q();

  float values[11];

  //DCM filter implementation set here so we can intit with calibrated values.  All initializations have to be done first.
  #if(MARG == 4)
    dcm = DCM();
    getValues( values);
    values[9] = maghead.iheading(1, 0, 0, values[0], values[1], values[2], values[6], values[7], values[8]);
    dcm.setSensorVals(values);
    dcm.DCM_init(Kp_ROLLPITCH, Ki_ROLLPITCH, Kp_YAW, Ki_YAW);
  #endif

}

#ifndef CALIBRATION_H

static uint8_t location; // assuming ordered reads

void eeprom_read_var(uint8_t size, byte * var) {
  for(uint8_t i = 0; i<size; i++) {
    var[i] = EEPROM.read(location + i);
  }
  location += size;
}

void FreeIMU::calLoad() {
  if(EEPROM.read(FREEIMU_EEPROM_BASE) == FREEIMU_EEPROM_SIGNATURE) { // check if signature is ok so we have good data
    location = FREEIMU_EEPROM_BASE + 1; // reset location
    
    eeprom_read_var(sizeof(acc_off_x), (byte *) &acc_off_x);
    eeprom_read_var(sizeof(acc_off_y), (byte *) &acc_off_y);
    eeprom_read_var(sizeof(acc_off_z), (byte *) &acc_off_z);
    
    eeprom_read_var(sizeof(magn_off_x), (byte *) &magn_off_x);
    eeprom_read_var(sizeof(magn_off_y), (byte *) &magn_off_y);
    eeprom_read_var(sizeof(magn_off_z), (byte *) &magn_off_z);
    
    eeprom_read_var(sizeof(acc_scale_x), (byte *) &acc_scale_x);
    eeprom_read_var(sizeof(acc_scale_y), (byte *) &acc_scale_y);
    eeprom_read_var(sizeof(acc_scale_z), (byte *) &acc_scale_z);
    
    eeprom_read_var(sizeof(magn_scale_x), (byte *) &magn_scale_x);
    eeprom_read_var(sizeof(magn_scale_y), (byte *) &magn_scale_y);
    eeprom_read_var(sizeof(magn_scale_z), (byte *) &magn_scale_z);
  }
  else { // neutral values
    acc_off_x = 0;
    acc_off_y = 0;
    acc_off_z = 0;
    acc_scale_x = 1;
    acc_scale_y = 1;
    acc_scale_z = 1;

    magn_off_x = 0;
    magn_off_y = 0;
    magn_off_z = 0;
    magn_scale_x = 1;
    magn_scale_y = 1;
    magn_scale_z = 1;
  }
}
#endif


/**
 * Populates raw_values with the raw_values from the sensors
*/
void FreeIMU::getRawValues(int * raw_values) {
	//Set raw values for Magnetometer, Press, Temp to 0 in case you are only using
	//an accelerometer and gyro
	//raw_values[9] will be set to MPU-6050 temp, see zeroGyro to change raw_values dimension
	//raw_values[6] = 0;
	//raw_values[7] = 0;
	//raw_values[8] = 0;
	//raw_values[9] = 0;
	
  #if HAS_ITG3200()
    acc.readAccel(&raw_values[0], &raw_values[1], &raw_values[2]);
    gyro.readGyroRaw(&raw_values[3], &raw_values[4], &raw_values[5]);
	gyro.readTemp(&senTemp);
	raw_values[9] = senTemp*100;
  #elif HAS_MPU6050() || HAS_MPU6000() || HAS_MPU9150() || HAS_MPU9250() || HAS_CURIE()
    #ifdef __AVR__
	  accgyro.getMotion6(&raw_values[0], &raw_values[1], &raw_values[2], &raw_values[3], &raw_values[4], &raw_values[5]);  	  
 	  #if HAS_MPU9150() || HAS_MPU9250()
		  mag.getHeading(&raw_values[6], &raw_values[7], &raw_values[8]);			
		  delay(10);
		#endif
	  rt = accgyro.getTemperature();	  
	  raw_values[9] = rt;
	 #else
      int16_t ax, ay, az, gx, gy, gz, mx, my, mz, rt;
      accgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	  #if HAS_MPU9150() || HAS_MPU9250() 
		  mag.getHeading(&mx, &my, &mz);
		  raw_values[6] = mx;
		  raw_values[7] = my;
		  raw_values[8] = mz;				
		  delay(10);
	  #endif
      raw_values[0] = ax;
      raw_values[1] = ay;
      raw_values[2] = az;
      raw_values[3] = gx;
      raw_values[4] = gy;
      raw_values[5] = gz;
      rt = accgyro.getTemperature();	  
      raw_values[9] = rt; 
    #endif
  #endif 

  #if HAS_HMC5883L()
    magn.getValues(&raw_values[6], &raw_values[7], &raw_values[8]);
  #endif  
  
 #if HAS_L3D20()
	gyro.read();
    raw_values[3] = gyro.g.x;
    raw_values[4] = gyro.g.y;
    raw_values[5] = gyro.g.z;	
  #endif

  #if HAS_LSM303()
	compass.read();
    raw_values[0] = compass.a.x;
    raw_values[1] = compass.a.y;
    raw_values[2] = compass.a.z;
    raw_values[6] = compass.m.x;
    raw_values[7] = compass.m.y;
    raw_values[8] = compass.m.z;
  #endif	

  #if HAS_LSM9DS1()
	lsm.readAccel();
  
	if(lsm.gyroAvailable()){ 
		lsm.readGyro(); 
		raw_values[3] = lsm.gx;
		raw_values[4] = lsm.gy;
		raw_values[5] = lsm.gz;
		}
	if(lsm.accelAvailable()){ 	
		raw_values[0] = lsm.ax;
		raw_values[1] = lsm.ay;
		raw_values[2] = lsm.az;
	}
	if(lsm.accelAvailable()){ 		
		lsm.readMag();
		raw_values[6] = lsm.mx;
		raw_values[7] = lsm.my;
		raw_values[8] = lsm.mz;
	}
	if(lsm.tempAvailable()){ 	
		lsm.readTemp();
		DTemp = lsm.temperature;
		raw_values[9] = DTemp;
	}
  #endif
	
  #if HAS_LSM9DS0() 

	lsm.readGyro(); 
	raw_values[3] = lsm.gx;
	raw_values[4] = lsm.gy;
	raw_values[5] = lsm.gz;
	
	lsm.readAccel();
	raw_values[0] = lsm.ax;
	raw_values[1] = lsm.ay;
	raw_values[2] = lsm.az;
	
	lsm.readMag();
	raw_values[6] = lsm.mx;
	raw_values[7] = lsm.my;
	raw_values[8] = lsm.mz;
	
	lsm.readTemp();
	DTemp = lsm.temperature;
	raw_values[9] = DTemp;
  #endif
  
  #if HAS_TPS()
	int iax, iay, iaz, igx, igy, igz, imx, imy, imz;
	amgData.readMotionSensor(iax, iay, iaz, igx, igy, igz, imx, imy, imz);
    raw_values[0] = iax;
    raw_values[1] = iay;
    raw_values[2] = iaz;
	raw_values[3] = igx;
	raw_values[4] = igy;
	raw_values[5] = igz;
    raw_values[6] = imx;
    raw_values[7] = imy;
    raw_values[8] = imz;
  #endif
		
}


/**
 * Populates values with calibrated readings from the sensors
*/
void FreeIMU::getValues(float * values) { 

  float acgyro_corr[9] = {0.,0.,0.,0.,0.,0.,0.,0.,0.};
  float values_cal[9] = {0.,0.,0.,0.,0.,0.,0.,0.,0.};
  uint8_t i;

  #if HAS_ITG3200()  //assumes adxl3345
    int accval[3];
    acc.readAccel(&accval[0], &accval[1], &accval[2]);
	accval[0] = mfilter_accx.filter((float) accval[0]);
    accval[1] = mfilter_accy.filter((float) accval[1]);
    accval[2] = mfilter_accz.filter((float) accval[2]);
	
    gyro.readGyro(&values_cal[3]);	
	  gyro.readTemp(&senTemp);
    	if(temp_corr_on == 1) {
    		if(senTemp < senTemp_break) {
    			for(i = 0; i < 9; i++) { 
    				acgyro_corr[i] = c3[i]*(senTemp*senTemp*senTemp) + c2[i]*(senTemp*senTemp) + c1[i]*senTemp + c0[i];
			}		
		} 
	} else {
		//for(i = 0; i < 9; i++) { 
		//	acgyro_corr[i] = 0.0f;
		//}
		memset(acgyro_corr, 0.0f, sizeof(acgyro_corr));
	}

	values_cal[0] = (float) accval[0] - acgyro_corr[0];
	values_cal[1] = (float) accval[1] - acgyro_corr[1];
	values_cal[2] = (float) accval[2] - acgyro_corr[2];
		
	values_cal[3] = (values_cal[3] - gyro_off_x)/gyro_sensitivity;
	values_cal[4] = (values_cal[4] - gyro_off_y)/gyro_sensitivity;
	values_cal[5] = (values_cal[5] - gyro_off_z)/gyro_sensitivity;

  #elif HAS_ALTIMU10() || HAS_ADA_10_DOF()
	gyro.read();
	compass.read();
	values_cal[0] = (float) compass.a.x;
	values_cal[1] = (float) compass.a.y;
	values_cal[2] = (float) compass.a.z;
	//values[0] = mfilter_accx.filter((float) compass.a.x);
	//values[1] = mfilter_accy.filter((float) compass.a.y);
	//values[2] = mfilter_accz.filter((float) compass.a.z);	
	values_cal[3] = (float) gyro.g.x;
	values_cal[4] = (float) gyro.g.y;
	values_cal[5] = (float) gyro.g.z;
	values_cal[6] = (float) compass.m.x;
	values_cal[7] = (float) compass.m.y; 
	values_cal[8] = (float) compass.m.z;
	
	values_cal[3] = (values_cal[3] - gyro_off_x) / gyro_sensitivity;  //Sensitivity set at 70 for +/-2000 deg/sec, L3GD20H
	values_cal[4] = (values_cal[4] - gyro_off_y) / gyro_sensitivity;
	values_cal[5] = (values_cal[5] - gyro_off_z) / gyro_sensitivity;

  #elif HAS_LSM9DS0() || HAS_LSM9DS1()
	lsm.readAccel();
	lsm.readGyro();
	lsm.readMag();
    values_cal[0] = (float) lsm.ax;
    values_cal[1] = (float) lsm.ay;
    values_cal[2] = (float) lsm.az;
    values_cal[6] = (float) lsm.mx;
    values_cal[7] = (float) lsm.my;
    values_cal[8] = (float) lsm.mz;
    values_cal[3] = (float) lsm.gx;
    values_cal[4] = (float) lsm.gy;
    values_cal[5] = (float) lsm.gz;	 
	
  	values_cal[3] = (values_cal[3] - gyro_off_x) / gyro_sensitivity;  //Sensitivity set at 70 for +/-2000 deg/sec, L3GD20H
  	values_cal[4] = (values_cal[4] - gyro_off_y) / gyro_sensitivity;
  	values_cal[5] = (values_cal[5] - gyro_off_z) / gyro_sensitivity;
  
  	lsm.readTemp();
  	DTemp = lsm.temperature;
  #elif HAS_TPS()
	int iax, iay, iaz, igx, igy, igz, imx, imy, imz;
	amgData.readMotionSensor(iax, iay, iaz, igx, igy, igz, imx, imy, imz);
	
    values_cal[0] = (float) iax;
    values_cal[1] = (float) iay;
    values_cal[2] = (float) iaz;
	values_cal[3] = ((float) igx - gyro_off_x)/ gyro_sensitivity;
	values_cal[4] = ((float) igy - gyro_off_y)/ gyro_sensitivity;
	values_cal[5] = ((float) igz - gyro_off_z)/ gyro_sensitivity;
    values_cal[6] = (float) imx;
    values_cal[7] = (float) imy;
    values_cal[8] = (float) imz;
	
	DTemp = 9999;
  #else  // MPU6050 or other 6dof
    int16_t accgyroval[9];
	#if HAS_MPU9150() || HAS_MPU9250()
		mag.getHeading(&accgyroval[6], &accgyroval[7], &accgyroval[8]);	
		delay(10);
		accgyro.getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], 
						   &accgyroval[3], &accgyroval[4], &accgyroval[5]);	   
		// read raw heading measurements from device
		
		accgyroval[0] = mfilter_accx.filter((float) accgyroval[0]);
		accgyroval[1] = mfilter_accy.filter((float) accgyroval[1]);
		accgyroval[2] = mfilter_accz.filter((float) accgyroval[2]);
		
		values_cal[6] = mfilter_mx.filter((float) accgyroval[6]);
		values_cal[7] = mfilter_my.filter((float) accgyroval[7]);
		values_cal[8] = mfilter_mz.filter((float) accgyroval[8]); 
	#else
		accgyro.getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], 
						   &accgyroval[3], &accgyroval[4], &accgyroval[5]);
	#endif
	
	DTemp = accgyro.getTemperature();

	if(temp_corr_on == 1){
		if(DTemp < temp_break){    
			for( i = 0; i < 9; i++) { 
				acgyro_corr[i] = c3[i]*(DTemp*DTemp*DTemp) + c2[i]*(DTemp*DTemp) + c1[i]*DTemp + c0[i];
			}
		} 
	} else {
		//for( i = 0; i < 9; i++) { 
		//	acgyro_corr[i] = 0.0f;
	    //}
		memset(acgyro_corr, 0.0f, sizeof(acgyro_corr));
	}
	
    // remove offsets from the gyroscope
	if(temp_corr_on == 1){
		//accgyroval[3] = accgyroval[3] - acgyro_corr[3];
		//accgyroval[4] = accgyroval[4] - acgyro_corr[4];
		//accgyroval[5] = accgyroval[5] - acgyro_corr[5];
		
		values_cal[3] = (float) accgyroval[3] - acgyro_corr[3];
		values_cal[4] = (float) accgyroval[4] - acgyro_corr[4];
		values_cal[5] = (float) accgyroval[5] - acgyro_corr[5];
		}
	  else {
		values_cal[3] = (float) accgyroval[3] - gyro_off_x;
		values_cal[4] = (float) accgyroval[4] - gyro_off_y;
		values_cal[5] = (float) accgyroval[5] - gyro_off_z;
	  }
	  
    for( i = 0; i<6; i++) {
      if( i < 3 ) {
        values_cal[i] = (float) accgyroval[i] - acgyro_corr[i];
      }
      else {
        //values[i] = ((float) accgyroval[i] - acgyro_corr[i])/ 16.4f; // NOTE: this depends on the sensitivity chosen
		values_cal[i] = values_cal[i] / gyro_sensitivity;   //for 6050 etc
	  }
    }	
  #endif
  
  #warning Accelerometer calibration active: have you calibrated your device?
  // remove offsets and scale accelerometer (calibration)
  values_cal[0] = (values_cal[0] - acc_off_x) / acc_scale_x;
  values_cal[1] = (values_cal[1] - acc_off_y) / acc_scale_y;
  values_cal[2] = (values_cal[2] - acc_off_z) / acc_scale_z;
  
  #if HAS_HMC5883L()
    magn.getValues(&values_cal[6]);
  #endif
  
  #if HAS_HMC5883L() || HAS_MPU9150() || HAS_MPU9250() || HAS_LSM303() || HAS_LSM9DS0() \
	  || HAS_LSM9DS1() || HAS_TPS()
    // calibration
	if(temp_corr_on == 1) {
		values_cal[6] = (values_cal[6] - acgyro_corr[6] - magn_off_x) / magn_scale_x;
		values_cal[7] = (values_cal[7] - acgyro_corr[7] - magn_off_y) / magn_scale_y;
		values_cal[8] = (values_cal[8] - acgyro_corr[8] - magn_off_z) / magn_scale_z;
	}	
	else {
		#warning Magnetometer calibration active: have you calibrated your device?
		values_cal[6] = (values_cal[6] - magn_off_x) / magn_scale_x;
		values_cal[7] = (values_cal[7] - magn_off_y) / magn_scale_y;
		values_cal[8] = (values_cal[8] - magn_off_z) / magn_scale_z;
	}
  #endif
  
  for(uint8_t i = 0; i < 9; i++) {
	values[i] = sensor_sign[i] * values_cal[sensor_order[i]];
  }

}


/**
 * Computes gyro offsets
*/
void FreeIMU::zeroGyro() {
  const int totSamples = nsamples;
  int raw[11];
  float values[11]; 
  float tmpOffsets[] = {0,0,0};
  
  for (int i = 0; i < totSamples; i++){
	#if HAS_ITG3200()
		gyro.readGyro(&values[3]);
		tmpOffsets[0] += values[3];
		tmpOffsets[1] += values[4];
		tmpOffsets[2] += values[5];	
	#else
		getRawValues(raw);
		tmpOffsets[0] += raw[3];
		tmpOffsets[1] += raw[4];
		tmpOffsets[2] += raw[5]; 
	#endif
  }
  
  gyro_off_x = tmpOffsets[0] / totSamples;
  gyro_off_y = tmpOffsets[1] / totSamples;
  gyro_off_z = tmpOffsets[2] / totSamples;

  delay(5);
}

void FreeIMU::initGyros() {
	//Code modified from Ardupilot library
	//
    Vector3f last_average[INS_MAX_INSTANCES], best_avg[INS_MAX_INSTANCES], gyro_offset[INS_MAX_INSTANCES];
    float best_diff[INS_MAX_INSTANCES];
    bool converged[INS_MAX_INSTANCES];
	
	////digitalWrite(12,HIGH);
	
    // remove existing gyro offsets
    for (uint8_t k=0; k<num_gyros; k++) {
        gyro_offset[k] = Vector3f(0,0,0);
        best_diff[k] = 0;
        last_average[k].zero();
        converged[k] = false;
    }
    
	// the strategy is to average 50 points over 0.5 seconds, then do it
    // again and see if the 2nd average is within a small margin of
    // the first

    uint8_t num_converged = 0;	
	
    // we try to get a good calibration estimate for up to 10 seconds
    // if the gyros are stable, we should get it in 1 second
	for (int16_t j = 0; j <= 50 && num_converged < num_gyros; j++) {
		Vector3f gyro_sum[INS_MAX_INSTANCES], gyro_avg[INS_MAX_INSTANCES], gyro_diff[INS_MAX_INSTANCES];
		float diff_norm[INS_MAX_INSTANCES];
		float ToRad = 0.05f * M_PI/180.0f;
		//For FreeIMU and most boards we are using only one gyro
		//if you have more change code to match Arduimu
		zeroGyro();
		gyro_avg[0] = Vector3f(gyro_off_x, gyro_off_y,gyro_off_z) ;
		
		for (uint8_t k=0; k<num_gyros; k++) {
            gyro_diff[k] = last_average[k] - gyro_avg[k];
            diff_norm[k] = gyro_diff[k].length();
        }
		
		for (uint8_t k=0; k<num_gyros; k++) {
            if (converged[k]) continue;
            if (j == 0) {
                best_diff[k] = diff_norm[k];
                best_avg[k] = gyro_avg[k];				
            } else if (gyro_diff[k].length() < ToRad) {
                // we want the average to be within 0.1 bit, which is 0.04 degrees/s
                last_average[k] = (gyro_avg[k] * 0.5f) + (last_average[k] * 0.5f);
                gyro_offset[k] = last_average[k];
                converged[k] = true;
                num_converged++;
            } else if (diff_norm[k] < best_diff[k]) {
                best_diff[k] = diff_norm[k];
                best_avg[k] = (gyro_avg[k] * 0.5f) + (last_average[k] * 0.5f);
            }
            last_average[k] = gyro_avg[k];
        }
    }

	delay(5);
	
	if (num_converged == num_gyros) {
        // all OK
		gyro_off_x = gyro_offset[0].x;
		gyro_off_y = gyro_offset[0].y;
		gyro_off_z = gyro_offset[0].z;
		////digitalWrite(12,LOW);

		return;
	}

    // we've kept the user waiting long enough - use the best pair we
    // found so far
    for (uint8_t k=0; k<num_gyros; k++) {
        if (!converged[k]) {
            gyro_offset[k] = best_avg[k];
        }
    }
	
	gyro_off_x = gyro_offset[0].x;
	gyro_off_y = gyro_offset[0].y;
	gyro_off_z = gyro_offset[0].z;
	
	
	//digitalWrite(12,LOW);

}


/**
 * Populates array q with a quaternion representing the IMU orientation with respect to the Earth
 * 
 * @param q the quaternion to populate
*/
void FreeIMU::getQ(float * q, float * val) {
  //float val[12];
  getValues(val);
  //DEBUG_PRINT(val[3] * M_PI/180);
  //DEBUG_PRINT(val[4] * M_PI/180);
  //DEBUG_PRINT(val[5] * M_PI/180);
  //DEBUG_PRINT(val[0]);
  //DEBUG_PRINT(val[1]);
  //DEBUG_PRINT(val[2]);
  //DEBUG_PRINT(val[6]);
  //DEBUG_PRINT(val[7]);
  //DEBUG_PRINT(val[8]);
 
  now = micros();
  sampleFreq = 1.0 / ((now - lastUpdate) / 1000000.0);
  lastUpdate = now;
  
  // Set up call to the appropriate filter using the axes alignment information
  // gyro values are expressed in deg/sec, the * M_PI/180 will convert it to radians/sec
  #if(MARG == 0 || MARG == 1 || MARG == 3)
	#if IS_9DOM() && not defined(DISABLE_MAGN)
		#if MARG == 0
			AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[6], val[7], val[8]);
		#elif MARG == 1
			MadgwickAHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[6], val[7], val[8]);
		#else
			MARGUpdateFilter(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[6], val[7], val[8]);
		#endif
		val[9] = maghead.iheading(1, 0, 0, val[0], val[1], val[2], val[6], val[7], val[8]);
	#else
		#if MARG == 0
			AHRSupdateIMU(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2]);
		#elif MARG == 1
			MadgwickAHRSupdateIMU(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2]);
		#else
			MARGUpdateFilterIMU(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2]);
		#endif
		val[9] = -9999.0f;
	#endif

	q[0] = q0;	
	q[1] = q1;
  	q[2] = q2;
  	q[3] = q3;

  #endif
  
  #if (MARG == 4 && IS_9DOM() && not defined(DISABLE_MAGN))
    val[9] = maghead.iheading(1, 0, 0, val[0], val[1], val[2], val[6], val[7], val[8]);
	dcm.setSensorVals(val);
	dcm.G_Dt = 1./ sampleFreq;
  	dcm.calDCM();
  	dcm.getDCM2Q(q);
  #endif
	
  MotionDetect( val );

  //#if IS_9DOM() && not defined(DISABLE_MAGN)
	//if(val[11] - motiondetect_old < 0) {  
	//	getQ_simple(q, val);
	//}
  //#endif
 
  motiondetect_old = val[11];
  
  #if HAS_PRESS()
	val[10] = getEstAltitude(q, val, (1./sampleFreq));
  #endif
  
}

float def_sea_press = 1013.25;

#if HAS_MS5611() && !HAS_APM25()
	/**
	* Returns an altitude estimate from barometer readings only using sea_press as current sea level pressure
	*/
	float FreeIMU::getBaroAlt(float sea_press) {
		float temp = baro.getTemperature(MS561101BA_OSR_4096);
		float press = baro.getPressure(MS561101BA_OSR_4096);
        float new_press = kPress.measureRSSI(press);
		return ((pow((sea_press / new_press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
	}

	// Returns temperature from MS5611 - added by MJS
	float FreeIMU::getBaroTemperature() {
		float temp1 = baro.getTemperature(MS561101BA_OSR_4096);
		return(temp1);
	}

	float FreeIMU::getBaroPressure() {
		float temp2 = baro.getPressure(MS561101BA_OSR_4096);
		return(temp2);
	}

	/**
	* Returns an altitude estimate from baromether readings only using a default sea level pressure
	*/
	float FreeIMU::getBaroAlt() {
		return getBaroAlt(def_sea_press);
	}

#endif

#if HAS_MS5611() && HAS_APM25()
	/**
	* Returns an altitude estimate from barometer readings only using sea_press as current sea level pressure
	*/
	float FreeIMU::getBaroAlt(float sea_press) {
		//baro.read();
		float temp = baro.get_temperature()/100.0f;
		float press = baro.get_pressure()/100.0f;
        float new_press = kPress.measureRSSI(press);
		return ((pow((sea_press / new_press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
	}

	// Returns temperature from MS5611 - added by MJS
	float FreeIMU::getBaroTemperature() {
		baro.read();
		float temp1 = baro.get_temperature()/100.0f;
		return(temp1);
	}

	float FreeIMU::getBaroPressure() {
		baro.read();
		float temp2 = baro.get_pressure()/100.0f;
		return(temp2);
	}

	/**
	* Returns an altitude estimate from baromether readings only using a default sea level pressure
	*/
	float FreeIMU::getBaroAlt() {
		return getBaroAlt(def_sea_press);
	}

#endif

#if HAS_BMP085()
	//used for BMP085
	int32_t Temperature = 0, Pressure = 0, Altitude = 0;
	
	// Returns temperature from BMP085 - added by MJS
	float FreeIMU::getBaroTemperature() {
		baro085.getTemperature(&Temperature);
		float temp1 = Temperature * 0.1;		
		return(temp1);
	}

	float FreeIMU::getBaroPressure() {
		baro085.getPressure(&Pressure);
		float temp3 = Pressure * 0.01;
        //float new_press = kPress.measureRSSI(Pressure * 0.01);
		return(temp3);
	}

	/**
	* Returns an altitude estimate from baromether readings only using a default sea level pressure
	*/
	float FreeIMU::getBaroAlt() {
		//baro085.getAltitude(&Altitude);
		//return Altitude * 0.01;
		return getBaroAlt(def_sea_press);
	}
	
	/**
	* Returns an altitude estimate from barometer readings only using sea_press as current sea level pressure
	*/
	float FreeIMU::getBaroAlt(float sea_press) {
		float temp = getBaroTemperature();
		float press = getBaroPressure();
        float new_press = kPress.measureRSSI(press);
		return ((pow((sea_press / new_press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
	}
	
#endif

#if HAS_LPS()
	// Returns temperature from LPS - added by MJS
	float FreeIMU::getBaroTemperature() {
		float temp1 = baroLPS.readTemperatureC();		
		return(temp1);
	}

	float FreeIMU::getBaroPressure() {
        //float new_press = kPress.measureRSSI(baroLPS.readPressureMillibars());
		float new_press = baroLPS.readPressureMillibars();
		return(new_press);
	}

	/**
	* Returns an altitude estimate from baromether readings only using a default sea level pressure
	*/
	float FreeIMU::getBaroAlt() {        
        //float new_press = kPress.measureRSSI(baroLPS.readPressureMillibars());
		//float temp3 = baroLPS.pressureToAltitudeMeters(def_sea_press);
		return getBaroAlt(def_sea_press);
	}

	/**
	* Returns an altitude estimate from barometer readings only using sea_press as current sea level pressure
	*/
	float FreeIMU::getBaroAlt(float sea_press) {
		float temp = baroLPS.readTemperatureC();
		float press = baroLPS.readPressureMillibars();
        float new_press = kPress.measureRSSI(press);
		return ((pow((sea_press / new_press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
	}
	
#endif

#if HAS_MPL3115A2()
	// Returns temperature, pressure and altitude from MPL3115A2
	// added by https://github.com/mariocannistra
	
	float FreeIMU::getBaroTemperature() {
		float temp1 = baro3115.readTemp();
		// also available in °F float temp1 = baro3115.readTempF();
		return(temp1);
	}

	float FreeIMU::getBaroPressure() {
		// 1 kPa = 10 hPa = 1000 Pa
		// 100 Pascals = 1 hPa = 1 mb
		// sensor output is in Pa
		// need to convert Pascals to millibars:
		float new_press = baro3115.readPressure() / 100 ;
		return(new_press);
	}

	/**
	* Returns an altitude estimate from baromether readings only using a default sea level pressure
	*/
	float FreeIMU::getBaroAlt() {
		return getBaroAlt(def_sea_press);
	}

	/**
	* Returns an altitude estimate from barometer readings only using sea_press as current sea level pressure
	*/
	float FreeIMU::getBaroAlt(float sea_press) {
		float temp = baro3115.readTemp();
		float press = baro3115.readPressure() / 100.;
        float new_press = kPress.measureRSSI(press);
		return ((pow((sea_press / new_press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
	}	
#endif

#if HAS_MS5637()
	// Returns temperature, pressure and altitude from MS5637
	// from the Freetronics library
	
	float FreeIMU::getBaroTemperature() {
		float temp1 = baro5637.getTemperature();
		// also available in °F float temp1 = baro3115.readTempF();
		return(temp1);
	}

	float FreeIMU::getBaroPressure() {
		// 1 kPa = 10 hPa = 1000 Pa
		// 100 Pascals = 1 hPa = 1 mb
		// sensor output is in Pa
		// need to convert Pascals to millibars:
		float new_press = baro5637.getPressure() ;
		return(new_press);
	}

	/**
	* Returns an altitude estimate from baromether readings only using a default sea level pressure
	*/
	float FreeIMU::getBaroAlt() {
		return getBaroAlt(def_sea_press);
	}

	/**
	* Returns an altitude estimate from barometer readings only using sea_press as current sea level pressure
	*/
	float FreeIMU::getBaroAlt(float sea_press) {
		float temp = baro5637.getTemperature();
		float press = baro5637.getPressure();
        float new_press = kPress.measureRSSI(press);
		return ((pow((sea_press / new_press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
	}	
#endif

/**
 * Returns the estimated altitude from fusing barometer and accelerometer
 * in a complementary filter.
*/
#if HAS_PRESS()
float FreeIMU::getEstAltitude(float * q1, float * val, float dt2) {
  //float q1[4]; // quaternion
  float q2[4]; // quaternion
  //float val[12];
  float dyn_acc[4];
  float dyn_acc_temp[4];
  float dyn_acc_earth[4];

  //now1 = micros();
  //dt2 = (now1 - lastUpdate1) / 1000000.0;
  
  //getQ(q1, val);
  
  dyn_acc[0] = 0;
  dyn_acc[1] = val[0];
  dyn_acc[2] = val[1];
  dyn_acc[3] = val[2];
  
  //gravityCompensateAcc(dyn_acc, q1);
  gravityCompensateAcc(dyn_acc, q1);
  dyn_acc[0] = 0;
  
  //Qmultiply(dyn_acc_temp, q1, dyn_acc);
  Qmultiply(dyn_acc_temp, q1, dyn_acc);


  //q2[1] = -q1[1]; q2[2] = -q1[2]; q2[3] = -q1[3]; q2[0] = q1[0]; //Conjugating
  q2[1] = -q1[1]; q2[2] = -q1[2]; q2[3] = -q1[3]; q2[0] = q1[0]; //Conjugating
  Qmultiply(dyn_acc_earth, dyn_acc_temp, q2);
  
  float alt = getBaroAlt();
	
   //lastUpdate1 = now1;
  //return altComp.update(dyn_acc_earth[3], alt, (1./(sampleFreq*4)));

  return altComp.update(dyn_acc_earth[3], alt, dt2);
}
#endif

/**
 * Returns the Euler angles in radians defined in the Aerospace sequence.
 * See Sebastian O.H. Madwick report "An efficient orientation filter for 
 * inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
 * 
 * @param angles three floats array which will be populated by the Euler angles in radians
*/
void FreeIMU::getEulerRad(float * angles) {
  float q[4]; // quaternion
  float val[12];
  
  getQ(q, val);
  angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}


/**
 * Returns the Euler angles in degrees defined with the Aerospace sequence.
 * See Sebastian O.H. Madwick report "An efficient orientation filter for 
 * inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
 * 
 * @param angles three floats array which will be populated by the Euler angles in degrees
*/
void FreeIMU::getEuler(float * angles) {
  getEulerRad(angles);
  arr3_rad_to_deg(angles);
}

/**
 * Returns the Euler angles in degrees defined with the Aerospace sequence (NED).  Conversion
 * based on MATLAB quat2angle.m for an ZXY rotation sequence. euler[0] = yaw, euler[1]=pitch and euler[2] = roll
 * Angles are lie between 0-360 degrees.  
 * 
 * @param angles three floats array which will be populated by the Euler angles in degrees
*/
void FreeIMU::getEuler360deg(float * angles) {
  float m11, m12, m21, m31, m32;
  float gx, gy, gz; // estimated gravity direction
  float q[4]; // quaternion
  float val[12];
  
  getQ(q, val);
  
  gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    
  m11 = 2.*(q[1]*q[2] + q[0]*q[3]);
  m12 = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
  m21 = -2.*(q[1]*q[3] - q[0]*q[2]);               
  m31 = 2.*(q[2]*q[3] + q[0]*q[1]);              
  m32 = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

  // find angles for rotations about X, Y, and Z axes
  angles[0] = -atan2( m11, m12 ) * 57.2957795;
  angles[1] = -asin( m21 ) * 57.2957795;
  angles[2] = -atan2( m31, m32 ) * 57.2957795;
    
  //  	Gx	gy	gz
  //0-90	"+"		"+"
  //90-180	"+"		"-"
  //180-270	"-"		"-"
  //270-360	"-"		"+"
    
  if(gx >= 0 && gz < 0)
      angles[1] = 180. - angles[1];
     else if(gx < 0 && gz < 0)
       angles[1] = 180. - angles[1];
      else if(gx < 0 && gz >=0)
        angles[1] = 360. + angles[1];
        
  if(angles[0] < 0) angles[0] = 360. + angles[0];
  if(angles[2] < 0) angles[2] = 360. + angles[2];
  
  angles[0] = 360 - angles[0];
  
}

/**
 * Returns the Euler angles in degrees defined with the Aerospace sequence (NED).  Conversion
 * based on MATLAB quat2angle.m for an ZXY rotation sequence. euler[0] = yaw, euler[1]=pitch and euler[2] = roll
 * Angles are lie between 0-360 degrees.  Added return of q and val array.
 * 
 * @param angles three floats array which will be populated by the Euler angles in degrees
*/
void FreeIMU::getEuler360degAttitude(float * angles, float * q, float * val) {
  float m11, m12, m21, m31, m32;
  float gx, gy, gz; // estimated gravity direction
  //float q[4]; // quaternion
  //float val[12];
  
  getQ(q, val);
  
  gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    
  m11 = 2.*(q[1]*q[2] + q[0]*q[3]);
  m12 = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
  m21 = -2.*(q[1]*q[3] - q[0]*q[2]);               
  m31 = 2.*(q[2]*q[3] + q[0]*q[1]);              
  m32 = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

  // find angles for rotations about X, Y, and Z axes
  angles[0] = -atan2( m11, m12 ) * 57.2957795;
  angles[1] = -asin( m21 ) * 57.2957795;
  angles[2] = -atan2( m31, m32 ) * 57.2957795;
    
  //  	Gx	gy	gz
  //0-90	"+"		"+"
  //90-180	"+"		"-"
  //180-270	"-"		"-"
  //270-360	"-"		"+"
    
  if(gx >= 0 && gz < 0)
      angles[1] = 180. - angles[1];
     else if(gx < 0 && gz < 0)
       angles[1] = 180. - angles[1];
      else if(gx < 0 && gz >=0)
        angles[1] = 360. + angles[1];
        
  if(angles[0] < 0) angles[0] = 360. + angles[0];
  if(angles[2] < 0) angles[2] = 360. + angles[2];
  
  angles[0] = 360 - angles[0];
  
}


/**
 * Returns the Euler angles in degrees defined with the Aerospace sequence (NED).  Conversion
 * based on MATLAB quat2angle.m for an ZXY rotation sequence.
 * Angles are lie between 0-360 degrees in radians.
 * 
 * @param angles three floats array which will be populated by the Euler angles in degrees
*/
void FreeIMU::getEuler360(float * angles) {
  getEulerRad(angles);
  arr3_deg_to_rad(angles);
}


/**
 * Returns the yaw pitch and roll angles, respectively defined as the angles in radians between
 * the Earth North and the IMU X axis (yaw), the Earth ground plane and the IMU X axis (pitch)
 * and the Earth ground plane and the IMU Y axis.
 * 
 * @note This is not an Euler representation: the rotations aren't consecutive rotations but only
 * angles from Earth and the IMU. For Euler representation Yaw, Pitch and Roll see FreeIMU::getEuler
 * 
 * @param ypr three floats array which will be populated by Yaw, Pitch and Roll angles in radians
*/
void FreeIMU::getYawPitchRollRad(float * ypr) {
  float q[4]; // quaternion
  float val[12];
  float gx, gy, gz; // estimated gravity direction
  getQ(q, val);
  
  gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  
  ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1);
  ypr[1] = atan(gx / sqrt(gy*gy + gz*gz));
  ypr[2] = atan(gy / sqrt(gx*gx + gz*gz));
}

/**
 * Returns the yaw pitch and roll angles, respectively defined as the angles in radians between
 * the Earth North and the IMU X axis (yaw), the Earth ground plane and the IMU X axis (pitch)
 * and the Earth ground plane and the IMU Y axis.
 * 
 * @note This is not an Euler representation: the rotations aren't consecutive rotations but only
 * angles from Earth and the IMU. For Euler representation Yaw, Pitch and Roll see FreeIMU::getEuler
 * 
 * @param ypr three floats array which will be populated by Yaw, Pitch and Roll angles in radians
*/
void FreeIMU::getYawPitchRollRadAHRS(float * ypr, float * q) {
  //float q[4]; // quaternion
  //float val[12];
  float gx, gy, gz; // estimated gravity direction
  //getQ(q, val);
  
  gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  
  ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1);
  ypr[1] = atan(gx / sqrt(gy*gy + gz*gz));
  ypr[2] = atan(gy / sqrt(gx*gx + gz*gz));
}

/**
 * Returns the yaw pitch and roll angles, respectively defined as the angles in degrees between
 * the Earth North and the IMU X axis (yaw), the Earth ground plane and the IMU X axis (pitch)
 * and the Earth ground plane and the IMU Y axis. 
 * 
 * Roll and Pitch angle ranges 0-360 degrees instead of 0-90 degrees
 * 
 * @note This is not an Euler representation: the rotations aren't consecutive rotations but only
 * angles from Earth and the IMU. For Euler representation Yaw, Pitch and Roll see FreeIMU::getEuler
 * 
 * @param ypr three floats array which will be populated by Yaw, Pitch and Roll angles in degrees
*/
void FreeIMU::getYawPitchRoll180(float * ypr) {
	float q[4];				// quaternion
	float val[12];
	float gx, gy, gz;		// estimated gravity direction
	
	getQ(q, val);

	gx = 2 * (q[1]*q[3] - q[0]*q[2]);
	gy = 2 * (q[0]*q[1] + q[2]*q[3]);
	gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

	//calculating yaw
	//ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1);	
	ypr[0] = val[9];
	if(ypr[0] > 180.) ypr[0] = ypr[0] - 360.;
	ypr[0] = ypr[0] * 0.0174532925;
	
	//calculating Pitch1
	//Serial.print(gx); Serial.print("       "); Serial.print(gz); Serial.print("       ");
	if(gx > 0 && gz > 0) {
		ypr[1] = atan(gx / sqrt(gy*gy + gz*gz));
	} else if(gx > 0 && gz <= 0) {
		ypr[1] = PI - atan(gx / sqrt(gy*gy + gz*gz));
	} else if(gx < 0 && gz < 0) {
		ypr[1] = (-PI - atan(gx / sqrt(gy*gy + gz*gz)));
	} else  {
		ypr[1] =  atan(gx / sqrt(gy*gy + gz*gz));
	}
	
	//Calculating Roll1
    ypr[2] = atan(gy / sqrt(gx*gx + gz*gz));

}

/**
 * Returns the yaw pitch and roll angles, respectively defined as the angles in degrees between
 * the Earth North and the IMU X axis (yaw), the Earth ground plane and the IMU X axis (pitch)
 * and the Earth ground plane and the IMU Y axis.
 * 
 * @note This is not an Euler representation: the rotations aren't consecutive rotations but only
 * angles from Earth and the IMU. For Euler representation Yaw, Pitch and Roll see FreeIMU::getEuler
 * 
 * @param ypr three floats array which will be populated by Yaw, Pitch and Roll angles in degrees
*/
void FreeIMU::getYawPitchRoll(float * ypr) {
  getYawPitchRollRad(ypr);
  arr3_rad_to_deg(ypr);
}

/**
 * Get heading from magnetometer if LSM303 not available
 * code extracted from rob42/FreeIMU-20121106_1323 Github library
 * (https://github.com/rob42/FreeIMU-20121106_1323.git)
 * which is based on 
 *
**/
float FreeIMU::calcMagHeading(float q0, float q1, float q2, float q3, float bx, float by, float bz){
  float Head_X, Head_Y;
  float cos_roll, sin_roll, cos_pitch, sin_pitch;
  float gx, gy, gz; // estimated gravity direction
  float ypr[3];
  
  gx = 2 * (q1*q3 - q0*q2);
  gy = 2 * (q0*q1 + q2*q3);
  gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
  
  ypr[0] = atan2(2 * q1 * q2 - 2 * q0 * q3, 2 * q0*q0 + 2 * q1 * q1 - 1);
  ypr[1] = atan(gx / sqrt(gy*gy + gz*gz));
  ypr[2] = atan(gy / sqrt(gx*gx + gz*gz)); 

  cos_roll = cos(-ypr[2]);
  sin_roll = sin(-ypr[2]);
  cos_pitch = cos(ypr[1]);
  sin_pitch = sin(ypr[1]);
  
  //Example calc
  //Xh = bx * cos(theta) + by * sin(phi) * sin(theta) + bz * cos(phi) * sin(theta)
  //Yh = by * cos(phi) - bz * sin(phi)
  //return wrap((atan2(-Yh, Xh) + variation))
    
  // Tilt compensated Magnetic field X component:
  Head_X = bx*cos_pitch + by*sin_roll*sin_pitch + bz*cos_roll*sin_pitch;
  // Tilt compensated Magnetic field Y component:
  Head_Y = by*cos_roll - bz*sin_roll;
  // Magnetic Heading
  return (atan2(-Head_Y,Head_X)*180./M_PI) + MAG_DEC;
 
}

/**
 * Sets thermal calibration on (1) or off (0) for the accelerometer and gyro calibration from the
 * main sketch
*/
void FreeIMU::setTempCalib(int opt_temp_cal) {
   temp_corr_on = opt_temp_cal;
   if(temp_corr_on == 1) {
		gyro_off_x = 0.0;
		gyro_off_y = 0.0;
		gyro_off_z = 0.0;
	}
	if(temp_corr_on == 0) {
		//digitalWrite(12,HIGH);
		initGyros();
		//digitalWrite(12,LOW);
	}
}

/**
 * Sets sea level pressure
 * 
*/
void FreeIMU::setSeaPress(float sea_press_inp) {

	def_sea_press = sea_press_inp;
}

void FreeIMU::getQ_simple(float * q, float * val)
{
 
  float yaw;
  float pitch = atan2(val[0], sqrt(val[1]*val[1]+val[2]*val[2]));
  float roll = -atan2(val[1], sqrt(val[0]*val[0]+val[2]*val[2]));
  
  yaw = val[9] - MAG_DEC;
  
  if(val[9] > 180.) {	yaw = (yaw - 360.) * M_PI/180;
   } else {
    yaw = yaw * M_PI/180;
   }
  
  float rollOver2 = roll * 0.5f;
  float sinRollOver2 = (float)sin(rollOver2);
  float cosRollOver2 = (float)cos(rollOver2);
  float pitchOver2 = pitch * 0.5f;
  float sinPitchOver2 = (float)sin(pitchOver2);
  float cosPitchOver2 = (float)cos(pitchOver2);
  float yawOver2 = yaw * 0.5f;
  float sinYawOver2 = (float)sin(yawOver2);
  float cosYawOver2 = (float)cos(yawOver2);

  q1 = cosYawOver2 * cosPitchOver2 * sinRollOver2 - sinYawOver2 * sinPitchOver2 * cosRollOver2;
  q0 = cosYawOver2 * cosPitchOver2 * cosRollOver2 + sinYawOver2 * sinPitchOver2 * sinRollOver2;
  q2 = sinYawOver2 * cosPitchOver2 * cosRollOver2 - cosYawOver2 * sinPitchOver2 * sinRollOver2;
  q3 = cosYawOver2 * sinPitchOver2 * cosRollOver2 + sinYawOver2 * cosPitchOver2 * sinRollOver2;
  
  if (q!=NULL){
	  q[0] = q0;
	  q[1] = q1;
	  q[2] = q2;
	  q[3] = q3;
  }

}

void FreeIMU::MotionDetect(float * val) {
	
	float gyro[3];
	float accnorm;
	int accnorm_test, accnorm_var_test, omegax, omegay, omegaz, omega_test, motionDetect;
	
	gyro[0] = val[3] * M_PI/180;
	gyro[1] = val[4] * M_PI/180;
	gyro[2] = val[5] * M_PI/180;
	
    /*###################################################################
    #
    #   acceleration squared euclidean norm analysis
	#   
	#   some test values previously used:
	#       if((accnorm >=0.96) && (accnorm <= 0.99)){
	#										<= 0.995
    #
    ################################################################### */
    accnorm = (val[0]*val[0]+val[1]*val[1]+val[2]*val[2]);
    if((accnorm >=0.94) && (accnorm <= 1.03)){  
        accnorm_test = 0;
    } else {
        accnorm_test = 1; }
    
	/** take average of 5 to 10 points  **/
    float accnormavg = accnorm_avg.process(accnorm);
    float accnormtestavg = accnorm_test_avg.process(accnorm_test);

    /*####################################################################
    #
    #   squared norm analysis to determine suddenly changes signal
    #
    ##################################################################### */
    //accnorm_var.process(sq(accnorm-accnorm_avg.getAvg()));
	// was 0.0005
    if(accnorm_var.process(sq(accnorm-accnormavg)) < 0.0005) {
        accnorm_var_test = 0;
    } else {
        accnorm_var_test = 1; }

    /*###################################################################
    #
    #   angular rate analysis in order to disregard linear acceleration
    #
	#   other test values used: 0, 0.00215, 0.00215
    ################################################################### */
    if ((gyro[0] >=-0.005) && (gyro[0] <= 0.005)) {
        omegax = 0;
    } else {
        omegax = 1; }
        
    if((gyro[1] >= -0.005) && (gyro[1] <= 0.005)) {
        omegay = 0;
    } else {
        omegay = 1; }
        
    if((gyro[2] >= -0.005) && (gyro[2] <= 0.005)) {
        omegaz = 0;
    } else {
        omegaz = 1; }
        
    if ((omegax+omegay+omegaz) > 0) {
        omega_test = 1;
    } else {
        omega_test = 0; }


    /* 
	###################################################################
    #
    # combined movement detector
    #
    #################################################################### 
	*/
    if ((accnormtestavg + omega_test + accnorm_var_test) > 0) {
        motionDetect = 1;
    } else {
        motionDetect = 0; }

    /* 
	################################################################## 
	*/   
    
    //motion_detect_ma.process(motionDetect);
    
    if(motion_detect_ma.process(motionDetect) > 0.5) {
		val[11] = 1.0f;
    } else {
		val[11] = 0.0f;
	}
}


/**                           END OF FREEIMU                           **/
/************************************************************************/
/**                           HELP FUNCTIONS                           **/

/**
 * Compensates the accelerometer readings in the 3D vector acc expressed in the sensor frame for gravity
 * @param acc the accelerometer readings to compensate for gravity
 * @param q the quaternion orientation of the sensor board with respect to the world
*/
void gravityCompensateAcc(float * acc, float * q) {
  float g[3];
  
  // get expected direction of gravity in the sensor frame
  g[0] = 2 * (q[1] * q[3] - q[0] * q[2]);
  g[1] = 2 * (q[0] * q[1] + q[2] * q[3]);
  g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
  
  // compensate accelerometer readings with the expected direction of gravity
  acc[0] = acc[0] - g[0];
  acc[1] = acc[1] - g[1];
  acc[2] = acc[2] - g[2];
}

/**
 * Sets the Quaternion to be equal to the product of quaternions {@code q1} and {@code q2}.
 * 
 * @param q1
 *          the first Quaternion
 * @param q2
 *          the second Quaternion
 */
void Qmultiply(float *  q, float *  q1, float * q2) {
    q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    q[1] = q1[0] * q2[1] + q2[0] * q1[1] + q1[2] * q2[3] - q1[3] * q2[2];
    q[2] = q1[0] * q2[2] + q2[0] * q1[2] - q1[1] * q2[3] + q1[3] * q2[1];
    q[3] = q1[0] * q2[3] + q2[0] * q1[3] + q1[1] * q2[2] - q1[2] * q2[1];
}

/**
 * Converts a 3 elements array arr of angles expressed in radians into degrees
*/
void arr3_rad_to_deg(float * arr) {
  arr[0] *= 180/M_PI;
  arr[1] *= 180/M_PI;
  arr[2] *= 180/M_PI;
}

/**
 * Converts a 3 elements array arr of angles expressed in degrees into radians
*/
void arr3_deg_to_rad(float * arr) {
  arr[0] /= 180/M_PI;
  arr[1] /= 180/M_PI;
  arr[2] /= 180/M_PI;
}

/* Madgwick IMU/AHRS and Fast Inverse Square Root
http://www.diydrones.com/forum/topics/madgwick-imu-ahrs-and-fast-inverse-square-root

"After experimenting with real sensors I moved to artificial ACC input data and set up a 
test bed for Madgwick's algorithm (MadgwickTests on GitHub). I've figured out that in 
Madgwick's algorithm the fast inverse square root leads to huge instabilities when noisy 
measurements are applied.

Posted by Tobias Simon on November 2, 2012 
*/

//int instability_fix = 1;

float FreeIMU::invSqrt(float x) {
        if (instability_fix == 0)
        {
             union {
               float f;
               int32_t i;
               } y;

             y.f = x;
             y.i = 0x5f375a86 - (y.i >> 1);
             y.f = y.f * ( 1.5f - ( x * 0.5f * y.f * y.f ) );
             return y.f;
        }
        else if (instability_fix == 1)
        {
                /* close-to-optimal  method with low cost from
				http://pizer.wordpress.com/2008/10/12/fast-inverse-square-root */
                uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
                float tmp = *(float*)&i;
                return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
        }
        else
        {
                /* optimal but expensive method: */
                return 1.0f / sqrt(x);
        }
}

/**
 * Fast inverse square root implementation. Compatible both for 32 and 8 bit microcontrollers.
 * @see http://en.wikipedia.org/wiki/Fast_inverse_square_root
*/
/* original code from FreeIMU Library
float invSqrt(float number) {
  union {
    float f;
    int32_t i;
  } y;

  y.f = number;
  y.i = 0x5f375a86 - (y.i >> 1);
  y.f = y.f * ( 1.5f - ( number * 0.5f * y.f * y.f ) );
  return y.f;
}
*/
/* Old 8bit version. Kept it here only for testing/debugging of the new code above.
float invSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}
*/
