/*
FreeIMU.h - A libre and easy to use orientation sensing library for Arduino
Copyright (C) 2011 Fabio Varesano <fabio at varesano dot net>

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

*/

#ifndef FreeIMU_6050_H_
#define FreeIMU_6050_H_

// Uncomment the appropriated version of FreeIMU you are using
#define FREEIMU_v04

// 3rd party boards. Please consider donating or buying a FreeIMU board to support this library development.
//#define GEN_MPU6050
//#define MPU9250_5611  //MPU-9250 IMU with MS5611 Altimeter from eBay
//#define GEN_MPU9150
//#define GEN_MPU9250  	// Use for Invensense MPU-9250 breakout board
//#define APM_2_5  		//  APMM 2.5.2 (EBAY)

//#define DISABLE_MAGN // Uncomment this line to disable the magnetometer in the sensor fusion algorithm

//Magnetic declination angle for iCompass
//#define MAG_DEC 4 //+4.0 degrees for Israel
#define MAG_DEC -13.1603  //degrees for Flushing, NY
//#define MAG_DEC 0

//Number of samples to average in iCompass
#define WINDOW_SIZE 1 //Set to 1 to turn off the Running Average

// Set filter type: 1 = Madgwick Gradient Descent, 0 - Madgwick implementation of Mahoney DCM
// in Quaternion form, 3 = Madwick Original Paper AHRS, 4 - DCM Implementation
#define MARG 4

// proportional gain governs rate of convergence to accelerometer/magnetometer
// integral gain governs rate of convergence of gyroscope biases
// set up defines for various boards in my inventory, DFROBOT and Freeimu have
// temperature calibration curves. (3.31.14)

#if defined(FREEIMU_v04)
	#define twoKpDef  (2.0f * 0.75f)	//works with and without mag enabled
	#define twoKiDef  (2.0f * 0.1625f)
	#define betaDef  0.085f
	//Used for DCM filter
	const float Kp_ROLLPITCH = 1.2f;  //was .3423
	const float Ki_ROLLPITCH = 0.0234f;
	const float Kp_YAW = 1.75f;   // was 1.2 and 0.02
	const float Ki_YAW = 0.002f;
#elif defined(GEN_MPU6050) || defined(GY86)
	#define twoKpDef  (2.0f * 0.5f)
	#define twoKiDef  (2.0f * 0.25f)
	#define betaDef	  0.2f
	//Used for DCM filter
	const float Kp_ROLLPITCH = 1.2f;  //was .3423
	const float Ki_ROLLPITCH = 0.0234f;
	const float Kp_YAW = 1.75f;   // was 1.2 and 0.02
	const float Ki_YAW = 0.002f;
#elif defined(GEN_MPU9150)
	#define twoKpDef  (2.0f * 0.75f)
	#define twoKiDef  (2.0f * 0.1f)	
	#define betaDef	  0.01f
	//Used for DCM filter
	const float Kp_ROLLPITCH = 1.2f;  //was .3423
	const float Ki_ROLLPITCH = 0.0234f;
	const float Kp_YAW = 1.75f;   // was 1.2 and 0.02
	const float Ki_YAW = 0.002f;
#elif defined(GEN_MPU9250) || defined(MPU9250_5611)
	#define twoKpDef  (2.0f * 1.75f) // was 0.95
	#define twoKiDef  (2.0f * 0.05f) // was 0.05	
	#define betaDef	  0.015f
	//Used for DCM filter
	const float Kp_ROLLPITCH = 1.2f;  //was .3423
	const float Ki_ROLLPITCH = 0.0234f;
	const float Kp_YAW = 1.75f;   // was 1.2 and 0.02
	const float Ki_YAW = 0.002f;
#elif defined(APM_2_5)
	#define twoKpDef  (2.0f * 0.5f)
	#define twoKiDef  (2.0f * 0.25f)
	#define betaDef	  0.015f	//was 0.015
	//Used for DCM filter
	const float Kp_ROLLPITCH = 1.2f;  //was .3423
	const float Ki_ROLLPITCH = 0.0234f;
	const float Kp_YAW = 1.75f;   // was 1.2 and 0.02
	const float Ki_YAW = 0.002f;
#else
	#define twoKpDef  (2.0f * 0.5f)
	#define twoKiDef  (2.0f * 0.1f)
	#define betaDef  0.1f
	//Used for DCM filter
	const float Kp_ROLLPITCH = 1.2f;  //was .3423
	const float Ki_ROLLPITCH = 0.0234f;
	const float Kp_YAW = 1.75f;   // was 1.2 and 0.02
	const float Ki_YAW = 0.002f;
#endif 

//
// Other Options
//
  #define temp_break  -1000	  //original temp_break = -4300;
  #define senTemp_break  32
  #define temp_corr_on_default  0
  #define nsamples 75
  #define instability_fix 1

// ****************************************************
// *** No configuration needed below this line      ***
// *** Unless you are defining a new IMU            ***
// ***                                              ***
// *** Define Marg= 3 factors: go to line 491       ***
// *** Define IMU Axis Alignment: go to line 500    ***
// ****************************************************
#define FREEIMU_LIB_VERSION "DEV"

#define FREEIMU_DEVELOPER "Fabio Varesano - varesano.net"

#if F_CPU == 16000000L
  #define FREEIMU_FREQ "16 MHz"
#elif F_CPU == 8000000L
  #define FREEIMU_FREQ "8 MHz"
#elif F_CPU == 24000000L
  #define FREEIMU_FREQ "24 MHz"
#elif F_CPU == 48000000L
  #define FREEIMU_FREQ "48 MHz" 
#elif F_CPU == 72000000L
  #define FREEIMU_FREQ "48 MHz" 
#elif F_CPU == 84000000L
  #define FREEIMU_FREQ "84 MHz"  
#elif F_CPU == 96000000L
  #define FREEIMU_FREQ "96 MHz"  
#else
  #define FREEIMU_FREQ "Not Defined"  
#endif


// define board IDs
#if defined(FREEIMU_v04)
  #define FREEIMU_ID "FreeIMU v0.4"
#elif defined(GEN_MPU6050)
  #define FREEIMU_ID "GEN MPU-6050"
#elif defined(GEN_MPU9150)
  #define FREEIMU_ID "GEN MPU-9150"  
#elif defined(MPU9250_5611)
  #define FREEIMU_ID "MPU9250_5611"
#elif defined(GEN_MPU9250)
  #define FREEIMU_ID "GEN MPU-9250"
#elif defined(APM_2_5)
  #define FREEIMU_ID "APM 2.5.2 (EBAY)" 
#elif  defined(Mario)
  #define FREEIMU_ID "MPU-9150 plus MPL3115A2" 
#elif defined(Microduino)
  #define FREEIMU_ID "Microduino IMU" 
#endif

// define imu sensors
#define HAS_MPU6050() (defined(Microduino) || defined(FREEIMU_v04) || defined(GEN_MPU6050))
#define HAS_MPU9150() (defined(GEN_MPU9150) )
#define HAS_MPU9250() (defined(MPU9250_5611) || defined(GEN_MPU9250) || defined(MPU9250_5611)) 
#define HAS_HMC5883L() (defined(FREEIMU_v04) || defined(APM_2_5) || defined(Microduino))
#define HAS_MPU6000() ( defined(APM_2_5))
#define HAS_APM25()	(defined(APM_2_5))

#define HAS_MS5611() (defined(MPU9250_5611) || defined(FREEIMU_v04) || defined(APM_2_5) || defined(GY86))

#define HAS_PRESS() (defined(MPU9250_5611) | defined(FREEIMU_v04) || defined(FREEIMU_v04) \
					|| defined(APM_2_5) || defined(Microduino))
					
#define IS_6DOM() (defined(GEN_MPU6050))
#define IS_9DOM() (defined(GEN_MPU9250) || defined(MPU9250_5611) || defined(GEN_MPU9150)  \
				   || defined(FREEIMU_v04) || defined(APM_2_5) || defined(Microduino) )
#define HAS_AXIS_ALIGNED() (defined(GEN_MPU6050) || defined(FREEIMU_v04) || defined(Microduino) )

#include <Wire.h>
#include "Arduino.h"
#include "calibration.h"
#include <MovingAvarageFilter.h>

#ifndef CALIBRATION_H
	#include <EEPROM.h>
#endif

#define FREEIMU_EEPROM_BASE 0x0A
#define FREEIMU_EEPROM_SIGNATURE 0x19

#if(MARG == 4)
	#include "DCM.h"
#endif

//Combo IMUs
#if HAS_MPU6050()
  #include <Wire.h>
  #include "I2Cdev.h"
  #include "MPU60X0.h"
  //MPU Address Select 
  //Use following define if MPU60X0 address is set to 0x69
  //otherwise the default address is used = 0x68
  //uncomment following line and comment out successor line
  //#define FIMU_ACCGYRO_ADDR MPU60X0_ADDRESS_AD0_HIGH
  #define FIMU_ACCGYRO_ADDR MPU60X0_DEFAULT_ADDRESS
#elif HAS_MPU6000()
  #include <SPI.h>
  #include "I2Cdev.h"
  #include "MPU60X0.h"
  #define FIMU_ACCGYRO_ADDR MPU60X0_DEFAULT_SS_PIN
#elif HAS_MPU9150()
  #include <Wire.h>
  #include "I2Cdev.h"
  #include "MPU60X0.h"
  #include "AK8975.h"
  #include "iCompass.h"
  //MPU Address Select 
  //Use following define if MPU60X0 address is set to 0x69
  //otherwise the default address is used = 0x68
  //uncomment following line and comment out successor line
  //#define FIMU_ACCGYRO_ADDR MPU60X0_ADDRESS_AD0_HIGH
  #define FIMU_ACCGYRO_ADDR MPU60X0_DEFAULT_ADDRESS
#elif HAS_MPU9250()
  #include <Wire.h>
  #include "I2Cdev.h"
  #include "MPU60X0.h"
  #include "AK8963.h"
  #include "iCompass.h"
  //MPU Address Select 
  //Use following define if MPU60X0 address is set to 0x69
  //otherwise the default address is used = 0x68
  //uncomment following line and comment out successor line
  //#define FIMU_ACCGYRO_ADDR MPU60X0_ADDRESS_AD0_HIGH
  #define FIMU_ACCGYRO_ADDR MPU60X0_DEFAULT_ADDRESS
#endif

//Magnetometer and Accelerometer combo, made this standalone
//if you just want to use as standalone or in combo with a different
//gyro

#if HAS_PRESS()  //Setup pressure sensors and .h files
  #include <FilteringScheme.h>
  #include <AltitudeComplementary.h>
  
  #if HAS_MS5611()
    #if HAS_APM25()
	  #include <AP_Baro_MS5611.h>
    #else
	  #include <MS561101BA.h>
	  #define FIMU_BARO_ADDR MS561101BA_ADDR_CSB_LOW
	  //#define FIMU_BARO_ADDR MS561101BA_ADDR_CSB_HIGH
    #endif
  #endif
#endif


#if HAS_HMC5883L()  //Magnetometer
  #include <HMC58X3.h>
  #include "iCompass.h"
  // HMC5843 address is fixed so don't bother to define it
#endif

#ifndef cbi
    #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

class FreeIMU_6050
{
  public:
    FreeIMU_6050();
	void init();
    /*void init();
	//void init0(bool fastmode);
    //void init(bool fastmode); */
	void RESET();
	void RESET_Q();
	
	void init(bool fastmode);
	void init(int accgyro_addr, bool fastmode);
	
    #ifndef CALIBRATION_H
		void calLoad();
    #endif
	
    void zeroGyro();
	void initGyros();
    void getRawValues(int * raw_values);
    void getValues(float * values);
    void getQ(float * q, float * val);
    //void getEuler(float * angles);
    void getYawPitchRoll(float * ypr);
    //void getEulerRad(float * angles);
    void getYawPitchRollRad(float * ypr);
	void getYawPitchRollRadAHRS(float * ypr, float * q);
	//void getYawPitchRoll180(float * ypr);
    void getEuler360deg(float * angles);	
    void getEuler360(float * angles);
	void getEuler360degAttitude(float * angles, float * q, float * val);
	float invSqrt(float x);
	void setTempCalib(int opt_temp_cal);
	void setSeaPress(float sea_press_inp);
	float calcMagHeading(float q0, float q1, float q2, float q3, float bx, float by, float bz);
	//void getQ_simple(float* q, float * val);
	//void MotionDetect(float * val);
   
	#if HAS_PRESS()
      //float getEstAltitude();
	  float getEstAltitude(float * q, float * val, float dt2);
	  #if HAS_MS5611()
        float getBaroAlt();
        float getBaroAlt(float sea_press);
	    float getBaroTemperature();
	    float getBaroPressure();
	  #endif  
    #endif 

	
	#if(MARG == 4)
		DCM dcm;
	#endif
	
    // we make them public so that users can interact directly with device classes
    
    #if HAS_HMC5883L()
      HMC58X3 magn;
	  iCompass maghead;	
    #endif

    #if HAS_MPU6050()
      MPU60X0 accgyro; 
    #elif HAS_MPU6000()
      MPU60X0 accgyro;
	#elif HAS_MPU9150()
	  MPU60X0 accgyro;
	  AK8975 mag;
	  iCompass maghead;	  
	#elif HAS_MPU9250()
	  MPU60X0 accgyro;
	  AK8963 mag;
	  iCompass maghead;	
    #endif
      
    #if HAS_PRESS()
      KalmanFilter kPress; // Altitude Kalman Filter.
      AltComp altComp; // Altitude Complementary Filter.
	  
      #if HAS_MS5611()
		#if HAS_APM25()
			AP_Baro_MS5611 baro;
		#else
			MS561101BA baro;
		#endif
      #endif
	  
    #endif
     
	//Global Variables
	 
    int* raw_acc, raw_gyro, raw_magn;
    // calibration parameters
    int16_t gyro_off_x, gyro_off_y, gyro_off_z;
    int16_t acc_off_x, acc_off_y, acc_off_z, magn_off_x, magn_off_y, magn_off_z;
    float acc_scale_x, acc_scale_y, acc_scale_z, magn_scale_x, magn_scale_y, magn_scale_z;
	float val[12];
	//int8_t nsamples, temp_break, instability_fix, senTemp_break;
	int16_t DTemp; 
	float rt, senTemp, gyro_sensitivity;
	float sampleFreq; // half the sample period expressed in seconds

	
	// --------------------------------------------------------------------
	// Define Marg = 3 factors here
	// --------------------------------------------------------------------
	#define gyroMeasError 3.14159265358979 * (.50f / 180.0f) 	// gyroscope measurement error in rad/s (shown as 5 deg/s)
	#define gyroMeasDrift 3.14159265358979 * (0.02f / 180.0f) 	// gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)

	#define beta1 sqrt(3.0f / 4.0f) * gyroMeasError 			// compute beta
	#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift 				// compute zeta
	
	// --------------------------------------------------------------------
	// Define IMU Axis Alignment here
	// --------------------------------------------------------------------	
	#if HAS_AXIS_ALIGNED()
		//accx, accy, accz, gyrox, gyroy, gyroz, magx, magy, magz
		int sensor_order[9] = {0,1,2,3,4,5,6,7,8};
		int sensor_sign[9] = {1,1,1,1,1,1,1,1,1};
	#elif defined(GEN_MPU9150) || defined(MPU9250_5611) || defined(GEN_MPU9250) || defined(GY86)
		int sensor_order[9] = {0,1,2,3,4,5,7,6,8};
		int sensor_sign[9] = {1,1,1,1,1,1,1,-1};	
	#elif defined(APM_2_5)	
		int sensor_order[9] = {1,0,2,4,3,5,7,6,8};
		int sensor_sign[9] = {1,-1,1,1,-1,1,-1,1,1};
	#endif 	

	// --------------------------------------------------------------------
	// No further changes below this point
	// --------------------------------------------------------------------
	
  private:
    //void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    //void AHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    
	bool  bSPI;
	float bx, by, bz;
    float iq0, iq1, iq2, iq3;
    float exInt, eyInt, ezInt;  			// scaled integral error
    volatile float twoKp;      				// 2 * proportional gain (Kp)
    volatile float twoKi;      				// 2 * integral gain (Ki)
    volatile float q0, q1, q2, q3, q3old; 	// quaternion of sensor frame relative to auxiliary frame
    volatile float integralFBx,  integralFBy, integralFBz;
    unsigned long lastUpdate, now; 			// sample period expressed in milliseconds
	unsigned long lastUpdate1 = 0;
	unsigned long now1;
	
	//Madgwick AHRS Gradient Descent 
    volatile float beta;				// algorithm gain

	//Following lines defines Madgwicks Grad Descent Algorithm from his original paper
	// Global system variables
	float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0; 	// estimated orientation quaternion elements with initial conditions
	float b_x = 1, b_z = 0; 				// reference direction of flux in earth frame
	float w_bx = 0, w_by = 0, w_bz = 0; // estimate gyroscope biases error

	#if(MARG == 0)
		void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
		//void AHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
  	#elif(MARG == 1)
		void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
		void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
	#elif(MARG == 3)
		void MARGUpdateFilter(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
		void MARGUpdateFilterIMU(float gx, float gy, float gz, float ax, float ay, float az);
	#endif
};

float invSqrt(float number);
void arr3_rad_to_deg(float * arr);
void arr3_deg_to_rad(float * arr);
void Qmultiply(float *  q, float *  q1, float * q2);
void gravityCompensateAcc(float * acc, float * q);

#endif // FreeIMU_6050_h

