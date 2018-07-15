#ifndef DCM_H
#define DCM_H

/***************************************************************************************************************
* Razor AHRS Firmware v1.4.2
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" (SEN-10125 and SEN-10736)
* and "9DOF Sensor Stick" (SEN-10183, 10321 and SEN-10724)
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2013 Peter Bartz [http://ptrbrtz.net]
* Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
*
* Infos, updates, bug reports, contributions and feedback:
*     https://github.com/ptrbrtz/razor-9dof-ahrs
*
*
* History:
*   * Original code (http://code.google.com/p/sf9domahrs/) by Doug Weibel and Jose Julio,
*     based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio 
*/

/*
  Axis definition (differs from definition printed on the board!):
    X axis pointing forward (towards the short edge with the connector holes)
    Y axis pointing to the right
    and Z axis pointing down.
    
  Positive yaw   : clockwise
  Positive roll  : right wing down
  Positive pitch : nose up
  
  Transformation order: first yaw then pitch then roll.
*/


// DEBUG OPTIONS
/*****************************************************************/
// When set to true, gyro drift correction will not be applied

/*****************************************************************/

class DCM
{
	public:
		DCM();
		
		// DCM parameters
		//#define Kp_ROLLPITCH 0.02f
		//#define Ki_ROLLPITCH 0.00002f
		//#define Kp_YAW 1.2f
		//#define Ki_YAW 0.00002f
		
		//#define Kp_ROLLPITCH 1.2f  //was .3423
		//#define Ki_ROLLPITCH 0.0234f
		//#define Kp_YAW 1.75f   // was 1.2 and 0.02
		//#define Ki_YAW 0.002f		
		
		#define DEBUG__NO_DRIFT_CORRECTION false
		
		// Stuff
		//#define GRAVITY 256.0f 				   // "1G reference" used for DCM filter and accelerometer calibration
		#define GRAVITY 1.0f //everything coming accross normalized from FreeIMU library.
		#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
		#define TO_DEG(x) (x * 57.2957795131)  // *180/pi			
		
		// Sensor variables
		float accel[3] = {0, 0, 0};  
		float magnetom[3] = {0, 0, 0};
		float gyro[3] = {0, 0, 0};
		
		// DCM variables
		float MAG_Heading = 0.0f;
		
		// Euler angles
		float yaw = 0.0f;
		float pitch = 0.0f;
		float roll = 0.0f;
		float q0, q1, q2, q3;
		
		// DCM timing in the main loop
		unsigned long timestamp = 0;
		unsigned long timestamp_old = 0;
		float G_Dt = 0.0f; // Integration time for DCM algorithm
		
		void DCM_init(float Kp_RP, float Ki_RP, float Kp_Y, float Ki_Y);
		void setSensorVals(float * cal_val);
		void Matrix_update(void);
		void Drift_correction(void);
		void Normalize(void);
		void reset_sensor_fusion();
		void getEulerRad(float * angles);
		void getEulerDeg(float * angles);
		void calDCM();
		void getDCM2Q(float * q);
	
	  private:
		// Sensor variables
		float magnetom_tmp[3] = {0, 0, 0};

		// DCM variables
		float Kp_ROLLPITCH, Ki_ROLLPITCH, Kp_YAW, Ki_YAW;
		float Accel_Vector[3]= {0, 0, 0}; // Store the acceleration in a vector
		float Gyro_Vector[3]= {0, 0, 0}; // Store the gyros turn rate in a vector
		float Omega_Vector[3]= {0, 0, 0}; // Corrected Gyro_Vector data
		float Omega_P[3]= {0, 0, 0}; // Omega Proportional correction
		float Omega_I[3]= {0, 0, 0}; // Omega Integrator
		float Omega[3]= {0, 0, 0};
		float errorRollPitch[3] = {0, 0, 0};
		float errorYaw[3] = {0, 0, 0};
		float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
		float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
		float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

		// More output-state variables
		int num_accel_errors = 0;
		int num_magn_errors = 0;
		int num_gyro_errors = 0;
	
		void init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll);
		void Matrix_Vector_Multiply( float a[3][3],  float b[3], float out[3]);
		void Matrix_Multiply( float a[3][3],  float b[3][3], float out[3][3]);
		void Vector_Add(float out[3],  float v1[3],  float v2[3]);
		void Vector_Scale(float out[3],  float v[3], float scale);
		void Vector_Cross_Product(float *out,  float *v1,  float *v2);
		float Vector_Dot_Product(float *v1, float *v2);

};		
		

#endif