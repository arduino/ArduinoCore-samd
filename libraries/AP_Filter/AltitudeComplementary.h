#ifndef __FILTER_ALTCOMP_H__
#define __FILTER_ALTCOMP_H__

#include <math.h>

////////////////////////////////////////////////////////////
// complementary filter from Palla Software
// 

class AltComp {

  private:
    float Kp1;                // PI observer velocity gain 
    float Kp2;                 // PI observer position gain
    float Ki;               // PI observer integral gain (bias cancellation)
  
    float AltErrorI;
    float AltError;
    float InstAcc;
    float Delta;
    float EstVelocity;
    float EstAlt;  
  
  public:
    
    AltComp() : Kp1(0.55f), Kp2(1.0f), Ki(0.001f), AltErrorI(0.0f), AltError(0.0f), InstAcc(0.0f),
				Delta(0.0f), EstVelocity(0.0f), EstAlt(0.0f) {}
    
    /**
     * Computes the complementary estimation of Altitude based from the Barometer and Accelerometer.
     * @param dyn_acc_z the Z component of the dynamic acceleration expressed in the Earth frame
     * @param BaroAlt the Barometer altitude reading as coming from the sensor expressed in meters with decimals
    */
    float update(float dyn_acc_z, float BaroAlt, float dt){
        // Estimation Error
		if(!isfinite(BaroAlt)) BaroAlt = 0.;
        AltError = BaroAlt - EstAlt;
        AltErrorI += AltError;
		
        AltErrorI = constrain(AltErrorI,-150.00,+150.00);  //was 150

        InstAcc = dyn_acc_z * 9.80665; //+ ((self.AltErrorI / 1000) if self.AltErrorI != 0.0 else 0)
        if (AltErrorI != 0.0){
          //self.InstAcc += self.AltErrorI / 10000.0
          InstAcc = AltErrorI * Ki;
        }
        if(isnan(InstAcc)){
          InstAcc = 0;
        }
		
		float dt1 = dt;

        // Integrators
		Delta = InstAcc * dt1 + (Kp1 * dt1) * AltError;
		
        if (EstVelocity != 0.0) {
          EstAlt += (EstVelocity/5.0 + Delta) * (dt1 / 2) + (Kp2 * dt1) * AltError;
        } else {
          EstAlt = Delta * (dt1 / 2) + (Kp2 * dt1) * AltError;
        }
 
        EstVelocity += Delta * 10.0;
		     
        return EstAlt;
    }
};

#endif // __FILTER_ALTCOMP_H__
