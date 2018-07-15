#ifndef iCompass_h
#define iCompass_h

#include <Arduino.h> // for byte data type
#include "RunningAverage.h"

class iCompass
{
  public:
    iCompass(void);
    iCompass(float dAngle);
    iCompass(float dAngle, unsigned int windSize);
    iCompass(float dAngle, unsigned int windSize, unsigned int maxS);

    //float heading(void);
    float iheading(int ix, int iy, int iz, float ax, float ay, float az, float mx, float my, float mz);
    
  private:
    template <typename T> struct vector
    {
      T x, y, z;
    };

    vector<float> a; // accelerometer readings
    vector<float> m; // magnetometer readings
  
    // vector functions
    template <typename Ta, typename Tb, typename To> static void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);
    template <typename Ta, typename Tb> static float vector_dot(const vector<Ta> *a,const vector<Tb> *b);
    static void vector_normalize(vector<float> *a);
    
    float oldHeading;
    RunningAverage myRA;
    unsigned int maxSamples;
    unsigned int samples;
    float declinationAngle;
    
    //=============================================================
    // converted from Michael Shimniok Data Bus code
    // http://mbed.org/users/shimniok/code/AVC_20110423/

    float clamp360(float x) {
        while ((x) >= 360.0) (x) -= 360.0; 
        while ((x) < 0) (x) += 360.0; 
        return x;
    }

    //==============================================================
    //
    float HeadingAvgCorr(float newx, float oldx) {
        while ((newx + 180.0) < oldx) (newx) += 360.0;
        while ((newx - 180.0) > oldx) (newx) -= 360.0;
        while ((newx) == 360.0) (newx) = 0.0;
        return newx;
    }


    //=======================================
    float iround(float number, float decimal) {
      //int ix;
      //ix = round(number*pow(10, decimal));
      //return float(ix)/pow(10, decimal);
      return float(round(number*pow(10, decimal)))/pow(10, decimal);
    }

    //=======================================
};
#endif



