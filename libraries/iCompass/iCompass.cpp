#include <iCompass.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////
#define M_PI 3.14159265359


// Constructors ////////////////////////////////////////////////////////////////

iCompass::iCompass(void) : myRA(1) { declinationAngle = 0; maxSamples = 500; float test = 1.0f; samples = 0; myRA.clear(); }
iCompass::iCompass(float dAngle) : myRA(1) { declinationAngle = dAngle; maxSamples = 500; float test = 1.0f; samples = 0; myRA.clear(); }
iCompass::iCompass(float dAngle, unsigned int windSize) : myRA(windSize) { declinationAngle = dAngle; maxSamples = 500; float test = 1.0f; samples = 0; myRA.clear(); }
iCompass::iCompass(float dAngle, unsigned int windSize, unsigned int maxS) : myRA(windSize) { declinationAngle = dAngle; maxSamples = maxS; float test = 1.0f; samples = 0; myRA.clear(); }


/*
Returns the angular difference in the horizontal plane between a
default vector and north, in degrees.

The default vector here is chosen to point along the surface of the
PCB, in the direction of the top of the text on the silkscreen.
This is the +X axis on the Pololu LSM303D carrier and the -Y axis on
the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH carriers.

float iCompass::heading()
{
   return heading((vector<int>){1, 0, 0});  //any north pointing in the x direction

}*/

/*
Returns the angular difference in the horizontal plane between the
"from" vector and north, in degrees.

Description of heading algorithm:
Shift and scale the magnetic reading based on calibration data to find
the North vector. Use the acceleration readings to determine the Up
vector (gravity is measured as an upward acceleration). The cross
product of North and Up vectors is East. The vectors East and North
form a basis for the horizontal plane. The From vector is projected
into the horizontal plane and the angle between the projected vector
and horizontal north is returned.
*/
float iCompass::iheading(int ix, int iy, int iz, float ax, float ay, float az, float mx, float my, float mz)
{
    if (samples == maxSamples)
    {
      samples = 0;
      myRA.clear();
    }
    
	vector<int> from = {ix, iy, iz};
    vector<float> m = {mx, my, mz};
    vector<float> a = {ax, ay, az};	

    // compute E and N
    vector<float> E;
    vector<float> N;
    vector_cross(&m, &a, &E);
    vector_normalize(&E);
    vector_cross(&a, &E, &N);
    vector_normalize(&N);

    // compute heading
    float heading = atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / M_PI;
    if (heading < 0) heading += 360;
    
    if(heading < -9990) {
        heading = 0;
    }
    
    heading = clamp360(iround(heading,1)+declinationAngle);
	
    if(maxSamples > 1) {
		myRA.addValue(heading);
		myRA.addValue(HeadingAvgCorr(heading, oldHeading));
		oldHeading = heading;
    
		samples++;

		return myRA.getAverage();
	} else {
		return heading;
	}
}

template <typename Ta, typename Tb, typename To> void iCompass::vector_cross(const vector<Ta> *a,const vector<Tb> *b, vector<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb> float iCompass::vector_dot(const vector<Ta> *a, const vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

void iCompass::vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

