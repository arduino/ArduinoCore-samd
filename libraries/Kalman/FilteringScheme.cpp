#include "FilteringScheme.h"


KalmanFilter::KalmanFilter() {}
KalmanFilter::KalmanFilter(float _q, float _r, float _p, float _intial_value) {
  q = _q;
  r = _r;
  p = _p;
  intial_value = _intial_value;
}  

void KalmanFilter::KalmanInit(float _q, float _r, float _p, float _intial_value) {
  q = _q;
  r = _r;
  p = _p;
  intial_value = _intial_value;
}  

float KalmanFilter::measureRSSI(float raw_rssi) {
  p = p+q;  
  k = p/(p+r)*1.0;
  x = x+k*(raw_rssi - x);
  p = (1 - k)*p*1.0;
  return x;
}

void KalmanFilter::set_q(float _q) {
  q = _q;
}

void KalmanFilter::set_r(float _r) {
  r = _r;
}
