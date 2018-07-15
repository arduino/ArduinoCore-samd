#ifndef FilteringScheme_h
#define FilteringScheme_h

//inspired by interactive-matter

class KalmanFilter {
  public:
    KalmanFilter();
    KalmanFilter(float q, float r, float p, float intial_value);
    void KalmanInit(float q, float r, float p, float intial_value);
    void set_q(float _q);
    void set_r(float _r);
    float q; //process noise covariance
    float r; //measurement noise covariance
    float x; //value
    float p; //estimation error covariance
    float k; //kalman gain
    float intial_value;
    float measureRSSI(float raw_rssi);    
    
};

#endif //FilteringScheme_h
