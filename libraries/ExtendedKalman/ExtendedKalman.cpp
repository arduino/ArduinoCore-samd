/* 
 * File:   ExtendedKalman.cpp
 * Author: matt
 * 
 * Created on 14 March 2013, 17:26
 */

#include "ExtendedKalman.h"

ExtendedKalmanClass::ExtendedKalmanClass() {
    Q.Zero();
    x.Zero();
    F.Zero();
    P.Zero();
    z.Zero();
    h.Zero();
    y.Zero();
    H.Zero();
    S.Zero();
    R.Zero();
    K.Zero();
    
    I = I.Identity();
    P = P.Identity();
    P *= 10000;
    
    Q(0,0) = 0;
    Q(1,1) = 0;
    Q(2,2) = 0;
    Q(3,3) = 0;
    Q(4,4) = 0.1;
    Q(5,5) = 0.1;
    Q(6,6) = 0.1;
    
    R(0,0) = 320;
    R(1,1) = 320;
    R(2,2) = 320;
    
    x(0) = 1;
}

ExtendedKalmanClass::ExtendedKalmanClass(const ExtendedKalmanClass& orig) {
}

ExtendedKalmanClass::~ExtendedKalmanClass() {
}

QuaternionClass ExtendedKalmanClass::predict(s_calibratedData* calibratedData, float dt) {    
    q0 = x(0);
    q1 = x(1);
    q2 = x(2);
    q3 = x(3);
    wxb = x(4);
    wyb = x(5);
    wzb = x(6);
    wx = -calibratedData->p * (pi / 180);
    wy = calibratedData->q * (pi / 180);
    wz = calibratedData->r * (pi / 180);
    
    //Half dt since dt is only ever used in this form, saves on evaluating dt/2 multiple times
    dt /= 2;
    
    //Predicted state estimate, x = f(x,u)
    x(0) = q0 + dt * (-q1*(wx-wxb) - q2*(wy-wyb) - q3*(wz-wzb));
    x(1) = q1 + dt * ( q0*(wx-wxb) - q3*(wy-wyb) + q2*(wz-wzb));
    x(2) = q2 + dt * ( q3*(wx-wxb) + q0*(wy-wyb) - q1*(wz-wzb));
    x(3) = q3 + dt * (-q2*(wx-wxb) + q1*(wy-wyb) + q0*(wz-wzb));
    //Gyro biases don't change
    
    //Normalise to unit quaternion
    norm = sqrt(x(0)*x(0) + x(1)*x(1) + x(2)*x(2) + x(3)*x(3));
    x(0) /= norm;
    x(1) /= norm;
    x(2) /= norm;
    x(3) /= norm;
    
    q0 = x(0);
    q1 = x(1);
    q2 = x(2);
    q3 = x(3);
    
    //Build jacobian of f(x,u), F(row,column)
    F(0,0) = 1;
    F(0,1) = -dt*(wx-wxb);
    F(0,2) = -dt*(wy-wyb);
    F(0,3) = -dt*(wz-wzb);
    F(0,4) =  dt*q1;
    F(0,5) =  dt*q2;
    F(0,6) =  dt*q3;
    F(1,0) =  dt*(wx-wxb);
    F(1,1) = 1;
    F(1,2) =  dt*(wz-wzb);
    F(1,3) = -dt*(wy-wyb);
    F(1,4) = -dt*q0;
    F(1,5) =  dt*q3;
    F(1,6) = -dt*q2;
    F(2,0) =  dt*(wy-wyb);
    F(2,1) = -dt*(wz-wzb);
    F(2,2) = 1;
    F(2,3) =  dt*(wx-wxb);
    F(2,4) = -dt*q3;
    F(2,5) = -dt*q0;
    F(2,6) =  dt*q1;
    F(3,0) =  dt*(wz-wzb);
    F(3,1) =  dt*(wy-wyb);
    F(3,2) = -dt*(wx-wxb);
    F(3,3) = 1;
    F(3,4) =  dt*q2;
    F(3,5) = -dt*q1;
    F(3,6) = -dt*q0;
    F(4,0) = 0;
    F(4,1) = 0;
    F(4,2) = 0;
    F(4,3) = 0;
    F(4,4) = 1;
    F(4,5) = 0;
    F(4,6) = 0;
    F(5,0) = 0;
    F(5,1) = 0;
    F(5,2) = 0;
    F(5,3) = 0;
    F(5,4) = 0;
    F(5,5) = 1;
    F(5,6) = 0;
    F(6,0) = 0;
    F(6,1) = 0;
    F(6,2) = 0;
    F(6,3) = 0;
    F(6,4) = 0;
    F(6,5) = 0;
    F(6,6) = 1;
    
    //Covariance of estimate
    P = F*P*F.transpose() + Q;
    
    QuaternionClass q;
    q.w = x(0);
    q.x = x(1);
    q.y = x(2);
    q.z = x(3);
    return q;
}

QuaternionClass ExtendedKalmanClass::update(s_calibratedData* calibratedData, float dt) {
    
    q0 = x(0);
    q1 = x(1);
    q2 = x(2);
    q3 = x(3);
    
    //Normalise accelerometer triad
    norm = sqrt(calibratedData->x*calibratedData->x + calibratedData->y*calibratedData->y + calibratedData->z*calibratedData->z);
    z(0) = calibratedData->x / norm;
    z(1) = calibratedData->y / norm;
    z(2) = -calibratedData->z / norm;
    
    //Map measurements to states
    h(0) = 2*q0*q2 - 2*q1*q3;
    h(1) = -2*q0*q1 - 2*q2*q3;
    h(2) = -q0*q0 + q1*q1 + q2*q2 - q3*q3;
    
    //Measurement residual
    y = z - h;
    
    //Populate h jacobian
    H(0,0) = 2*q2;
    H(0,1) = -2*q3;
    H(0,2) = 2*q0;
    H(0,3) = -2*q1;
    H(0,4) = 0;
    H(0,5) = 0;
    H(0,6) = 0;
    H(1,0) = -2*q1;
    H(1,1) = -2*q0;
    H(1,2) = -2*q3;
    H(1,3) = -2*q2;
    H(1,4) = 0;
    H(1,5) = 0;
    H(1,6) = 0;
    H(2,0) = -2*q0;
    H(2,1) = 2*q1;
    H(2,2) = 2*q2;
    H(2,3) = -2*q3;
    H(2,4) = 0;
    H(2,5) = 0;
    H(2,6) = 0;
    
    //Measurement covariance update + Kalman gain calculation + corrected prediction
    //S = H*P*H' + R
    S = H*P*H.transpose() + R; //H*P*H' is evaluating to 0 for some reason
    
    //K = P*H'/S
    K = P*H.transpose()*S.inverse();
    
    //Correct state estimate
    x = x + K*y;
    
    //Normalise to unit quaternion
    norm = sqrt(x(0)*x(0) + x(1)*x(1) + x(2)*x(2) + x(3)*x(3));
    x(0) /= norm;
    x(1) /= norm;
    x(2) /= norm;
    x(3) /= norm;    
    
    //Update state covariance
    P = (I - K*H)*P;
    
    QuaternionClass q;
    q.w = x(0);
    q.x = x(1);
    q.y = x(2);
    q.z = x(3);
    return q;
}