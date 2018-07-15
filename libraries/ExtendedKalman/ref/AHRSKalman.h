/* 
 * File:   AHRS.h
 * Author: matt
 *
 * Created on 29 October 2012, 22:38
 */

#ifndef AHRSKalman_H
#define	AHRSKalman_H

#include <stlport.h>
#include <Eigen30.h>
#include <iostream>

#include "ExtendedKalman.h"

#include "struct_calibratedData.h"
#include "struct_euler.h"
#include "Quaternion.h"

const double g = 9.81816;

class AHRSKalmanClass
{
public:
    AHRSKalmanClass();
    AHRSKalmanClass(const AHRSKalmanClass& orig);
    virtual ~AHRSKalmanClass();
    void update();
    
    s_calibratedData calibratedData;
    s_euler orientation;
    QuaternionClass quaternion;
private:
    void calibrateData_();
    void fuse_();
    void quaternionToYPR_(QuaternionClass* q, s_euler* orientation);
    double magnitude_(double x, double y, double z);
//    KalmanClass kalmanPhi_, kalmanPsi_;
    ExtendedKalmanClass EKF;
};

extern AHRSKalmanClass AHRS;

#endif	/* AHRS_H */

