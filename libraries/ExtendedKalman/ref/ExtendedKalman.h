/* 
 * File:   ExtendedKalman.h
 * Author: matt
 *
 * Created on 14 March 2013, 17:26
 */

#ifndef EXTENDEDKALMAN_H
#define	EXTENDEDKALMAN_H

#include <iostream>

#include "struct_calibratedData.h"
#include "struct_euler.h"
#include "Quaternion.h"
#include "Eigen/Dense"
using Eigen::Matrix;

#define pi 3.14159265358979

class ExtendedKalmanClass
{
public:
    ExtendedKalmanClass();
    ExtendedKalmanClass(const ExtendedKalmanClass& orig);
    virtual ~ExtendedKalmanClass();
    
    QuaternionClass predict(s_calibratedData* calibratedData, float dt);
    QuaternionClass update(s_calibratedData* calibratedData, float dt);
private:
    double q0, q1, q2, q3;
    double wx, wy, wz;
    double wxb, wyb, wzb;
    Matrix<double,7,7> Q;
    Matrix<double, 7, 1> x;
    Matrix<double, 7, 7> F;
    Matrix<double, 7, 7> P;
    Matrix<double, 3, 1> z;
    Matrix<double, 3, 1> h;
    Matrix<double, 3, 1> y;
    Matrix<double, 3, 7> H;
    Matrix<double, 3, 3> S;
    Matrix<double, 3, 3> R;
    Matrix<double, 7, 3> K;
    Matrix<double, 7, 7> I;
    double norm;

};

#endif	/* EXTENDEDKALMAN_H */

