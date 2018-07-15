/* 
 * File:   Quaternion.cpp
 * Author: matt
 * 
 * Created on 04 April 2013, 15:12
 */

#include "Quaternion.h"

QuaternionClass::QuaternionClass() {
}

QuaternionClass::QuaternionClass(const QuaternionClass& orig) {
}

QuaternionClass::~QuaternionClass() {
}

 void QuaternionClass::normalise(){
    double norm = sqrt(w*w + x*x + y*y + z*z);
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;
}

