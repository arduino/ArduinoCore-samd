/* 
 * File:   Quaternion.h
 * Author: matt
 *
 * Created on 04 April 2013, 15:12
 */

#ifndef QUATERNION_H
#define	QUATERNION_H

#include "math.h"

class QuaternionClass
{
public:
    QuaternionClass();
    QuaternionClass(const QuaternionClass& orig);
    virtual ~QuaternionClass();
    
    void normalise();
    
    double w, x, y, z;
private:

};

#endif	/* QUATERNION_H */

