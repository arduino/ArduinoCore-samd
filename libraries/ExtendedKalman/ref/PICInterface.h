/* 
 * File:   PICInterface.h
 * Author: matt
 *
 * Created on 28 October 2012, 12:53
 */

#ifndef PICINTERFACE_H
#define	PICINTERFACE_H

#include <iostream>

#include "I2CInterface.h"

const uint8_t PIC_ADDRESS = 0x50;

//Simulated register names and addresses
const uint8_t REG_PWM1H = 0x00;
const uint8_t REG_PWM1L = 0x01;
const uint8_t REG_PWM2H = 0x02;
const uint8_t REG_PWM2L = 0x03;
const uint8_t REG_PWM3H = 0x04;
const uint8_t REG_PWM3L = 0x05;
const uint8_t REG_PWM4H = 0x06;
const uint8_t REG_PWM4L = 0x07;
const uint8_t REG_PWM5H = 0x08;
const uint8_t REG_PWM5L = 0x09;
const uint8_t REG_PWM6H = 0x0a;
const uint8_t REG_PWM6L = 0x0b;
const uint8_t REG_PWMFIRE = 0x0c;
const uint8_t REG_RX1H = 0x0d;
const uint8_t REG_RX1L = 0x0e;
const uint8_t REG_RX2H = 0x0f;
const uint8_t REG_RX2L = 0x10;
const uint8_t REG_RX3H = 0x11;
const uint8_t REG_RX3L = 0x12;
const uint8_t REG_RX4H = 0x13;
const uint8_t REG_RX4L = 0x14;
const uint8_t REG_RX5H = 0x15;
const uint8_t REG_RX5L = 0x16;
const uint8_t REG_RX6H = 0x17;
const uint8_t REG_RX6L = 0x18;

struct s_rxWidths
{
    uint16_t roll;
    uint16_t pitch;
    uint16_t throttle;
    uint16_t yaw;
    uint16_t sw1;
    uint16_t sw2;
};

struct s_rxcalibrated
{
    float pitchDem;
    float rollDem;
    float throttleDem;
    float yawRateDem;
    float pitchRateDem;
    float rollRateDem;
    bool sw1;
    bool sw2;
};

struct s_pwmwidths
{
    uint16_t frontright;
    uint16_t rearright;
    uint16_t rearleft;
    uint16_t frontleft;
    uint16_t aux1;
    uint16_t aux2;
};

class PICInterfaceClass
{
public:
    PICInterfaceClass();
    virtual ~PICInterfaceClass();

    void setPWM();
    void getRX();

    s_rxWidths rxWidths;
#define FILTER_LEN 10
    s_rxWidths rxWidthsHist[FILTER_LEN];
    s_pwmwidths pwmwidths;
    s_rxcalibrated rx;
    s_rxcalibrated rxfiltered;

private:
    uint16_t make16_(uint8_t H, uint8_t L);
    void make8_(uint16_t *tosplit, uint8_t *target);
    void calibrateRX_();
    s_rxWidths averageRX_(s_rxWidths rxinput[], int len, int i);

};

extern PICInterfaceClass PICInterface;

#endif	/* PICINTERFACE_H */

