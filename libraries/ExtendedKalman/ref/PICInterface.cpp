/* 
 * File:   PICInterface.cpp
 * Author: matt
 * 
 * Created on 28 October 2012, 12:53
 */

#include "PICInterface.h"

#define RX_MAX 20000
#define RX_MIN 10000
#define PITCH_RANGE 60.0F
#define ROLL_RANGE 60.0F
#define PITCH_RATE_RANGE 180.0F
#define ROLL_RATE_RANGE 180.0F
#define YAW_RATE_RANGE 100.0F

PICInterfaceClass PICInterface;

PICInterfaceClass::PICInterfaceClass() {

}

PICInterfaceClass::~PICInterfaceClass() {

}

void PICInterfaceClass::setPWM() {
    uint8_t widthsChar[13]; //+1 for pwm_fire bit
    make8_(&pwmwidths.frontright, &widthsChar[0]);
    make8_(&pwmwidths.rearright, &widthsChar[2]);
    make8_(&pwmwidths.rearleft, &widthsChar[4]);
    make8_(&pwmwidths.frontleft, &widthsChar[6]);
    make8_(&pwmwidths.aux1, &widthsChar[8]);
    make8_(&pwmwidths.aux2, &widthsChar[10]);

    I2CInterface.writeRegister(PIC_ADDRESS, REG_PWM1H, widthsChar, 13);
}

void PICInterfaceClass::getRX() {
    uint8_t widthsChar[12] = {0};

    //Appears to generate read errors above ~5 bytes, returning only 1's
    //	I2CInterface.readRegister(PIC_ADDRESS, REG_RX1H, widthsChar, sizeof(widthsChar));
    I2CInterface.readRegister(PIC_ADDRESS, REG_RX1H, &widthsChar[0], 2);
    I2CInterface.readRegister(PIC_ADDRESS, REG_RX2H, &widthsChar[2], 2);
    I2CInterface.readRegister(PIC_ADDRESS, REG_RX3H, &widthsChar[4], 2);
    I2CInterface.readRegister(PIC_ADDRESS, REG_RX4H, &widthsChar[6], 2);
    I2CInterface.readRegister(PIC_ADDRESS, REG_RX5H, &widthsChar[8], 2);
    I2CInterface.readRegister(PIC_ADDRESS, REG_RX6H, &widthsChar[10], 2);
    rxWidths.roll = make16_(widthsChar[0], widthsChar[1]);
    rxWidths.pitch = make16_(widthsChar[2], widthsChar[3]);
    rxWidths.throttle = make16_(widthsChar[4], widthsChar[5]);
    rxWidths.yaw = make16_(widthsChar[6], widthsChar[7]);
    rxWidths.sw1 = make16_(widthsChar[8], widthsChar[9]);
    rxWidths.sw2 = make16_(widthsChar[10], widthsChar[11]);

    static int i = 0;
    rxWidthsHist[i] = rxWidths;
    i++;
    if(i == FILTER_LEN) {
	i = 0;
    }

    rxWidths = averageRX_(rxWidthsHist, FILTER_LEN, i);

    calibrateRX_();
}

inline void PICInterfaceClass::calibrateRX_() {
    rx.pitchDem = PITCH_RANGE * (static_cast<float> (rxWidths.pitch - ((RX_MAX - RX_MIN) / 2) - RX_MIN) / (RX_MAX - RX_MIN));
    rx.rollDem = ROLL_RANGE * (static_cast<float> (rxWidths.roll - ((RX_MAX - RX_MIN) / 2) - RX_MIN) / (RX_MAX - RX_MIN));
    rx.yawRateDem = YAW_RATE_RANGE * (static_cast<float> (rxWidths.yaw - ((RX_MAX - RX_MIN) / 2) - RX_MIN) / (RX_MAX - RX_MIN));
    rx.throttleDem = static_cast<float> (rxWidths.throttle - RX_MIN) / (RX_MAX - RX_MIN);
    rx.sw1 = (rxWidths.sw1 > 15000);
    rx.sw2 = (rxWidths.sw2 > 15000);
    rx.pitchRateDem = PITCH_RATE_RANGE * (static_cast<float> (rxWidths.pitch - ((RX_MAX - RX_MIN) / 2) - RX_MIN) / (RX_MAX - RX_MIN));;
    rx.rollRateDem = ROLL_RATE_RANGE * (static_cast<float> (rxWidths.roll - ((RX_MAX - RX_MIN) / 2) - RX_MIN) / (RX_MAX - RX_MIN));
    

    rx.yawRateDem = -rx.yawRateDem;
}

inline uint16_t PICInterfaceClass::make16_(uint8_t H, uint8_t L) {
    return((static_cast<uint16_t> (H) << 8) | static_cast<uint16_t> (L));
}

inline void PICInterfaceClass::make8_(uint16_t *tosplit, uint8_t *target) {
    *target = static_cast<uint8_t> ((*tosplit >> 8) & 0xFF);
    *(target + 1) = static_cast<uint8_t> (*tosplit & 0xFF);
}

inline s_rxWidths PICInterfaceClass::averageRX_(s_rxWidths rxinput[], int len, int i) {
    unsigned int pitch = 0;
    unsigned int roll = 0;
    unsigned int throttle = 0;
    unsigned int yaw = 0;
    for(int k = 0; k < len; k++) {
	pitch += rxinput[k].pitch;
	roll += rxinput[k].roll;
	throttle += rxinput[k].throttle;
	yaw += rxinput[k].yaw;
    }
    s_rxWidths result;
    result.pitch = pitch / len;
    result.roll = roll / len;
    result.throttle = throttle / len;
    result.yaw = yaw / len;
    result.sw1 = rxinput[i-1].sw1;
    result.sw2 = rxinput[i-1].sw2;
    return result;
}