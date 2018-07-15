// I2Cdev library collection - AK8963 I2C device class header file
// Based on AKM AK8963/B datasheet, 12/2009
// 8/27/2011 by Jeff Rowberg <jeff@rowberg.net> modified by Merlin 6-14-14 for 8963
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2011-08-27 - initial release
//	   2014-06-14 - modified for AK8963 based on Kris Wieners MPU-9250 Sketch
/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _AK8963_H_
#define _AK8963_H_

#include "Arduino.h"
#include "I2Cdev.h"

#define AK8963_ADDRESS_00         0x0C
#define AK8963_ADDRESS_01         0x0D
#define AK8963_ADDRESS_10         0x0E // default for InvenSense MPU-6050 evaluation board
#define AK8963_ADDRESS_11         0x0F
#define AK8963_DEFAULT_ADDRESS    AK8963_ADDRESS_00

#define AK8963_RA_WIA             0x00
#define AK8963_RA_INFO            0x01
#define AK8963_RA_ST1             0x02
#define AK8963_RA_HXL             0x03
#define AK8963_RA_HXH             0x04
#define AK8963_RA_HYL             0x05
#define AK8963_RA_HYH             0x06
#define AK8963_RA_HZL             0x07
#define AK8963_RA_HZH             0x08
#define AK8963_RA_ST2             0x09
#define AK8963_RA_CNTL1			  0x0A
#define AK8963_RA_CNTL2           0x0B 
#define AK8963_RA_ASTC            0x0C
#define AK8963_RA_TS1             0x0D // SHIPMENT TEST, DO NOT USE
#define AK8963_RA_TS2             0x0E // SHIPMENT TEST, DO NOT USE
#define AK8963_RA_I2CDIS          0x0F
#define AK8963_RA_ASAX            0x10
#define AK8963_RA_ASAY            0x11
#define AK8963_RA_ASAZ            0x12

#define AK8963_ST1_DRDY_BIT       0		//Data ready bit
#define AK8963_ST1_DOR_BIT		  1		//Data overrun bit
#define AK8963_ST2_HOFL_BIT       3		//Mag sensor overflow
//#define AK8963_ST2_DERR_BIT       2
#define AK8963_ST2_BITM			  3		//Output bit, 0 = 14 bit, 1 = 16 bit

#define AK8963_CNTL1_MODE_BIT     3
#define AK8963_CNTL1_MODE_LENGTH  4
#define AK8963_MODE_POWERDOWN     0x0
#define AK8963_MODE_SINGLE        0x1
#define AK8963_MODE_CONT1		  0x2	//Continuous mode 1
#define AK8963_MODE_CONT2		  0x6	//Continuous mode 2
#define AK8963_MODE_EXT			  0x4	//External Trigger mode 1
#define AK8963_MODE_SELFTEST      0x8
#define AK8963_MODE_FUSEROM       0xF

#define AK8963_CNTL1_OUT_BIT	  4
#define AK8963_CNTL1_OUT_LENGTH	  1		//Output bit, 0 = 14 bit, 1 = 16 bit

#define AK8963_CNTL2_RESET		  0x01
			  
#define AK8963_ASTC_SELF_BIT      6

#define AK8963_I2CDIS_BIT         0

class AK8963 {
    public:
        AK8963();
        AK8963(bool useSPI, uint8_t address);
        
        void initialize();
        bool testConnection();

        // WIA register
        uint8_t getDeviceID();
        
        // INFO register
        uint8_t getInfo();
        
        // ST1 register
        bool getDataReady();
		bool getDataOverRun();
        
        // H* registers
        void getHeading(int16_t *x, int16_t *y, int16_t *z);
        
        // ST2 register
        bool getOverflowStatus();
		bool getOutBitST();

        // CNTL register
        uint8_t getMode();
        void setModeRes(uint8_t mode, uint8_t Mscale);
        void reset();
        
        // ASTC register
        void setSelfTest(bool enabled);
		void getSelfTest(int16_t *x, int16_t *y, int16_t *z);
        
        // I2CDIS
        void disableI2C(); // um, why...?
        
        // ASA* registers
        void getAdjustment(uint8_t *x, uint8_t *y, uint8_t *z);


    private:
		bool	bSPI;	
        uint8_t devAddr;
        uint8_t buffer[14];
        uint8_t mode;
		uint8_t Mscale;
};

#endif /* _AK8963_H_ */