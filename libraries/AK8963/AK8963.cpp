// I2Cdev library collection - AK8963 I2C device class header file
// Based on AKM AK8963/B datasheet, 12/2009
// 8/27/2011 by Jeff Rowberg <jeff@rowberg.net> for 8975
// 6/14/2014 modified for AKM AK8963
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib for 8975
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

#include "AK8963.h"


/** Default constructor, uses default I2C address.
 * @see AK8963_DEFAULT_ADDRESS
 */
AK8963::AK8963() {
    devAddr = AK8963_DEFAULT_ADDRESS;
}

/** Specific address constructor.
 * @param address I2C address
 * @see AK8963_DEFAULT_ADDRESS
 * @see AK8963_ADDRESS_00
 */
AK8963::AK8963(bool useSPI, uint8_t address) {
	bSPI = useSPI;
    devAddr = address;
}

/** Power on and prepare for general usage.
 * No specific pre-configuration is necessary for this device.
 */
void AK8963::initialize() {

}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool AK8963::testConnection() {
    if (I2Cdev::readByte(bSPI, devAddr, AK8963_RA_WIA, buffer) == 1) {
        return (buffer[0] == 0x48);
    }
    return false;
}

// WIA register

uint8_t AK8963::getDeviceID() {
    I2Cdev::readByte(bSPI, devAddr, AK8963_RA_WIA, buffer);
    return buffer[0];
}

// INFO register

uint8_t AK8963::getInfo() {
    I2Cdev::readByte(bSPI, devAddr, AK8963_RA_INFO, buffer);
    return buffer[0];
}

// ST1 register

bool AK8963::getDataReady() {
    I2Cdev::readBit(bSPI, devAddr, AK8963_RA_ST1, AK8963_ST1_DRDY_BIT, buffer);
    return buffer[0];
}

bool AK8963::getDataOverRun() {
    I2Cdev::readBit(bSPI, devAddr, AK8963_RA_ST1, AK8963_ST1_DOR_BIT, buffer);
    return buffer[0];
}

// H* registers
void AK8963::getHeading(int16_t *x, int16_t *y, int16_t *z) {
    //I2Cdev::writeByte(bSPI, devAddr, AK8963_RA_CNTL1, AK8963_MODE_SINGLE);
    //delay(10);
	
	if (getDataReady() & 0x01) {
	    I2Cdev::readBytes(bSPI, devAddr, AK8963_RA_HXL, 7, buffer);
		uint8_t c = buffer[6]; // End data read by reading ST2 register
		if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
			*x = (((int16_t)buffer[1]) << 8) | buffer[0];
			*y = (((int16_t)buffer[3]) << 8) | buffer[2];
			*z = (((int16_t)buffer[5]) << 8) | buffer[4];
		}
	}	
}

void AK8963::getSelfTest(int16_t *x, int16_t *y, int16_t *z) {
 	I2Cdev::writeByte(bSPI, devAddr, AK8963_RA_CNTL1, AK8963_MODE_POWERDOWN); // Power down magnetometer  
	delay(10);   
	setSelfTest(true);		// Set Self Test Bit
	delay(10); 	
	// Read mag registers
	if(getDataReady() & 0x01) {
	    I2Cdev::readBytes(bSPI, devAddr, AK8963_RA_HXL, 7, buffer);
		uint8_t c = buffer[6]; // End data read by reading ST2 register
		if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
			*x = (((int16_t)buffer[1]) << 8) | buffer[0];
			*y = (((int16_t)buffer[3]) << 8) | buffer[2];
			*z = (((int16_t)buffer[5]) << 8) | buffer[4];
		}
	}
	setSelfTest(false);		// Set Self Test Bit
	delay(10); 	 	
	I2Cdev::writeByte(bSPI, devAddr, AK8963_RA_CNTL1, AK8963_MODE_POWERDOWN); // Power down magnetometer  
	delay(10);   
	
}

// ST2 register
bool AK8963::getOverflowStatus() {
    I2Cdev::readBit(bSPI, devAddr, AK8963_RA_ST2, AK8963_ST2_HOFL_BIT, buffer);
    return buffer[0];
}

bool AK8963::getOutBitST() {
    I2Cdev::readBit(bSPI, devAddr, AK8963_RA_ST2, AK8963_ST2_BITM, buffer);
    return buffer[0];
}

// CNTL1 register
uint8_t AK8963::getMode() {
    //I2Cdev::readBits(bSPI, devAddr, AK8963_RA_CNTL1, AK8963_CNTL1_MODE_BIT, AK8963_CNTL1_MODE_LENGTH, buffer);
	I2Cdev::readByte(bSPI, devAddr, AK8963_RA_CNTL1, buffer);
    return buffer[0];
}

//Configure magenetometer mode and resolution
void AK8963::setModeRes(uint8_t mode, uint8_t Mscale) {
    //I2Cdev::writeBits(bSPI, devAddr, AK8963_RA_CNTL1, AK8963_CNTL1_MODE_BIT, AK8963_CNTL1_MODE_LENGTH, mode);
	I2Cdev::writeByte(bSPI, devAddr, AK8963_RA_CNTL1, Mscale << 4 | mode);
	delay(10);
}

// CNTL2 register
void AK8963::reset() {
    I2Cdev::writeBytes(bSPI, devAddr, AK8963_RA_CNTL2, AK8963_CNTL2_RESET, buffer);
}

// ASTC register
void AK8963::setSelfTest(bool enabled) {
    I2Cdev::writeBit(bSPI, devAddr, AK8963_RA_ASTC, AK8963_ASTC_SELF_BIT, enabled);
}

// I2CDIS
void AK8963::disableI2C() {
    I2Cdev::writeBit(bSPI, devAddr, AK8963_RA_I2CDIS, AK8963_I2CDIS_BIT, true);
}

// ASA* registers
void AK8963::getAdjustment(uint8_t *x, uint8_t *y, uint8_t *z) {
	I2Cdev::writeByte(bSPI, devAddr, AK8963_RA_CNTL1, AK8963_MODE_POWERDOWN); // Power down magnetometer  
	delay(10);
	I2Cdev::writeByte(bSPI, devAddr, AK8963_RA_CNTL1, AK8963_MODE_FUSEROM); // Enter Fuse ROM access mode
	delay(10);
    
	I2Cdev::readBytes(bSPI, devAddr, AK8963_RA_ASAX, 3, buffer);	 // Read the x-, y-, and z-axis calibration values
    *x = buffer[0];
    *y = buffer[1];
    *z = buffer[2];
	
	I2Cdev::writeByte(bSPI, devAddr, AK8963_RA_CNTL1, AK8963_MODE_POWERDOWN); // Power down magnetometer  
	delay(10);	
	

}
