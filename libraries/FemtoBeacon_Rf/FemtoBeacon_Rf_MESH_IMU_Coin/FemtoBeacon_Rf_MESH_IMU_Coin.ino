/**
 * FemtoBeacon wirless IMU and LPS platform.
 * Mesh networked IMU demo.
 *
 * @author A. Alibno <aalbino@femtoduino.com>
 * @version 1.0.1
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>


#define Serial SERIAL_PORT_USBVIRTUAL

/** BEGIN Atmel's LightWeight Mesh stack. **/
    #include "lwm.h"
    #include "lwm/sys/sys.h"
    #include "lwm/nwk/nwk.h"
/** END Atmel's LightWeight Mesh stack. **/

/** BEGIN mjs513 fork https://github.com/femtoduino/FreeIMU-Updates library. **/
    #include "calibration.h" // Uncomment once you have calibrated your IMU, generated a calibration.h file and updated FreeIMU.h!
    
    //These are optional depending on your IMU configuration

    //#include <ADXL345.h>
    //#include <HMC58X3.h>
    //#include <LSM303.h>
    //#include <LPS.h> 
    //#include <ITG3200.h> //note LPS library must come before ITG lib
    //#include <bma180.h>
    //#include <MS561101BA.h> //Comment out for APM 2.5
    //#include <BMP085.h>
    #include <I2Cdev.h>
    #include <MPU60X0.h>
    //#include <AK8975.h>
    #include <AK8963.h>
    //#include <L3G.h>
    //#include <SFE_LSM9DS0.h>
    //#include <BaroSensor.h>
    #include <AP_Baro_MS5611.h>  //Uncomment for APM2.5


    //These are mandatory
    #include <AP_Math_freeimu.h>
    #include <Butter.h>    // Butterworth filter
    #include <iCompass.h>
    #include <MovingAvarageFilter.h>

    //#define DEBUG
    #include "DebugUtils.h"
    #include "CommunicationUtils.h"
    //#include "DCM.h"
    #include "FilteringScheme.h"
    #include "RunningAverage.h"
    #include "FreeIMU.h"

    // Arduino Zero: no eeprom 
    #define HAS_EEPPROM 0
/** END mjs513 fork https://github.com/femtoduino/FreeIMU-Updates library. **/

/** BEGIN Networking vars **/
    extern "C" {
      void                      println(char *x) { Serial.println(x); Serial.flush(); }
    }

    #ifdef NWK_ENABLE_SECURITY
    #define APP_BUFFER_SIZE     (NWK_MAX_PAYLOAD_SIZE - NWK_SECURITY_MIC_SIZE)
    #else
    #define APP_BUFFER_SIZE     NWK_MAX_PAYLOAD_SIZE
    #endif

    // Address must be set to 1 for the first device, and to 2 for the second one.
    #define APP_ADDRESS         5 // Each coin (node) should have a unique integer address > 1
    #define DEST_ADDRESS        1 // The RF Dongle node uses Address 1 (this is of course, for simplicity's sake)
    #define APP_ENDPOINT        1 // What callback endpoint number we are using.
    #define APP_PANID                 0x01
    #define APP_SECURITY_KEY          "TestSecurityKey0"
    
    char                        bufferData[APP_BUFFER_SIZE];
    static NWK_DataReq_t        sendRequest;
    static void                 sendMessage(void);
    static void                 sendMessageConfirm(NWK_DataReq_t *req);
    static bool                 receiveMessage(NWK_DataInd_t *ind);

    static bool                 send_message_busy = false;

    byte pingCounter            = 0;
/** END Networking vars **/

/** BEGIN Sensor vars **/
    //int raw_values[11];
    //char str[512];
    float ypr[3]; // yaw pitch roll
    //float val[9];
    
    

    // Set the FreeIMU object
    FreeIMU sensors = FreeIMU();

    // FemtoBeacon FSYNC pin is PA18 (not PA19, which is mislabeled in the silkscreen of FemtoBeacon rev 2.0.0)
    // Must connect to GND if FSYNC is unused.
    byte PIN_FSYNC = 4;

    // FemtoBeacon INT pin is PA19 (not PA18, which is mislabeled in the silkscreen of FemtoBeacon r2.0.0)
    byte PIN_INT = 3;
/** END Sensor vars **/

byte delimeter = (byte) '|';
byte filler = (byte) ' ';

void setup() {

  pinMode(PIN_INT, INPUT);
  pinMode(PIN_FSYNC, OUTPUT);
  digitalWrite(PIN_FSYNC, HIGH);
  delay(10);
  digitalWrite(PIN_FSYNC, LOW);
  
  // put your setup code here, to run once:
  setupSerialComms();
  Serial.print("Starting LwMesh...");
  setupMeshNetworking();
  Serial.println("OK.");
  delay(500);
  Serial.print("Starting Sensors...");
  setupSensors();
  Serial.println("OK.");

  // REQUIRED! calls to dtostrf will otherwise fail (optimized out?)
  char cbuff[7];
  dtostrf(123.4567, 6, 2, cbuff);
  Serial.print("cbuff test is ");
  Serial.println(cbuff);

  Serial.println("OK, ready!");
}

void setupSerialComms() {
    //while(!Serial);
    
    Serial.begin(115200);
    Serial.print("LWP Ping Demo. Serial comms started. ADDRESS is ");
    Serial.println(APP_ADDRESS);
}

void setupMeshNetworking() {
    SPI.usingInterrupt(digitalPinToInterrupt(PIN_SPI_IRQ));
    
    SPI.beginTransaction(
      SPISettings(
          MODULE_AT86RF233_CLOCK, 
          MSBFIRST, 
          SPI_MODE0
      )
    );

    attachInterrupt(digitalPinToInterrupt(PIN_SPI_IRQ), HAL_IrqHandlerSPI, RISING);
    /*  wait for SPI to be ready  */
    delay(10);

    SYS_Init();
    NWK_SetAddr(APP_ADDRESS);
    NWK_SetPanId(APP_PANID);
    PHY_SetChannel(0x1a);
    PHY_SetRxState(true);
    NWK_OpenEndpoint(APP_ENDPOINT, receiveMessage);
}

void setupSensors() {
    Wire.begin();
  
    delay(10);
    sensors.init(); // the parameter enable or disable fast mode
    delay(10);
}

void loop() {
  
  handleSensors();
  handleNetworking();
}

void handleNetworking()
{
    SYS_TaskHandler();
    
    if(APP_ADDRESS > 1) {

      #ifdef DEBUG
      
      Serial.print("Node #");
      Serial.print(APP_ADDRESS);
      Serial.println(" handleNetworking() ->sendMessage()");
      
      #endif
      
      sendMessage();
    }
}

void handleSensors()
{
    #ifdef DEBUG
    
    Serial.println("handleSensors()");
    
    #endif
    
    sensors.getYawPitchRoll(ypr);


    #ifdef DEBUG
    
    Serial.println("...getting YPR string");
    Serial.print("Buffer size is ");
    Serial.println(APP_BUFFER_SIZE);
    
    Serial.print("sensor data is ");
    Serial.print(ypr[0]);
    Serial.print(',');
    Serial.print(ypr[1]);
    Serial.print(',');
    Serial.println(ypr[2]);
    
    #endif

    //// Use dtostrf?
    // Copy ypr to buffer.
    memset(bufferData, ' ', APP_BUFFER_SIZE);
    bufferData[APP_BUFFER_SIZE] = '\0';

    // ...We need a wide enough size (8 should be enough to cover negative symbol and decimal). 
    // ...Precision is 2 decimal places.
    dtostrf(ypr[0], 8, 2, &bufferData[0]);
    dtostrf(ypr[1], 8, 2, &bufferData[9]);
    dtostrf(ypr[2], 8, 2, &bufferData[18]);
    // ...Delimeters
    bufferData[8] = ',';
    bufferData[17] = ',';
    
    Serial.print("TX data: ");
    Serial.println(bufferData);
}


static void sendMessage(void) {

  if (send_message_busy) {
    #ifdef DEBUG
    
    Serial.println("...sendMessage() busy");
    
    #endif
    
    return;
  }

  #ifdef DEBUG
  
  Serial.println("sendMessage()");
  
  #endif
  
  sendRequest.dstAddr       = DEST_ADDRESS;
  sendRequest.dstEndpoint   = APP_ENDPOINT; // Endpoint number on destination device
  sendRequest.srcEndpoint   = APP_ENDPOINT; // Local Endpoint number
  sendRequest.options       = NWK_IND_OPT_BROADCAST_PAN_ID; // Broadcast to the PAN ID group.
  sendRequest.data          = (uint8_t*)&bufferData;
  sendRequest.size          = sizeof(bufferData);
  sendRequest.confirm       = sendMessageConfirm;
  
  NWK_DataReq(&sendRequest);

  send_message_busy = true;
}

static void sendMessageConfirm(NWK_DataReq_t *req)
{
  #ifdef DEBUG
  
  Serial.print("sendMessageConfirm() req->status is ");
  
  if (NWK_NO_ACK_STATUS == req->status)
  {
    Serial.println("NWK_NO_ACK_STATUS");
  } else if (NWK_NO_ROUTE_STATUS == req->status) {
    Serial.println("NWK_NO_ROUTE_STATUS");
  } else if (NWK_ERROR_STATUS) {
    Serial.println("NWK_ERROR_STATUS");
  }
  
  #endif
  send_message_busy = false;
  if (NWK_SUCCESS_STATUS == req->status)
  {
    //send_message_busy = false;
    Serial.println("NWK_SUCCESS_STATUS");
  }
  (void) req;
}

static bool receiveMessage(NWK_DataInd_t *ind) {
    //char sensorData[5];

    Serial.print("receiveMessage() ");
    Serial.print("lqi: ");
    Serial.print(ind->lqi, DEC);

    Serial.print("  ");

    Serial.print("rssi: ");
    Serial.print(ind->rssi, DEC);
    Serial.print("  ");

    Serial.print("data: ");
    
    String str((char*)ind->data);

    Serial.println(str);
    
    return true;
}

/*
 * SEE http://forum.arduino.cc/index.php?topic=368720.0
 * 
  dtostrf - Emulation for dtostrf function from avr-libc
  Copyright (c) 2015 Arduino LLC.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <stdio.h>

#if 0
char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}
#else

char *dtostrf(double val, int width, unsigned int prec, char *sout)
{
  int decpt, sign, reqd, pad;
  const char *s, *e;
  char *p;
  s = fcvtf(val, prec, &decpt, &sign);
  if (prec == 0 && decpt == 0) {
  s = (*s < '5') ? "0" : "1";
    reqd = 1;
  } else {
    reqd = strlen(s);
    if (reqd > decpt) reqd++;
    if (decpt == 0) reqd++;
  }
  if (sign) reqd++;
  p = sout;
  e = p + reqd;
  pad = width - reqd;
  if (pad > 0) {
    e += pad;
    while (pad-- > 0) *p++ = ' ';
  }
  if (sign) *p++ = '-';
  if (decpt <= 0 && prec > 0) {
    *p++ = '0';
    *p++ = '.';
    e++;
    while ( decpt < 0 ) {
      decpt++;
      *p++ = '0';
    }
  }    
  while (p < e) {
    *p++ = *s++;
    if (p == e) break;
    if (--decpt == 0) *p++ = '.';
  }
  if (width < 0) {
    pad = (reqd + width) * -1;
    while (pad-- > 0) *p++ = ' ';
  }
  *p = 0;
  return sout;
}
#endif
