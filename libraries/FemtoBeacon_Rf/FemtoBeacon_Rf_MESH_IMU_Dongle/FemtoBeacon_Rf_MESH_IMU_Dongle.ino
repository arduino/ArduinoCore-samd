/**
 * FemtoBeacon wirless IMU and LPS platform.
 * Mesh networked IMU demo.
 *
 * @author A. Alibno <aalbino@femtoduino.com>
 * @version 1.0.0
 */

#include <stdio.h>

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define Serial SERIAL_PORT_USBVIRTUAL

/** BEGIN Atmel's LightWeight Mesh stack. **/
    #include "lwm.h"
    #include "lwm/sys/sys.h"
    #include "lwm/nwk/nwk.h"
/** END Atmel's LightWeight Mesh stack. **/

/** BEGIN mjs513/FreeIMU-Updates library. **/
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
/** END mjs513/FreeIMU-Updates library. **/

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
    #define APP_ADDRESS         2
    #define APP_ENDPOINT        1
    #define APP_PANID                 0x4567
    #define APP_SECURITY_KEY          "TestSecurityKey0"
    
    static char                 bufferData[APP_BUFFER_SIZE];
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
/** END Sensor vars **/

byte delimeter = (byte) '|';
byte filler = (byte) ' ';

void setup() {
  // put your setup code here, to run once:
  setupSerialComms();
  Serial.print("Starting LwMesh...");
  setupMeshNetworking();
  Serial.println("OK.");
  delay(500);
  
}

void setupSerialComms() {
    while(!Serial);
    
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
    NWK_SetPanId(0x01);
    PHY_SetChannel(0x1a);
    PHY_SetRxState(true);
    NWK_OpenEndpoint(1, receiveMessage);
}

void loop() {
  
  handleNetworking();
  
  Serial.println("----");
  
  delay(1000);
}

void handleNetworking()
{
    SYS_TaskHandler();
    
    if(APP_ADDRESS == 1) {
      Serial.println("handleNetworking() ->sendMessage()");
        sendMessage();
    }
    Serial.println("handleNetworking()");
}

void handleSensors()
{
    //sprintf (bufferData, "\r\nLIGHT LAMP AT ADDRESS %d!\r\n", APP_ADDRESS);
    Serial.println("handleSensors()");
    sensors.getYawPitchRoll(ypr);
}


static void sendMessage(void) {

  if (send_message_busy) {
    return;
  }
  //pingCounter++;
  //char sensorData[5] = "    ";
  //byte i = 0;

  //if (ypr[0] != NULL)
  //{
  //  dtostrf(ypr[0], 1, 1, sensorData);
  //}

  // we just leak for now
  //NWK_DataReq_t *message = (NWK_DataReq_t*)malloc(sizeof(NWK_DataReq_t));

  Serial.println("sendMessage()");
  sendRequest.dstAddr       = 2;//1 - APP_ADDRESS;
  sendRequest.dstEndpoint   = 1;//APP_ENDPOINT;
  sendRequest.srcEndpoint   = 1;//APP_ENDPOINT;
  //sendRequest.options       = NWK_OPT_ACK_REQUEST;
  sendRequest.data          = (uint8_t*)&bufferData;
  sendRequest.size          = strlen(bufferData);
  sendRequest.confirm       = sendMessageConfirm;
  
  NWK_DataReq(&sendRequest);

  send_message_busy = true;
}

static void sendMessageConfirm(NWK_DataReq_t *req)
{
  Serial.print("sendMessageConfirm() req->status is ");
  if (NWK_NO_ACK_STATUS == req->status)
  {
    Serial.println("NWK_NO_ACK_STATUS");
  } else if (NWK_NO_ROUTE_STATUS == req->status) {
    Serial.println("NWK_NO_ROUTE_STATUS");
  } else if (NWK_ERROR_STATUS) {
    Serial.println("NWK_ERROR_STATUS");
  }
  
  if (NWK_SUCCESS_STATUS == req->status)
  {
    send_message_busy = false;
    Serial.println("NWK_SUCCESS_STATUS");
  }
  (void) req;
}

static bool receiveMessage(NWK_DataInd_t *ind) {

    Serial.print("receiveMessage() ");
    Serial.print("lqi: ");
    Serial.print(ind->lqi, DEC);

    Serial.print("  ");

    Serial.print("rssi: ");
    Serial.print(ind->rssi, DEC);
    Serial.print("  ");

    Serial.print("data: ");

    //bufferData = (char) &ind->data;
    //Serial.println(bufferData);
    //memcpy();
    
    String str((char*)ind->data);

    Serial.println(str);

    //pingCounter = (byte)*(ind->data);
    //Serial.println(pingCounter);

    //Serial.print("my Yaw:");
    //Serial.print(ypr[0]);
    //Serial.print(", Pitch:");
    //Serial.print(ypr[1]);
    //Serial.print(", Roll:");
    //Serial.println(ypr[2]);
    
    return true;
}
