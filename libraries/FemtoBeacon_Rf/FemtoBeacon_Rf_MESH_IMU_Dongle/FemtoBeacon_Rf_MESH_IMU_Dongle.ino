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
    #define APP_ADDRESS         1
    #define DEST_ADDRESS        1
    #define APP_ENDPOINT        1
    #define APP_PANID                 0x01
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
    float ypr[3]; // yaw pitch roll
    
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

void loop() {
  
  handleNetworking();
  
  //Serial.println("----");
  
  //delay(1000);
}

void handleNetworking()
{
    SYS_TaskHandler();
    
    //Serial.print("Node #");
    //Serial.print(APP_ADDRESS);
    //Serial.println(" handleNetworking()");
}

static bool receiveMessage(NWK_DataInd_t *ind) {
    Serial.print("Node #");
    Serial.print(APP_ADDRESS);
    Serial.print(" receiveMessage() from Node #");
    Serial.print(ind->srcAddr);
    Serial.print(" = lqi: ");
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
