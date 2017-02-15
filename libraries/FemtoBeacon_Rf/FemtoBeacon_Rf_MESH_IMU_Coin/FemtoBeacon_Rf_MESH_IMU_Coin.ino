/**
 * FemtoBeacon wirless IMU and LPS platform.
 * Mesh networked IMU demo.
 *
 * @author A. Alibno <aalbino@femtoduino.com>
 * @version 1.0.1
 */
/**
 * This sketch assumes the following FreeIMU.h values:
 * 
 *   - MARG should be 4 (DCM)
 *   - MAG_DEC needs to be set to your location's magnetic declination (degrees)
 *   - Calibrate your IMU using the FreeIMU GUI tool (should generate a calibration.h file, include alongside this sketch)
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <Arduino.h>
#include <Wire.h>
#include <avr/dtostrf.h>
#include <SPI.h>


#define Serial SERIAL_PORT_USBVIRTUAL

//#define DEBUG
//#define OUTPUT_SERIAL
    

/** BEGIN Atmel's LightWeight Mesh stack. **/
    #include "lwm.h"
    #include "lwm/sys/sys.h"
    #include "lwm/nwk/nwk.h"
/** END Atmel's LightWeight Mesh stack. **/

/** BEGIN mjs513 fork https://github.com/femtoduino/FreeIMU-Updates library. **/
    //#include "calibration.h" // Uncomment once you have calibrated your IMU, generated a calibration.h file and updated FreeIMU.h!
    
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
    
    #include "DebugUtils.h"
    #include "CommunicationUtils.h"
    #include "DCM.h"
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
    #define APP_ADDRESS         2 // Each coin (node) should have a unique integer address > 1
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

    byte pingCounter            = -2147483648; // Minimum value as a starting point
    long tickCounter            = 0;
/** END Networking vars **/

/** BEGIN Sensor vars **/
    // Sensor reading.
    float ypr[3]; // Hold Yaw-Pitch-Roll (YPR) data.
    float baro; // Hold Barometer Altitude data
    float temp; // Hold Temperature data
    float pressure; // Hold Pressure data
    
    // Set the FreeIMU object
    FreeIMU sensors = FreeIMU();

    // FemtoBeacon FSYNC pin is PA18 (not PA19, which is mislabeled in the silkscreen of FemtoBeacon rev 2.0.0)
    // Must connect to GND if FSYNC is unused.
    byte PIN_FSYNC = 4;

    // FemtoBeacon INT pin is PA19 (not PA18, which is mislabeled in the silkscreen of FemtoBeacon r2.0.0)
    byte PIN_INT = 3;
/** END Sensor vars **/

char delimeter = ',';
char filler = (char) 0;

void setup() {
  /*int wait = 0;
  while (!Serial) {
    wait++;

    if (wait > F_CPU) {
      break;
    }
  }*/
  
  pinMode(PIN_INT, INPUT);
  pinMode(PIN_FSYNC, OUTPUT);
  digitalWrite(PIN_FSYNC, HIGH);
  delay(10);
  digitalWrite(PIN_FSYNC, LOW);
  
  // put your setup code here, to run once:
  #ifdef OUTPUT_SERIAL
  setupSerialComms();
  
  Serial.print("Starting LwMesh...");
  #endif
  setupMeshNetworking();

  #ifdef OUTPUT_SERIAL
  Serial.println("OK.");

  Serial.print("Starting Sensors...");
  #endif
  setupSensors();

  #ifdef OUTPUT_SERIAL
  Serial.println("OK.");
  Serial.print("App buffer size is ");
  Serial.println(APP_BUFFER_SIZE);
  #endif
  // REQUIRED! calls to dtostrf will otherwise fail (optimized out?)
  char cbuff[7];
  dtostrf(123.4567, 6, 2, cbuff);

  #ifdef OUTPUT_SERIAL
  Serial.print("cbuff test is ");
  Serial.println(cbuff);

  Serial.println("OK, ready!");
  #endif
}

void setupSerialComms() {
    //while(!Serial);

    
    Serial.begin(500000);
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

    // Set TX Power for internal at86rf233, default is 0x0 (+4 dbm)
    // TX_PWR  0x0 ( +4   dBm)
    // TX_PWR  0x1 ( +3.7 dBm)
    // TX_PWR  0x2 ( +3.4 dBm)
    // TX_PWR  0x3 ( +3   dBm)
    // TX_PWR  0x4 ( +2.5 dBm)
    // TX_PWR  0x5 ( +2   dBm)
    // TX_PWR  0x6 ( +1   dBm)
    // TX_PWR  0x7 (  0   dBm)
    // TX_PWR  0x8 ( -1   dBm)
    // TX_PWR  0x9 ( -2   dBm)
    // TX_PWR  0xA ( -3   dBm)
    // TX_PWR  0xB ( -4   dBm)
    // TX_PWR  0xC ( -6   dBm)
    // TX_PWR  0xD ( -8   dBm)
    // TX_PWR  0xE (-12   dBm)
    // TX_PwR  0xF (-17   dBm)
    
    // Example:
    PHY_SetTxPower(0x00); // Set to +4 dBm
  
    NWK_SetAddr(APP_ADDRESS);
    NWK_SetPanId(APP_PANID);
    PHY_SetChannel(0x1a);
    PHY_SetRxState(true);
    NWK_OpenEndpoint(APP_ENDPOINT, receiveMessage);
}

void setupSensors() {
    Wire.begin();
  
    delay(10);
    sensors.init(true); // the parameter enable or disable fast mode
    delay(10);
}

void loop() {
  
  digitalWrite(PIN_FSYNC, HIGH);
  digitalWrite(PIN_FSYNC, LOW);
  
  //handleSensors();
  handleNetworking();
}

void handleNetworking()
{
    SYS_TaskHandler();
    
    if(APP_ADDRESS > 1) {

      #ifdef DEBUG
      #ifdef OUTPUT_SERIAL
      Serial.print("Node #");
      Serial.print(APP_ADDRESS);
      Serial.println(" handleNetworking() ->sendMessage()");
      #endif
      #endif

      if (!send_message_busy) {

        // We are reading the sensors only when we can send data.
        // @TODO implement FIFO Stack of sensor data to transmit.
        handleSensors();
        
        sendMessage();
      }
    }
}

void handleSensors()
{
    #ifdef DEBUG
    #ifdef OUTPUT_SERIAL
    Serial.println("handleSensors()");
    #endif
    #endif
    
    sensors.getYawPitchRoll180(ypr);
    baro = sensors.getBaroAlt();
    temp = sensors.getBaroTemperature();
    pressure = sensors.getBaroPressure();
    

    //// Use dtostrf?
    // Copy ypr to buffer.
    resetBuffer();

    // ...We need a wide enough size (8 should be enough to cover negative symbol and decimal). 
    // ...Precision is 2 decimal places.

    // Yaw
    dtostrf(ypr[0], 8, 2, &bufferData[0]);
    bufferData[8] = delimeter;

    // Pitch
    dtostrf(ypr[1], 8, 2, &bufferData[9]);
    bufferData[17] = delimeter;

    // Roll
    dtostrf(ypr[2], 8, 2, &bufferData[18]);
    //bufferData[26] = delimeter;

    /*
    // BaroAlt
    dtostrf(baro, 8, 2, &bufferData[27]);
    bufferData[35] = delimeter;

    // Temperature
    dtostrf(temp, 8, 2, &bufferData[36]);
    bufferData[44] = delimeter;

    // Pressure
    dtostrf(pressure, 8, 2, &bufferData[45]);
    */

    #ifdef OUTPUT_SERIAL
    Serial.print("TX data: ");
    Serial.println(bufferData);
    #endif
}

void resetBuffer() {
  memset(bufferData, filler, APP_BUFFER_SIZE);
  bufferData[APP_BUFFER_SIZE] = '\0';
}


static void sendMessage(void) {

  if (send_message_busy) {
    #ifdef DEBUG
    #ifdef OUTPUT_SERIAL
    Serial.println("...sendMessage() busy");
    #endif
    #endif
    
    return;
  }

  #ifdef DEBUG
  #ifdef OUTPUT_SERIAL
  Serial.println("sendMessage()");
  #endif
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
  #ifdef OUTPUT_SERIAL
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
  #endif
  send_message_busy = false;
  if (NWK_SUCCESS_STATUS == req->status)
  {
    //send_message_busy = false;
    #ifdef OUTPUT_SERIAL
    Serial.println("NWK_SUCCESS_STATUS");
    #endif
  }
  (void) req;
}

static bool receiveMessage(NWK_DataInd_t *ind) {
    //char sensorData[5];
    #ifdef OUTPUT_SERIAL
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
    #endif
    
    return true;
}

