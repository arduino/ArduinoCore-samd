/**
 * FemtoBeacon wirless IMU and LPS platform.
 * Mesh networked IMU demo. Uses Atmel's LWM library (ported for use w/ Arduino)
 *
 * @author A. Alibno <aalbino@femtoduino.com>
 * @version 1.0.1
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
    #define APP_PANID           0x01
    #define APP_SECURITY_KEY    "TestSecurityKey0"
    #define APP_CHANNEL         0x1a
    
    static char                 bufferData[APP_BUFFER_SIZE];
    static NWK_DataReq_t        sendRequest;
    static void                 sendMessage(void);
    static void                 sendMessageConfirm(NWK_DataReq_t *req);
    static bool                 receiveMessage(NWK_DataInd_t *ind);

    static bool                 send_message_busy = false;

    byte pingCounter            = 0;
/** END Networking vars **/

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
    PHY_SetChannel(APP_CHANNEL);
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
    /*
    // My Node address
    Serial.print(APP_ADDRESS);
    Serial.print(' ');

    // Incomming mesh node address
    Serial.print(ind->srcAddr);
    Serial.print(' ');

    // RF Link quality index
    Serial.print(ind->lqi, DEC);
    Serial.print(' ');

    // RSSI
    Serial.print(ind->rssi, DEC);
    Serial.print(" ");
    */
    
    // Data
    char* data = (char*) ind->data;

    String str(data);

    int splitIndex = str.indexOf(',');
    int secondSplitIndex = str.indexOf(',', splitIndex + 1);
    int thirdSplitIndex = str.indexOf(',', secondSplitIndex + 1);
    int fourthSplitIndex = str.indexOf(',', thirdSplitIndex + 1);
    int fifthSplitIndex = str.indexOf(',', fourthSplitIndex + 1);
    int sixthSplitIndex = str.indexOf(',', fifthSplitIndex + 1);


    int seventhSplitIndex = str.indexOf(',', sixthSplitIndex + 1);
    int eighthSplitIndex = str.indexOf(',', seventhSplitIndex + 1);
    int ninethSplitIndex = str.indexOf(',', eighthSplitIndex + 1);
    /*int tenthSplitIndex = str.indexOf(',', ninethSplitIndex + 1);
    int eleventhSplitIndex = str.indexOf(',', tenthSplitIndex + 1);
    int twelvethSplitIndex = str.indexOf(',', eleventhSplitIndex + 1);
    int thirteenthSplitIndex = str.indexOf(',', twelvethSplitIndex + 1);
    int fourteenthSplitIndex = str.indexOf(',', thirteenthSplitIndex + 1);
    int fifteenthSplitIndex = str.indexOf(',', fourteenthSplitIndex + 1);
    int sixteenthSplitIndex = str.indexOf(',', fifteenthSplitIndex + 1);
    */
    
    unsigned long timestamp;
    
    float yaw_value;
    float pitch_value;
    float roll_value;
    float euler1;
    float euler2;
    float euler3;

    float value0;
    float value1;
    float value2;/*
    float value3;
    float value4;
    float value5;
    float value6;
    float value7;
    float value8;
    float value9;*/

    

    timestamp = str.substring(0, splitIndex).toInt();
    yaw_value = str.substring(splitIndex + 1, secondSplitIndex).toFloat();
    pitch_value = str.substring(secondSplitIndex + 1, thirdSplitIndex).toFloat();
    roll_value = str.substring(thirdSplitIndex + 1).toFloat();
    euler1 = str.substring(fourthSplitIndex + 1).toFloat();
    euler2 = str.substring(fifthSplitIndex + 1).toFloat();
    euler3 = str.substring(sixthSplitIndex + 1).toFloat();
    
    value0 = str.substring(seventhSplitIndex + 1).toFloat();
    value1 = str.substring(eighthSplitIndex + 1).toFloat();
    value2 = str.substring(ninethSplitIndex + 1).toFloat();
    /*value3 = str.substring(tenthSplitIndex + 1).toFloat();
    value4 = str.substring(eleventhSplitIndex + 1).toFloat();
    value5 = str.substring(twelvethSplitIndex + 1).toFloat();
    value6 = str.substring(thirteenthSplitIndex + 1).toFloat();
    value7 = str.substring(fourteenthSplitIndex + 1).toFloat();
    value8 = str.substring(fifteenthSplitIndex + 1).toFloat();
    value9 = str.substring(sixteenthSplitIndex + 1).toFloat();
    */

    Serial.print(APP_ADDRESS);
    Serial.print(',');
    Serial.print(APP_PANID);
    Serial.print(',');
    Serial.print(APP_CHANNEL);
    Serial.print(',');

    Serial.print(ind->srcAddr);
    Serial.print(',');

    Serial.print(timestamp);
    Serial.print(',');
    
    Serial.print(yaw_value);
    Serial.print(',');
    Serial.print(pitch_value);
    Serial.print(',');
    Serial.print(roll_value);
    Serial.print(',');
    Serial.print(euler1);
    Serial.print(',');
    Serial.print(euler2);
    Serial.print(',');
    Serial.print(euler3);
    Serial.print(',');
    Serial.print(value0);
    Serial.print(',');
    Serial.print(value1);
    Serial.print(',');
    Serial.println(value2);
    /*Serial.print(',');
    Serial.print(value3);
    Serial.print(',');
    Serial.print(value4);
    Serial.print(',');
    Serial.print(value5);
    Serial.print(',');
    Serial.print(value6);
    Serial.print(',');
    Serial.print(value7);
    Serial.print(',');
    Serial.print(value8);
    Serial.print(',');
    Serial.println(value9);*/
    //String str((char*)ind->data);
    //Serial.println(str);
    
    return true;
}
