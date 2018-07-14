#ifndef FemtoCore_h
    #define FemtoCore_h


    #include <stdio.h>
    #include <string.h>
    #include <stdlib.h>

    #include <Arduino.h>
    #include <Wire.h>
    #include <avr/dtostrf.h>
    #include <SPI.h>

    #include <RTCZero.h>


    #define ENABLE_SERIAL       // We want serial USB output.
    #define DEBUG               // We also want to see debug serial output.

    #ifndef FEMTO_SERIAL_BAUD_RATE
        #define FEMTO_SERIAL_BAUD_RATE 115200 // Default baud rate. Note, this is the highest that works on Windows COM ports!
    #endif

    #define Serial SERIAL_PORT_USBVIRTUAL // Our "Serial" object is a Serial USB object.

    void serialEvent();
    void resetBuffer(char* bufferData, int bufferSize);


    /** BEGIN mjs513 fork https://github.com/femtoduino/FreeIMU-Updates library. **/
    // #ifdef IS_FEMTOBEACON_COIN
    //     //#include "calibration.h" // Uncomment once you have calibrated your IMU, generated a calibration.h file and updated FreeIMU.h!

    //     #include <I2Cdev.h>
    //     #include <MPU60X0.h>

    //     #include <AK8963.h>
    //     #include <AP_Baro_MS5611.h>  //Uncomment for APM2.5


    //     //These are mandatory
    //     #include <AP_Math_freeimu.h>
    //     #include <Butter.h>    // Butterworth filter
    //     #include <iCompass.h>
    //     #include <MovingAvarageFilter.h>

    //     #include "DebugUtils.h"
    //     #include "CommunicationUtils.h"
    //     #include "DCM.h"
    //     #include "FilteringScheme.h"
    //     #include "RunningAverage.h"
    //     #include "FreeIMU.h"

    //     // Arduino Zero: no eeprom
    //     #define HAS_EEPPROM 0
    // #endif
    /** END mjs513 fork https://github.com/femtoduino/FreeIMU-Updates library. **/



    /** Networking vars BOF **/
    #ifndef DEFAULT_ANTENNA
        #define DEFAULT_ANTENNA 0x01 // 1 = Chip antenna, 2 = uFL antenna
    #endif

    // #define NWK_ENABLE_SECURITY // Enable AES encrypted comms.

    #ifdef NWK_ENABLE_SECURITY
        #define APP_BUFFER_SIZE     (NWK_MAX_PAYLOAD_SIZE - NWK_SECURITY_MIC_SIZE)
    #else
        #define APP_BUFFER_SIZE      NWK_MAX_PAYLOAD_SIZE
    #endif

    #include "lwm.h"
    #include "lwm/sys/sys.h"
    #include "lwm/nwk/nwk.h"
    #include "lwm/phy/phy.h"

    #include "lwm/sys/sysTypes.h"
    #include "lwm/hal/hal.h"
    #include "lwm/hal/halPhy.h"

    static          String inputString;     // a String to hold incoming data

    // void SYS_INIT(void);
    // void SYS_TaskHandler(void);

    // void PHY_SetChannel(uint8_t channel);
    // void PHY_SetRxState(bool rx);
    // void PHY_SetTxPower(uint8_t txPower);

    // void NWK_OpenEndpoint(uint8_t id, bool (*handler)(NWK_DataInd_t *ind));
    // void NWK_SetAddr(uint16_t addr);
    // void NWK_SetPanId(uint16_t panId);

    // void NWK_DataReq(NWK_DataReq_t *req);

    /** Networking vars EOF **/

    
    /** RGB timer stuff BOF **/
    // Should be fast enough to fake PWM the Red pin.
    #define FEMTO_RGB_MAX_DUTY_CYCLE 48

    void TC5_Handler (void);

    void tcConfigure(int sampleRate);
    bool tcIsSyncing();
    void tcStartCounter();
    void tcReset();
    void tcDisable();

    /** RGB timer stuff EOF **/

    // /** BEGIN Sensor vars **/
    // // Sensor reading.
    // float ypr[3]; // Hold Yaw-Pitch-Roll (YPR) data.
    // float eulers[3]; // Hold euler angles (360 deg).
    // float values[10];
    // float baro; // Hold Barometer Altitude data
    // float temp; // Hold Temperature data
    // float pressure; // Hold Pressure data

    // // Set the FreeIMU object
    // FreeIMU sensors = FreeIMU();

    // // FemtoBeacon FSYNC pin is PA18 (not PA19, which is mislabeled in the silkscreen of FemtoBeacon rev 2.0.0)
    // // Must connect to GND if FSYNC is unused.
    // byte PIN_FSYNC = 4;

    // // FemtoBeacon INT pin is PA19 (not PA18, which is mislabeled in the silkscreen of FemtoBeacon r2.0.0)
    // byte PIN_INT = 3;
    // /** END Sensor vars **/


    // volatile bool is_sensor_on = 1;
    // volatile bool is_timestamp_on = 1;
    // volatile bool is_wireless_ok = 1;

    /** RTC Stuff BOF **/
    static RTCZero rtc;
    /** RTC Stuff EOF **/

    class FemtoCore 
    {
        public:
            
            /**
             *           -----+ Common Anode (Pad #1 TOP VIEW)
             *    +---+ +---+ |
             * B  | 4 | | 1 | |
             *    +---+ +---+
             *    +---+ +---+
             * G  | 3 | | 2 |  R
             *    +---+ +---+
             *
             *           ------+ Common Anode (Pad #1, TOP VIEW)
             *    +--+ +--+ ++ |
             * B  |  | |  | || |
             *    +--+ |  | ||
             *    +--+ |  | ++
             * G  |  | |  +--+  R
             *    +--+ +-----+
             */

            static const int FEMTO_LED_R = 5; // RGB LED pad #2, Arduino pin 5 (PA06) RED
            static const int FEMTO_LED_G = 6; // RGB LED pad #3, Arduino pin 6 (PA07) GREEN
            static const int FEMTO_LED_B = 21; // RGB LED pad #4, Arduino pin 21 (PA27) BLUE


            static const int FEMTO_ANTENNA_SMD = 1;
            static const int FEMTO_ANTENNA_UFL = 2;

            /** BEGIN Serial Rx **/
            
            static volatile bool   stringComplete;  // whether the string is complete
            /** END Serial Rx **/

            /**
             * Constructor.
             */
            FemtoCore();

            /**
             * @param int appAddress This node's address.
             * @param int destAddress The destination node address.
             * @param int appEndpoint The callback endpoint number to use.
             * @param int appPanID The broadcast PAN ID we belong to.
             * @param int appChannel The channel to use (think walkie-talkie channels.)
             * @param char* appSecurityKey The char array of characters to use as the encryption security key when AES option is enabled.
             */
            static void init(
                int appAddress, 
                int destAddress, 
                int appEndpoint, 
                int appPanID, 
                int appChannel, 
                char* appSecurityKey);

            static void setAddress(int appAddress);
            static int  getAddress();

            static void setDestAddress(int destAddress);
            static int  getDestAddress();

            static void setEndpoint(int appEndpoint);
            static int  getEndpoint();

            static void setPanId(int appPanID);
            static int  getPanId();

            static void setChannel(int appChannel);
            static int  getChannel();

            static void  setSecurityKey(char * appSecurityKey);
            static volatile char* getSecurityKey();

            static void handleRGB();
            static void handleRGBPWM(int led, volatile long &tick, volatile long &duty_cycle, volatile bool &state);

            static void setRGB(byte R, byte G, byte B);
            static void setRGB(byte R, byte G, byte B, bool forceHandling);

            /**
             * Cycles through Red, Green, and Blue components of the RGB LED.
             */
            static void rgbTest();

            static void updateNetworkingConfig();

            static void setNetworkingAntenna(int antenna);
            static int  getNetworkingAntenna();

            static void setNetworkingPowerLevel(int powerLevel);
            static int  getNetworkingPowerLevel();

            static void setNetworkingRXState(bool rxState);
            static bool getNetworkingRXState();

            static void handleNetworking();
            static void handleSerial();
            static void handleSerialRx();

            static void send(char* data);
            static void send(char* data, int destNodeAddress);
            static void send(char* data, int destNodeAddress, int destNodeEndpoint, bool requireConfirm);

            static void sleep();
            static void wakeUp();

        private:
            static volatile int _appAddress;
            static volatile int _destAddress;
            static volatile int _appEndpoint;
            static volatile int _appPanID;
            static volatile int _appChannel;
            static volatile char* _appSecurityKey;

            static volatile long _rgbLastTick;
            static volatile long _rgbCurrentTick;
            

            static volatile long _rgbRedTick;
            static volatile long _rgbGreenTick;
            static volatile long _rgbBlueTick;

            static volatile long _rgbRedDutyCycle; // (Brightness: 1 = ON, MAX_DUTY_CYCLE = OFF)
            static volatile long _rgbGreenDutyCycle;
            static volatile long _rgbBlueDutyCycle;

            static volatile bool _rgbRedState;
            static volatile bool _rgbGreenState;
            static volatile bool _rgbBlueState;

            static volatile int _defaultAntenna;

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

            // Set to default (max +4.4db) of 0x00
            static volatile int     _networkingPowerLevel; // +4.4db
            static volatile bool    _networkingRXState;    // Networking RX state enabled (default is true). Without this, we don't seem to get incomming data callbacks.

            static volatile int     _networking_error_count;
            static volatile bool    _networking_is_busy_sending;
            static volatile bool    _networking_status_is_ok;

            static volatile bool    _should_be_sleeping;

            static volatile bool    _sensor_is_on;

            static NWK_DataReq_t _sendRequest;

            

            static void _setupRGB();
            static void _setupSerial();
            static void _setupMeshNetworking();
            static void _setupRTC();

            static bool _networkingReceiveMessage(NWK_DataInd_t *ind);
            static void _phyWriteRegister(uint8_t reg, uint8_t value);

            static void _networkingSendMessage(char* bufferData, int destNodeAddress, int destNodeEndpoint, bool requireConfirm);

            static void _networkingSendMessageConfirm(NWK_DataReq_t *req);
    };

#endif // FemtoCore_h