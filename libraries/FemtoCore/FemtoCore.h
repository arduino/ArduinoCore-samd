#ifndef FemtoCore_h
    #define FemtoCore_h

    #define FEMTO_SEMVER_MAJOR "1"
    #define FEMTO_SEMVER_MINOR "0"
    #define FEMTO_SEMVER_PATCH "0"

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
    
    //#include "calibration.h" // Uncomment once you have calibrated your IMU, generated a calibration.h file and updated FreeIMU.h!

    #include <I2Cdev.h>
    #include <MPU60X0.h>

    #include <AK8963.h>
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



    /** Networking vars BOF **/
    #ifndef DEFAULT_ANTENNA
        #define DEFAULT_ANTENNA 0x01 // 1 = Chip antenna, 2 = uFL antenna
    #endif

    #define NWK_ENABLE_SECURITY // Enable AES encrypted comms.

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
    

    class FemtoCore 
    {
        public:
            
            /**
             * Constructor.
             */
            FemtoCore();

            static volatile bool is_femtobeacon_coin;
            // Sensor peripherals (9-DoF Sensor, Precision Altimeter)
            // ... FreeIMU Serial commands.

            // @TODO USB peripherals 

            // @TODO OTA Updates
            
            /**
             *           -----+ Common Anode (Pad #1 TOP VIEW)
             *    +---+ +---+ |
             * B  | 4 | | 1 | |
             *    +---+ +---+
             *    +---+ +---+
             * G  | 3 | | 2 |  R
             *    +---+ +---+
             *
             *
             *
             *
             *           ------+ Common Anode (Pad #1, TOP VIEW)
             *    +--+ +--+ ++ |         .   Note, on Dongle, rotate clockwise 90deg more.
             * B  |  | |  | || |           .
             *    +--+ |  | ||              .  (Edge of coin PCB)
             *    +--+ |  | ++               .
             * G  |  | |  +--+  R             .
             *    +--+ +-----+                .
             *                                .
             *                     +------------+
             *                     | USB Port
             *                     |
             *                     |
             *                     |
             *                     |
             *                     |
             *                     |
             *                     +------------+
             *                                .
             *                                .
             *                               .
             *                              .
             *                             .
             *                           .
             */

            static const int FEMTO_LED_R = 5; // RGB LED pad #2, Arduino pin 5 (PA06) RED
            static const int FEMTO_LED_G = 6; // RGB LED pad #3, Arduino pin 6 (PA07) GREEN
            static const int FEMTO_LED_B = 21; // RGB LED pad #4, Arduino pin 21 (PA27) BLUE

            static const int FEMTO_NETWORKING_BROADCAST_ADDRESS = 0xffff;
            static const int FEMTO_ANTENNA_SMD = 1;
            static const int FEMTO_ANTENNA_UFL = 2;

            /** BEGIN Serial Rx **/
            
            static volatile bool   stringComplete;  // whether the string is complete
            /** END Serial Rx **/

            /** RTC Stuff BOF **/
            static RTCZero rtc;
            /** RTC Stuff EOF **/

            /** FreeIMU Stuff BOF **/
            static const int FREEIMU_OUTPUT_BUFFER_SIZE = 128; // In the FreeIMU_serial_ARM_CPU sketch, the "str" variable was originally 128 chars.
            static const int FEMTO_PIN_IMU_INT = 4;
            static FreeIMU freeIMU;
            /** FreeIMU Stuff EOF **/

            /** Network AT86RF233 Stuff BOF **/
            static const int FEMTO_PIN_ATRF233_SLP_TR = 14;
            /** Network AT86RF233 Stuff EOF **/

            /** Sleep/Wake BOF **/
            static const byte FEMTO_WAKE_TRIGGER_TIME      = 0x01; // 0001
            static const byte FEMTO_WAKE_TRIGGER_NET       = 0x02; // 0010
            static const byte FEMTO_WAKE_TRIGGER_SENSOR    = 0x04; // 0100

            // ...Motion types used to trigger wake calls
            static const byte FEMTO_SENSOR_INT_FREE_FALL            = 0x01;
            static const byte FEMTO_SENSOR_INT_MOTION               = 0x02;
            static const byte FEMTO_SENSOR_INT_ZERO_MOTION          = 0x04;

            // ...These are used outside of sleep/wake functionality
            static const byte FEMTO_SENSOR_INT_FIFO_BUFFER_OVERFLOW = 0x08;
            static const byte FEMTO_SENSOR_INT_I2C_MASTER_INTERRUPT = 0x10; // DEC 16
            static const byte FEMTO_SENSOR_INT_DATA_READY           = 0x20; // DEC 32
            /** Sleep/Wake EOF **/
            

            /**
             * @param int appAddress This node's address.
             * @param int destAddress The destination node address.
             * @param int appEndpoint The callback endpoint number to use.
             * @param int appPanID The broadcast PAN ID we belong to.
             * @param int appChannel The channel to use (think walkie-talkie channels.)
             * @param char* appSecurityKey The char array of characters to use as the encryption security key when AES option is enabled.
             * @param bool is_coin If true, we identify as a FemtoBeacon coin (with sensors). False means it's a dongle (no sensors).
             */
            static void init(
                int appAddress, 
                int destAddress, 
                int appEndpoint, 
                int appPanID, 
                int appChannel, 
                char* appSecurityKey, 
                bool is_coin);

            static void startRTC();
            static void stopRTC();

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

            /**
             * Set LED color using RGB (0-255, 0-255, 0-255)
             */
            static void setRGB(byte R, byte G, byte B);
            static void setRGB(byte R, byte G, byte B, bool forceHandling);

            /**
             * Set LED color using HSV (0.0-360.0, 0.0-100.0, 0.0-100.0)
             */
            static void setHSV(float H, float S, float V);
            static void setHSV(float H, float S, float V, bool forceHandling);

            /**
             * Cycles through Red, Green, and Blue components of the RGB LED.
             */
            static void rgbTest();
            /**
             * Cycles through all Hues ROYGBIV
             */
            static void hsvTest();

            static void updateNetworkingConfig();

            static void setNetworkingAntenna(int antenna);
            static int  getNetworkingAntenna();

            static void setNetworkingPowerLevel(int powerLevel);
            static int  getNetworkingPowerLevel();

            static void setNetworkingRXState(bool rxState);
            static bool getNetworkingRXState();

            static void setDataFlow(bool data_flow);
            static bool getDataFlow();

            static void setDataFlowCommand(char* command);
            static char* getDataFlowCommand();

            static bool getIsNetworkBusy();

            static void handleNetworking();
            static void handleSerial();
            static void handleSerialRx();

            static void handleRepeatCommand();

            static void send(char* data);
            static void send(char* data, int destNodeAddress);
            static void send(char* data, int destNodeAddress, int destNodeEndpoint, bool requireConfirm);

            static void broadcast(char* data);
            static void broadcast(char* data, int destNodeAddress);

            static void processCommand(char* cmd, byte input_from, byte output_to, int to_node_id);

            static void sendSampleLegacy(byte input_from, byte output_to, int to_node_id);

            static void sleep();
            static void wakeUp();

            static void wakeSensors();
            static void sleepSensors();
            static void sensorWakeEvent();

            static void wakeNetwork();
            static void sleepNetwork();
            static void networkWakeEvent();

            static int  getRTCSleepMS();
            static void setRTCSleepMS(int sleep_ms);

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

            static volatile int     _sleep_mode; // Default is 0 (timed). 1 = AT68RF233 network event trigger. 2 = Sensor event trigger.
            static volatile int     _rtc_sleep_ms; // Default is 10 seconds (10000 ms)
            static volatile bool    _should_be_sleeping;

            static volatile bool    _sensor_is_on;
            static volatile bool    _data_flow_enabled;
            static char    _data_flow_command[APP_BUFFER_SIZE];

            static KalmanFilter kFilters[4];

            static float   _free_imu_ypr[3]; // Buffer to hold FreeIMU Yaw, Pitch, Roll data.
            static float   _free_imu_val[12]; // Buffer to hold FreeIMU results.
            static float   _free_imu_quaternions[4]; // Buffer to hold FreeIMU quaternion data.
            static char    _free_imu_network_data[APP_BUFFER_SIZE]; // Used by processCommand().
            static char    _free_imu_serial_data[FREEIMU_OUTPUT_BUFFER_SIZE]; // Used by processCommand(). In the original FreeIMU_serial_ARM_CPU sketch, the "str" char array was hard-coded to 128 characters.
            static int     _free_imu_raw_values[11]; // Buffer to hold FreeIMU raw value data.

            static NWK_DataReq_t _sendRequest;

            static void _HSV_to_RGB(float h, float s, float v, byte* r, byte* g, byte* b);

            
            static void _configureAntenna();
            static void _setupRGB();
            static void _setupSerial();
            static void _setupMeshNetworking();
            static void _setupRTC();
            static void _setupSensors();
            static void _setupSensorsWakeOnMotion();
            static void _setupNetworkWakeOnFrame();
            static void _setupFilters();

            static void _detachSensorsWakeOnMotion();
            static void _detachNetworkWakeOnFrame();

            static bool _networkingReceiveMessage(NWK_DataInd_t *ind);
            static void _phyWriteRegister(uint8_t reg, uint8_t value);

            static void _networkingSendMessage(char* bufferData, int destNodeAddress, int destNodeEndpoint, bool requireConfirm);

            static void _networkingSendMessageConfirm(NWK_DataReq_t *req);

            static void _reply(char* message, byte output_to, int dest_node_id);
            static char _serialBusyWait();

            static void _wakeTriggerTime();
            static void _wakeTriggerNetwork();
            static void _wakeTriggerSensor();

            static volatile byte _sensor_interrupt_source; // Default is FEMTO_SENSOR_INT_MOTION

            static unsigned int hexToDec(String hexString);
    };

#endif // FemtoCore_h