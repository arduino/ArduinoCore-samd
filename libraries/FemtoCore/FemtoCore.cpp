#include "FemtoCore.h"

volatile bool FemtoCore::is_femtobeacon_coin = false;

volatile int FemtoCore::_appAddress;
volatile int FemtoCore::_destAddress;
volatile int FemtoCore::_appEndpoint;
volatile int FemtoCore::_appPanID;
volatile int FemtoCore::_appChannel;
volatile char* FemtoCore::_appSecurityKey;

// Initialize our static volatile RGB related variables.
volatile long FemtoCore::_rgbCurrentTick  = 0;
volatile long FemtoCore::_rgbLastTick     = 0;

volatile long FemtoCore::_rgbRedTick      = 0;
volatile long FemtoCore::_rgbGreenTick    = 0;
volatile long FemtoCore::_rgbBlueTick     = 0;

volatile long FemtoCore::_rgbRedDutyCycle     = FEMTO_RGB_MAX_DUTY_CYCLE;
volatile long FemtoCore::_rgbGreenDutyCycle   = FEMTO_RGB_MAX_DUTY_CYCLE;
volatile long FemtoCore::_rgbBlueDutyCycle    = FEMTO_RGB_MAX_DUTY_CYCLE;

volatile bool FemtoCore::_rgbRedState     = false;
volatile bool FemtoCore::_rgbGreenState   = false;
volatile bool FemtoCore::_rgbBlueState    = false;

volatile int FemtoCore::_defaultAntenna   = DEFAULT_ANTENNA;

volatile int  FemtoCore::_networkingPowerLevel   = 0x00;
volatile bool FemtoCore::_networkingRXState      = true; // Default to true, otherwise we never receive incomming mesh data!

volatile int FemtoCore::_networking_error_count         = 0;
volatile bool FemtoCore::_networking_is_busy_sending    = false;
volatile bool FemtoCore::_networking_status_is_ok       = true;
NWK_DataReq_t FemtoCore::_sendRequest;
volatile bool FemtoCore::_should_be_sleeping            = false;

volatile bool FemtoCore::_sensor_is_on                  = false;
KalmanFilter FemtoCore::kFilters[4];

float   FemtoCore::_free_imu_ypr[3]; // Buffer to hold FreeIMU Yaw, Pitch, Roll data.
float   FemtoCore::_free_imu_val[12]; // Buffer to hold FreeIMU results.
float   FemtoCore::_free_imu_quaternions[4]; // Buffer to hold FreeIMU quaternion data.
char    FemtoCore::_free_imu_network_data[APP_BUFFER_SIZE] = ""; // Used by processCommand().
char    FemtoCore::_free_imu_serial_data[FemtoCore::FREEIMU_OUTPUT_BUFFER_SIZE] = ""; // Used by processCommand(), FreeIMU stuff. In the original FreeIMU_serial_ARM_CPU sketch, the "str" char array was hard-coded to 128 characters.
int     FemtoCore::_free_imu_raw_values[11]; // Buffer to hold FreeIMU raw value data.

volatile bool FemtoCore::stringComplete  = false;      // whether the string is complete

RTCZero FemtoCore::rtc;

FreeIMU FemtoCore::freeIMU = FreeIMU();


FemtoCore::FemtoCore() {}

void FemtoCore::init(int appAddress, int destAddress, int appEndpoint, int appPanID, int appChannel, char* appSecurityKey, bool is_coin)
{
    _appAddress  = appAddress;
    _destAddress = destAddress;
    _appEndpoint = appEndpoint;
    _appPanID    = appPanID;
    _appChannel  = appChannel;
    _appSecurityKey = appSecurityKey;

    is_femtobeacon_coin = is_coin;

    #ifdef ENABLE_SERIAL
        _setupSerial();
    #endif

    _setupRGB();
    _setupRTC();
    _setupMeshNetworking();

    if (is_femtobeacon_coin) {
        _setupFilters();
        _setupSensors();

        broadcast("=INIT_COMPLETE");
    }

    #ifdef DEBUG
        Serial.println("FemtoCore::init() complete.");
    #endif
}

void FemtoCore::setAddress(int appAddress) {
    _appAddress = appAddress;
}
int  FemtoCore::getAddress() {
    return _appAddress;
}

void FemtoCore::setDestAddress(int destAddress) {
    _destAddress = destAddress;
}
int  FemtoCore::getDestAddress() {
    return _destAddress;
}

void FemtoCore::setEndpoint(int appEndpoint) {
    _appEndpoint = appEndpoint;
}
int  FemtoCore::getEndpoint() {
    return _appEndpoint;
}

void FemtoCore::setPanId(int appPanID) {
    _appPanID = appPanID;
}
int  FemtoCore::getPanId() {
    return _appPanID;
}

void FemtoCore::setChannel(int appChannel) {
    _appChannel = appChannel;
}
int  FemtoCore::getChannel() {
    return _appChannel;
}

void   FemtoCore::setSecurityKey(char * appSecurityKey) {
    _appSecurityKey = _appSecurityKey;
}
volatile char*  FemtoCore::getSecurityKey() {
    return _appSecurityKey;
}







void FemtoCore::handleRGB() {
    _rgbCurrentTick = micros();
  
    if (_rgbCurrentTick > _rgbLastTick + 1) {
        _rgbLastTick = _rgbCurrentTick;

        handleRGBPWM(FEMTO_LED_R, _rgbRedTick,   _rgbRedDutyCycle,    _rgbRedState);
        handleRGBPWM(FEMTO_LED_G, _rgbGreenTick, _rgbGreenDutyCycle,  _rgbGreenState);
        handleRGBPWM(FEMTO_LED_B, _rgbBlueTick,  _rgbBlueDutyCycle,   _rgbBlueState);

    }
}

void FemtoCore::handleRGBPWM(int led, volatile long &tick, volatile long &duty_cycle, volatile bool &state) {
    if (duty_cycle >= FEMTO_RGB_MAX_DUTY_CYCLE)
    {
        digitalWrite(led, HIGH);
        state = false;
        tick = 0;
        return;
    }

    if (state) {
        digitalWrite(led, HIGH);
        ++tick;
        if (tick >= duty_cycle) {
            state = false;
            tick = 0;
        }
    } else {
        digitalWrite(led, LOW);
        ++tick;

        if (tick >= (FEMTO_RGB_MAX_DUTY_CYCLE - duty_cycle)) {
            state = true;
            tick = 0;
        }
    }
}

void FemtoCore::setRGB(byte R, byte G, byte B) {
    setRGB(R, G, B, true);
}

void FemtoCore::setRGB(byte R, byte G, byte B, bool forceHandling) {
    // Map 0-255 to 1000-0
    _rgbRedDutyCycle    = map(R, 0, 255, FEMTO_RGB_MAX_DUTY_CYCLE, 1);
    _rgbGreenDutyCycle  = map(G, 0, 255, FEMTO_RGB_MAX_DUTY_CYCLE, 1);
    _rgbBlueDutyCycle   = map(B, 0, 255, FEMTO_RGB_MAX_DUTY_CYCLE, 1);

    if (forceHandling) {
        handleRGB();
    }
}
void FemtoCore::setHSV(float H, float S, float V) {
    setHSV(H, S, V, true);
}
void FemtoCore::setHSV(float H, float S, float V, bool forceHandling) {
    byte r;
    byte g;
    byte b;

    _HSV_to_RGB(H, S, V, &r, &g, &b);

    setRGB(r, g, b, forceHandling);
}

/**
 * Convert HSL values (H = 0.0 to 360.0, L = 0.0 to 100, V = 0.0 to 100) to RGB.
 * See https://gist.github.com/hdznrrd/656996
 */
void FemtoCore::_HSV_to_RGB(float H, float S, float V, byte* r, byte* g, byte* b) {
    int i;
    float f, p, q, t;

    H = max(0.0, min(360.0, H));
    S = max(0.0, min(100.0, S));
    V = max(0.0, min(100.0, V));

    S /= 100;
    V /= 100;

    if (S == 0) {
        // Achromatic (gray)
        *r = *g = *b = round(V * 255);
    }

    H /= 60; // sector 0 to 5 (hexagon)

    i = floor(H);
    f = H - i; // factorial part of h
    p = V * (1 - S);
    q = V * (1 - S * f);
    t = V * (1 - S * (1 - f));

    switch (i) {
        case 0:
            *r = round(255 * V);
            *g = round(255 * t);
            *b = round(255 * p);
            break;
        case 1:
            *r = round(255 * q);
            *g = round(255 * V);
            *b = round(255 * p);
            break;
        case 2:
            *r = round(255 * p);
            *g = round(255 * V);
            *b = round(255 * t);
            break;
        case 3:
            *r = round(255 * p);
            *g = round(255 * q);
            *b = round(255 * V);
            break;
        case 4:
            *r = round(255 * t);
            *g = round(255 * p);
            *b = round(255 * V);
            break;

        default:
            *r = round(255 * V);
            *g = round(255 * p);
            *b = round(255 * q);

            break;
    }
}



void FemtoCore::_setupRGB() {
    
    // Start up the clock used to emulate PWM control of our RGB LED pins.
    tcConfigure(FEMTO_RGB_MAX_DUTY_CYCLE);
    tcStartCounter();
  
    // Initialize our tick counters.
    _rgbCurrentTick = micros();
    _rgbLastTick = FemtoCore::_rgbCurrentTick;

    // Pins driven by analogWrite do not need to be declared as outputs, but whatever.
    pinMode(FEMTO_LED_R, OUTPUT);
    pinMode(FEMTO_LED_G, OUTPUT);
    pinMode(FEMTO_LED_B, OUTPUT);

    FemtoCore::rgbTest();
    FemtoCore::hsvTest();

}

void FemtoCore::rgbTest() {
    // Do a little light show to assert functionality.
    FemtoCore::setRGB(255, 0, 0); // Red
    delay(250);

    FemtoCore::setRGB(0, 255, 0); // Green
    delay(250);

    FemtoCore::setRGB(0, 0, 255); // Blue
    delay(250);

    // Turn off RGB LED
    FemtoCore::setRGB(0, 0, 0);
}

void FemtoCore::hsvTest() {
    int hue = 0;
    for (hue = 0; hue < 360; hue += 15) {
        FemtoCore::setHSV(hue, 100, 100);
        delay(50);
    }

    FemtoCore::setHSV(359, 100, 100);
    delay(50);

    FemtoCore::setHSV(0, 0, 0); // OFF
}

void FemtoCore::_setupSerial() {
        // Wait for a USB connection.
        while(!Serial);

        // Start at this baud rate.
        Serial.begin(FEMTO_SERIAL_BAUD_RATE);

        // The Serial RX (incomming) string buffer.
        inputString.reserve(APP_BUFFER_SIZE);

        #ifdef DEBUG
            Serial.println("FemtoCore::_setupSerial() complete.");
        #endif
}

void FemtoCore::_setupMeshNetworking() {
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


    updateNetworkingConfig();

    #ifdef DEBUG
        Serial.println("FemtoCore::_setupMeshNetworking() complete.");
    #endif
}

void FemtoCore::_setupRTC() {
    rtc.begin();

    #ifdef DEBUG
        Serial.println("FemtoCore::_setupRTC() complete.");
    #endif
}

void FemtoCore::_setupFilters() {
    int k_index = 3;
    float qVal = 0.125; //Set Q Kalman Filter(process noise) value between 0 and 1
    float rVal = 32.; //Set K Kalman Filter (sensor noise)

    for(int i = 0; i <= k_index; i++) { //Initialize Kalman Filters for 10 neighbors
        //KalmanFilter(float q, float r, float p, float intial_value);
        kFilters[i].KalmanInit(qVal,rVal,5.0,0.5);
    }
}
void FemtoCore::_setupSensors() {
    Wire.begin();

    #ifdef DEBUG
        Serial.println("FemtoCore::_setupSensors() initializing... ");
    #endif
    freeIMU.init(true);

    #ifdef DEBUG
        Serial.println("FemtoCore::_setupSensors() complete.");
    #endif
}

/**
 * Update the networking config after setting networking property/properties.
 */
void FemtoCore::updateNetworkingConfig() {

    #ifdef DEBUG
        Serial.print("This Node Address: 0x");
        Serial.println(_appAddress, HEX);

        Serial.print("Dest Node Address: 0x");
        Serial.println(_destAddress, HEX);

        Serial.print("Endpoint:          0x");
        Serial.println(_appEndpoint, HEX);

        Serial.print("Broadcast PAN ID:  0x");
        Serial.println(_appPanID, HEX);

        Serial.print("Channel:           0x");
        Serial.println(_appChannel, HEX);

        Serial.print("TX Power:          0x");
        Serial.println(_networkingPowerLevel, HEX);

        Serial.print("Antenna:           0x");
        Serial.println(_defaultAntenna, HEX);
    #endif

    // Use u.FL antenna?
    // Set to chip antenna, or uFL antenna?
    if (FEMTO_ANTENNA_UFL == _defaultAntenna) {
      _phyWriteRegister(ANT_DIV_REG, (2 << ANT_CTRL) | (1 << ANT_EXT_SW_EN));
    } else {
      _phyWriteRegister(ANT_DIV_REG, (1 << ANT_CTRL) | (1 << ANT_EXT_SW_EN));
    }

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
    PHY_SetTxPower(_networkingPowerLevel); // Set to +4 dBm

    NWK_SetAddr(_appAddress);
    NWK_SetPanId(_appPanID);
    PHY_SetChannel(_appChannel);
    PHY_SetRxState(_networkingRXState);
    NWK_OpenEndpoint(_appEndpoint, _networkingReceiveMessage);

    #ifdef DEBUG
        Serial.println("FemtoCore::updateNetworkingConfig() complete.");
    #endif
}

void FemtoCore::setNetworkingAntenna(int antenna) {
    _defaultAntenna = antenna;
}

int  FemtoCore::getNetworkingAntenna() {
    return _defaultAntenna;
}

void FemtoCore::setNetworkingPowerLevel(int powerLevel) {
    _networkingPowerLevel = powerLevel;
}
int  FemtoCore::getNetworkingPowerLevel() {
    return _networkingPowerLevel;
}

void FemtoCore::setNetworkingRXState(bool rxState) {
    _networkingRXState = rxState;
}
bool FemtoCore::getNetworkingRXState() {
    return _networkingRXState;
}

void FemtoCore::_phyWriteRegister(uint8_t reg, uint8_t value) {
  SPI.transfer(RF_CMD_REG_W | reg);
  SPI.transfer(value);
}

bool FemtoCore::_networkingReceiveMessage(NWK_DataInd_t *ind) {

    // Pointer to ind has the following properties:
    // ->srcAddr Source Address
    // ->dstAddr Destination Address
    // ->srcEndpoint Source Endpoint
    // ->dstEndpoint Destination Endpoint
    // ->options Option flags.
    // ->data Pointer to uint8_t array data.
    // ->size Size of the message.
    // ->lqi Link Quality Indicator.
    // ->rssi Relative Received Signal Strength.
    #ifdef DEBUG
        Serial.print("FemtoCore::_networkingReceiveMessage() called. Data (size ");
        Serial.print((int)ind->size);
        Serial.print(") is ");
        Serial.println((char*)ind->data);
    #endif;

    char* input_string = (char*)ind->data; // ind->data must by typecast from uint8_t* to char*
    String input = String(input_string);

    #ifdef DEBUG
        Serial.print("FemtoCore::_networkingReceiveMessage() String is ");
        Serial.println(input);
    #endif

    String firstChar = input.substring(0, 1);
    // char first_char = (char)input_string[0];
    char first_char = firstChar.charAt(0);
    #ifdef DEBUG
        Serial.println("");
        Serial.print("FemtoCore::_networkingReceiveMessage() First char is '");
        Serial.print(first_char);
        Serial.print("'.");
        Serial.println(" Processing...");
    #endif

    
    if (first_char == '=') {
        // This is a reply. Output to Serial with new line
        #ifdef ENABLE_SERIAL
            Serial.print("from:0x");
            Serial.print((int)ind->srcAddr, HEX);
            Serial.print(":0x");
            Serial.print((int)ind->srcEndpoint, HEX);
            Serial.print(":");
            Serial.println(input_string);
        #endif
    } else if (first_char == '<') {
        // This is a reply. Output to Serial without new line
        #ifdef ENABLE_SERIAL
            Serial.print("from:0x");
            Serial.print((int)ind->srcAddr, HEX);
            Serial.print(":0x");
            Serial.print((int)ind->srcEndpoint), HEX;
            Serial.print(":");
            Serial.print(input_string);
        #endif
    } else {
        // From network (direct), to network (direct), reply to the incomming node ID address.
        processCommand(input_string, 0x01, 0x02, (int)ind->srcAddr);
    }

    #ifdef DEBUG
        Serial.println("FemtoCore::_networkingReceiveMessage() complete.");
    #endif
    return true;
}

void FemtoCore::handleNetworking() {
    SYS_TaskHandler();
}

/**
 * Send a message. Note, remember to terminate your data with a new-line char!
 */
void FemtoCore::send(char* data) {
    send(data, _destAddress, _appEndpoint, _networkingRXState); // Only set to true if we enabled RX State config!
}

void FemtoCore::send(char* data, int destNodeAddress) {
    send(data, destNodeAddress, _appEndpoint, _networkingRXState);
}

void FemtoCore::broadcast(char* data) {
    send(data, FEMTO_NETWORKING_BROADCAST_ADDRESS, _appEndpoint, false);
}

void FemtoCore::broadcast(char* data, int destNodeAddress) {
    send(data, FEMTO_NETWORKING_BROADCAST_ADDRESS, _appEndpoint, false);
}

void FemtoCore::send(char* data, int destNodeAddress, int destNodeEndpoint, bool requireConfirm) {
    // Turn data into chunks of APP_BUFFER_SIZE.
    int buffer_size = strlen(data); // Measure how many bytes occupied by data.
    int timeout = 0;
    char tempBuffer[APP_BUFFER_SIZE];
    char* tempBufferPointer;

    if (buffer_size >= APP_BUFFER_SIZE) {
        #ifdef DEBUG
            Serial.print("FemtoCore::send() chunked, max buffer size is ");
            Serial.print(APP_BUFFER_SIZE);
            Serial.print(", this buffer size is ");
            Serial.print(buffer_size);
            Serial.print(". Sending: ");
            Serial.println(data);
        #endif
        int start_index = 0;
        int chunk_size = APP_BUFFER_SIZE - 1;
        int leftover_size = buffer_size % chunk_size;

        // begin for loop
        for (start_index = 0; start_index < buffer_size; start_index += chunk_size) {

            // On final loop, copy the left over bits only (to avoid copying outside the data size)
            if (leftover_size > 0 && start_index < (buffer_size - (buffer_size % chunk_size)) - chunk_size) {
                memcpy(
                    tempBuffer, 
                    data + start_index, 
                    (buffer_size % chunk_size));
            } else {
            // Copy in chunks sized chunk_size
                char* terminator = (char*)((char) 0);

                memcpy(tempBuffer, data + start_index, chunk_size);
                memcpy(tempBuffer, terminator, chunk_size + 1); // Needs the null terminator.
            }

            #ifdef DEBUG
                Serial.print("FemtoCore::send() Chunk (at index ");
                Serial.print(start_index);
                Serial.print(", size ");
                Serial.print(sizeof(tempBuffer));
                Serial.print(") data:");
                Serial.println(tempBuffer);
            #endif
            tempBufferPointer = (char*) tempBuffer;
            _networkingSendMessage(tempBufferPointer, destNodeAddress, destNodeEndpoint, requireConfirm);
            
            // Clear out the temp buffer
            

            // Wait for confirm if required before sending next chunk.
            if (requireConfirm) {
                while (_networking_is_busy_sending) {
                    ++timeout;
                    delay(10);

                    if (timeout >= NWK_ACK_WAIT_TIME) {
                        timeout = 0;
                        _networking_is_busy_sending = false;
                        memset(tempBuffer, 0, sizeof(tempBuffer));
                        #ifdef DEBUG
                            Serial.println("FemtoCore::send() chunk send timed out waiting for message acknowledge confirm!");
                        #endif
                        break;
                    }
                }
                
            } else {
                memset(tempBuffer, 0, sizeof(tempBuffer));
            }
        } // end for loop

    } else {
        #ifdef DEBUG
            Serial.print("FemtoCore::send() non-chunked, max buffer size is ");
            Serial.print(APP_BUFFER_SIZE);
            Serial.print(". Processing ");
            Serial.print(data);
            Serial.print(" (size ");
            Serial.print(strlen(data));
            Serial.println(").");
        #endif
        
        // char charData[APP_BUFFER_SIZE];
        // memcpy(charData, data, strlen(data));

        #ifdef DEBUG
            Serial.print("FemtoCore::send() non-chunked, data is: ");
            Serial.println(data);
        #endif
        // _networkingSendMessage(charData, destNodeAddress, destNodeEndpoint, requireConfirm);
        _networkingSendMessage(data, destNodeAddress, destNodeEndpoint, requireConfirm);

    }

    #ifdef DEBUG
        Serial.print("FemtoCore::send() from 0x");
        Serial.print(_appAddress, HEX);
        Serial.print(" to 0x");
        Serial.print(destNodeAddress, HEX);
        Serial.print(" on endpoint 0x");
        Serial.print(destNodeEndpoint, HEX);
        Serial.println(" complete.");
    #endif
}

void FemtoCore::_networkingSendMessage(char* bufferData, int destNodeAddress, int destNodeEndpoint, bool requireConfirm) {

    if (requireConfirm && _networking_is_busy_sending) {
        #ifdef DEBUG
            Serial.println("FemtoCore::_networkingSendMessage() ...sendMessage() busy. Exiting routine.");
        #endif

        return;
    }

    #ifdef DEBUG
        Serial.print("FemtoCore::_networkingSendMessage() sending message ");
        Serial.print(bufferData);
        Serial.print(" (size ");
        Serial.print(strlen(bufferData));
        Serial.println(").");
    #endif

    // Broadcast types:
    // NWK_IND_OPT_ACK_REQUESTED     = 1 << 0
    // NWK_IND_OPT_SECURED           = 1 << 1
    // NWK_IND_OPT_BROADCAST         = 1 << 2
    // NWK_IND_OPT_LOCAL             = 1 << 3
    // NWK_IND_OPT_BROADCAST_PAN_ID  = 1 << 4
    // NWK_IND_OPT_LINK_LOCAL        = 1 << 5
    // NWK_IND_OPT_MULTICAST         = 1 << 6

    _sendRequest.dstAddr       = destNodeAddress;
    _sendRequest.dstEndpoint   = destNodeEndpoint; // Endpoint number on destination device
    _sendRequest.srcEndpoint   = _appEndpoint; // Local Endpoint number

    _sendRequest.options       = (requireConfirm ? 
                                    NWK_IND_OPT_ACK_REQUESTED : // Default to acknowledge request flag
                                    0 // Just broadcast.
                                 );

    // Support for NWK_IND_OPT_SECURED option when NWK_ENABLE_SECURITY is defined.
    #ifdef NWK_ENABLE_SECURITY
        _sendRequest.options |= NWK_OPT_ENABLE_SECURITY;
    #endif

    _sendRequest.data          = (uint8_t*)bufferData;
    _sendRequest.size          = strlen(bufferData);//+1;

    // if (requireConfirm) {
    //     _sendRequest.options      |= NWK_IND_OPT_ACK_REQUESTED; // Assert acknowledge request flag.
        
    // }
    _sendRequest.confirm       = _networkingSendMessageConfirm;

    // DOO EEET.
    NWK_DataReq(&_sendRequest);

    _networking_is_busy_sending = requireConfirm; // Mark as busy if we are waiting for message confirmation.

    #ifdef DEBUG
        Serial.println("FemtoCore::_networkingSendMessage() complete.");

        if (requireConfirm) {
            Serial.println("FemtoCore::_networkingSendMessage() awaiting message acknowledge.");
        }
    #endif
}

void FemtoCore::_networkingSendMessageConfirm(NWK_DataReq_t *req)
{

    #ifdef DEBUG
        Serial.println("FemtoCore::_networkingSendMessageConfirm() processing...");
    #endif

    _networking_is_busy_sending = false;
    _networking_status_is_ok = false;

    if (NWK_NO_ACK_STATUS == req->status) {

        #ifdef DEBUG
            Serial.println("FemtoCore::_networkingSendMessageConfirm() NWK_NO_ACK_STATUS");
        #endif
        setRGB(255, 255, 0, false); // Yellow.
    } else if (NWK_PHY_NO_ACK_STATUS == req->status) {
        #ifdef DEBUG
            Serial.println("FemtoCore::_networkingSendMessageConfirm() NWK_PHY_NO_ACK_STATUS.");
        #endif

        setRGB(255, 0, 0, false); // Red
    } else if (NWK_NO_ROUTE_STATUS == req->status) {

        #ifdef DEBUG
            Serial.println("FemtoCore::_networkingSendMessageConfirm() NWK_NO_ROUTE_STATUS");
        #endif
        setRGB(255, 90, 0, false); // Orange.

    } else if (NWK_SUCCESS_STATUS == req->status) {
        _networking_error_count = 0;
        _networking_status_is_ok = true;

        _networking_is_busy_sending = false;
        #ifdef DEBUG
            Serial.println("FemtoCore::_networkingSendMessageConfirm() NWK_SUCCESS_STATUS");
        #endif

        setRGB(0, 0, 255, false); // Blue.
    } else {

        #ifdef DEBUG
            Serial.print("FemtoCore::_networkingSendMessageConfirm() UNKNOWN STATE is 0x");
            Serial.println(req->status, HEX);
        #endif

        // @TODO See if there is an network change event we can use to determine if we go to sleep().
        // if (is_femtobeacon_coin) {
        //     ++_networking_error_count;
        //     setRGB(255, 0, 0, false); // Red

        //     if (_networking_error_count > 1000) {
        //         #ifdef DEBUG
        //             Serial.println("FemtoCore::_networkingSendMessageConfirm() NWK_ERROR_STATUS ...Going to sleep.");
        //         #endif

        //         _networking_error_count = 0;
        //         sleep();
        //     }
        // }
    }


    #ifdef DEBUG
        Serial.println("FemtoCore::_networkingSendMessageConfirm() complete.");
    #endif

    // #ifdef ENABLE_SERIAL
    //     serial.println("OK")
    // #endif
    (void) req;
}

void FemtoCore::handleSerial() {

    if (Serial.available()) serialEvent();
}

void FemtoCore::handleSerialRx() {
    
    // @TODO Clean up this kludge. Figure out why we get garbage on the receiving node when size is < 8.

    // Send it. See https://coderwall.com/p/zfmwsg/arduino-string-to-char
    char* input_string = const_cast<char*>(inputString.c_str());
    String cmd = inputString.substring(1); // Grab stuff after the first char.
    char* data = const_cast<char*>(cmd.c_str());

    #ifdef DEBUG
        Serial.print("FemtoCore::handleSerialRx() input data (");
        Serial.print(strlen(input_string));
        Serial.print(") is: ");
        Serial.println(input_string);
        Serial.print("FemtoCore::handleSerialRx() without first char is ");
        Serial.println(data);
    #endif
    // refactor into processCommand() call.
    if ((char)input_string[0] == ':') {
        
        // Send to currently set destination node ID.
        // processCommand(input_string, 0x00, 0x01, _destAddress);
        send(data);
    } else if ((char)input_string[0] == '>') {
        // From Serial, To Network (broadcast). No dest node ID
        // processCommand(input_string, 0x00, 0x02, 0x00);
        broadcast(data);
    } else {
        // From Serial, To Serial. No dest node ID.
        processCommand(input_string, 0x00, 0x00, 0x00);
    }

    FemtoCore::stringComplete = false;
    inputString = "";

}

void FemtoCore::sleep() {
    // Set internal AT86RF233 tranciever to SLEEP before
    // SAM R21 enters STANDBY
    #ifdef DEBUG
        Serial.println("FemtoCore::sleep() Internal AT86RF233 going into standby mode...");
    #endif
    // Stope the RGB LED timer
    setRGB(128, 0, 0); // Red (low power)

    tcDisable();
    tcReset();

    // Set AT86RF233 Radio module to Standby mode.
    NWK_SleepReq();
    delay(1000);

    // @TODO Set the MPU-9250 to SLEEP mode
    //freeIMU.
    _sensor_is_on = false;

    // Go to sleep
    // See https://github.com/arduino/ArduinoCore-samd/issues/142
    // Configure the regulator to run in normal mode when in standby mode
    // Otherwise it defaults to low power mode and can only supply 50uA
    _should_be_sleeping = true;

    // See https://github.com/arduino-libraries/RTCZero/blob/master/examples/SleepRTCAlarm/SleepRTCAlarm.ino
    // Add alarm to wake up
    // ...set alarm for 10 seconds into the future
    uint32_t uEpoch = rtc.getEpoch();
    uint32_t uNextEpoch = uEpoch + 10000; // 10 seconds into the future

    //int AlarmTime = rtc.getSeconds() + 10; // Add 10 seconds
    //AlarmTime = AlarmTime % 60; // Checks for roll over at 60 seconds, and corrects


    #ifdef DEBUG
        Serial.println("FemtoCore::wakeUp() SAM R21 Entering standby mode!");
        Serial.end();
        USBDevice.detach();
        delay(1000);
    #endif


    #ifdef DEBUG
        Serial.print("FemtoCore::wakeUp() Time is ");
        Serial.print(uEpoch);
        Serial.print(".Next alarm time (seconds) is ");
        Serial.print(uNextEpoch);
        Serial.println(". Setting alarm...");
        Serial.end();
    #endif
    
    setRGB(0, 0, 0); // Off

    rtc.attachInterrupt(wakeUp);
    rtc.setAlarmEpoch(uNextEpoch);
    rtc.enableAlarm(rtc.MATCH_SS); // Match seconds only

    rtc.standbyMode();

    #ifdef DEBUG
        USBDevice.attach();
        delay(1000);
        
        while(!Serial);

        Serial.begin(FEMTO_SERIAL_BAUD_RATE);
        Serial.println("FemtoCore::wakeUp() Woke up... Continue Mesh networking.");
        // Serial.end();
    #endif

    _should_be_sleeping = false;
    _sensor_is_on = true;

    // RGB LED stuff
    tcConfigure(FEMTO_RGB_MAX_DUTY_CYCLE);
    tcStartCounter();
    // Wake up the Network device
    NWK_WakeupReq();
    delay(1000);

    #ifdef DEBUG
        Serial.println("FemtoCore::wakeUp() Networking re-enabled.");
        // Serial.end();
    #endif

    if (is_femtobeacon_coin) {

        // freeIMU.RESET();
        // freeIMU.RESET_Q();
        // freeIMU.init(true);

        #ifdef DEBUG
            Serial.println("FemtoCore::wakeUp() Sensors reset.");
            // Serial.end();
        #endif
    }

    // Reset RGB pins, and tick
    _setupRGB();
    setRGB(255, 255, 0);
}

void FemtoCore::wakeUp() {
    _should_be_sleeping = false;
    _sensor_is_on = true;

    #ifdef DEBUG
        Serial.println("FemtoCore::wakeUp() complete.");
    #endif
}

void FemtoCore::sendSampleLegacy(byte input_from, byte output_to, int to_node_id) {
    
    float ypr[3]; // Hold the YPR data (YPR 180 deg)
    float eulers[3]; // Hold the Euler angles (360 deg)
    float values[10]; // Raw values from FreeIMU
    float baro_reading;
    float temp_reading;
    float pressure_reading;

    unsigned long current_ms; // Read the current millis() reading.

    String strCurrentMS;
    char* c_current_ms;

    char c_yaw[5];
    char c_pitch[5];
    char c_roll[5];
    char c_euler1[10];
    char c_euler2[10];
    char c_euler3[10];
    char c_accel_x[10];
    char c_accel_y[10];
    char c_accel_z[10];
    // char c_barometer[];
    // char c_temperature[];
    // char c_pressure[];

    char data[APP_BUFFER_SIZE];
    char* data_pointer;

    current_ms = millis();
    strCurrentMS = String(current_ms, HEX);
    c_current_ms = const_cast<char*>(strCurrentMS.c_str());
    // c_current_ms = strCurrentMS.c_str();

    freeIMU.getYawPitchRoll180(ypr);
    freeIMU.getEuler360deg(eulers);
    freeIMU.getValues(values);

    // baro = freeIMU.getBaroAlt(); // Returns an altitude estimate from barometer readings only using sea_press as current sea level pressure
    // temp = freeIMU.getBaroTemperature();
    // pressure = freeIMU.getBaroPressure();

    dtostrf(ypr[0], 0, 4, c_yaw); // Up to 4 decimal places
    dtostrf(ypr[1], 0, 4, c_pitch);
    dtostrf(ypr[2], 0, 4, c_roll);

    dtostrf(eulers[0], 0, 4, c_euler1);
    dtostrf(eulers[1], 0, 4, c_euler2);
    dtostrf(eulers[2], 0, 4, c_euler3);

    dtostrf(values[0], 0, 4, c_accel_x);
    dtostrf(values[1], 0, 4, c_accel_y);
    dtostrf(values[2], 0, 4, c_accel_z);

    // data = c_current_ms + ',' + c_yaw + ',' + c_pitch + ',' + c_roll;
    // data += ',' + c_euler1 + ',' + c_euler2 + ',' + c_euler3;
    // data += ',' + c_accel_x + ',' + c_accel_y + ',' + c_accel_z + '\n';
    #ifdef DEBUG
        Serial.println("FemtoCore::sendSampleLegacy() converting to char array...");
        Serial.print('\t');
        Serial.print(c_current_ms);Serial.print(',');
        Serial.print(c_yaw); Serial.print(',');
        Serial.print(c_pitch); Serial.print(',');
        Serial.print(c_roll); Serial.print(',');
        Serial.print(c_euler1); Serial.print(',');
        Serial.print(c_euler2); Serial.print(',');
        Serial.print(c_euler3); Serial.print(',');
        Serial.print(c_accel_x); Serial.print(',');
        Serial.print(c_accel_y); Serial.print(',');
        Serial.println(c_accel_z);
    #endif
    sprintf(data, 
        "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", 
        c_current_ms, 
        c_yaw, c_pitch, c_roll, 
        c_euler1, c_euler2, c_euler3, 
        c_accel_x, c_accel_y, c_accel_z);

    // Send away.
    data_pointer = (char*) data;
    // send(data_pointer, destNodeAddress);
    _reply(data_pointer, output_to < 1 ? 1 : output_to, to_node_id);
    memset(data, 0, sizeof(data));
}
/**
 * Process a command. 
 * @param char* cmd Pointer to the char array representing the command
 * @param byte  input_from Zero (0) means Serial, One (1) means network node.
 * @param byte  output_to Zero (0) means Serial (without new line), One (1) means Serial (with new line), Two (2) means network (direct), Three (3) means network (broadcast)
 * @param int   to_node_id If a reply is required, this is the node address ID which requested a reply.
 */
void FemtoCore::processCommand(char* command_chars, byte input_from, byte output_to, int to_node_id) {

    int command_size = strlen(command_chars);
    // char cmd = (char)command_chars[0];
    
    
    String command = String(command_chars);
    char cmd = command.charAt(0);

    #ifdef DEBUG
        Serial.print("FemtoCore::processCommand got ");
        Serial.println(command);
    #endif

    if (is_femtobeacon_coin) {

        // FreeIMU command.
        if(cmd=='v') {
            char* buffer = output_to > 0 ? _free_imu_network_data : _free_imu_serial_data;

            sprintf(buffer, "FemtoCore %s.%s.%s. FreeIMU by %s, FREQ:%s, LIB_VERSION: %s, IMU: %s", FEMTO_SEMVER_MAJOR, FEMTO_SEMVER_MINOR, FEMTO_SEMVER_PATCH, FREEIMU_DEVELOPER, FREEIMU_FREQ, FREEIMU_LIB_VERSION, FREEIMU_ID);
            // Serial.print(_free_imu_serial_data);
            // Serial.print('\n');

            // If output_to is serial, output with a new-line at the end.
            // If output_to is network, send output as is.
            _reply(buffer, output_to < 1 ? 1 : output_to, to_node_id);
            memset(buffer, 0, sizeof(buffer));
        }
        else if(cmd=='1'){
            freeIMU.init(true);
            if (output_to > 0) {
                _reply("1:OK", output_to, to_node_id);
            }
        }
        else if(cmd=='2'){
            freeIMU.RESET_Q();
            if (output_to > 0) {
                _reply("2:OK", output_to, to_node_id);
            }
        }
        else if(cmd=='g'){
            freeIMU.initGyros();
            if (output_to > 0) {
                _reply("g:OK", output_to, to_node_id);
            }
        }
        else if(cmd=='t'){
            //available opttions temp_corr_on, instability_fix
            freeIMU.setTempCalib(1);
            if (output_to > 0) {
                _reply("t:OK", output_to, to_node_id);
            }
        }
        else if(cmd=='f'){
            //available opttions temp_corr_on, instability_fix
            freeIMU.initGyros();
            freeIMU.setTempCalib(0);
            if (output_to > 0) {
                _reply("f:OK", output_to, to_node_id);
            }
        }
        else if(cmd=='p'){
            //set sea level pressure
            // long sea_press = Serial.parseInt();
            long sea_press = command.substring(1).toInt();
            freeIMU.setSeaPress(sea_press/100.0);
            //Serial.println(sea_press);

            if (output_to > 0) {
                _reply("p:OK", output_to, to_node_id);
            }
        } 
        else if(cmd=='r') {
            // uint8_t count = _serialBusyWait(); // Expects a char representing count
            uint8_t count = command.substring(1).toInt(); // grab the char after the 'r' char. It's the count.
            char* buffer = output_to > 1 ? _free_imu_network_data : _free_imu_serial_data;
            for(uint8_t i=0; i<count; i++) {
                String output = String("");

                //freeIMU.getUnfilteredRawValues(_free_imu_raw_values);
                freeIMU.getRawValues(_free_imu_raw_values);
                sprintf(buffer, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,", _free_imu_raw_values[0], _free_imu_raw_values[1], _free_imu_raw_values[2], _free_imu_raw_values[3], _free_imu_raw_values[4], _free_imu_raw_values[5], _free_imu_raw_values[6], _free_imu_raw_values[7], _free_imu_raw_values[8], _free_imu_raw_values[9]);
                // Serial.print(_free_imu_serial_data);
                output = String(buffer);

                #if HAS_PRESS()
                  // Serial.print(freeIMU.getBaroTemperature()); Serial.print(",");
                  // Serial.print(freeIMU.getBaroPressure()); Serial.print(",");
                    output += String(freeIMU.getBaroTemperature()) + ',' + String(freeIMU.getBaroPressure()) + ',';

                #endif
                // Serial.print(millis()); Serial.print(",");
                // Serial.println("\r\n");

                output += millis() + ',' + "\r\n";
                buffer = const_cast<char*>(output.c_str());
                _reply(buffer, output_to < 1 ? 0 : output_to, to_node_id);
                memset(buffer, 0, sizeof(buffer));
            }
        }
        else if(cmd=='b') {
            // uint8_t count = _serialBusyWait(); // Expects a char representing count
            uint8_t count = command.substring(1).toInt(); // grab the char after the 'b' char. It's the count.
            char* buffer = output_to > 1 ? _free_imu_network_data : _free_imu_serial_data;
            for(uint8_t i=0; i<count; i++) {
                // String output = String("");

                #if HAS_ITG3200()
                  freeIMU.acc.readAccel(&_free_imu_raw_values[0], &_free_imu_raw_values[1], &_free_imu_raw_values[2]);
                  freeIMU.gyro.readGyroRaw(&_free_imu_raw_values[3], &_free_imu_raw_values[4], &_free_imu_raw_values[5]);
                  // writeArr(_free_imu_raw_values, 6, sizeof(int)); // writes accelerometer, gyro values & mag if 9150
                  toPrintableArr(_free_imu_raw_values, 6, sizeof(int), buffer);
                #elif HAS_MPU9150()  || HAS_MPU9250() || HAS_LSM9DS0() || HAS_LSM9DS1()
                  freeIMU.getRawValues(_free_imu_raw_values);
                  // writeArr(_free_imu_raw_values, 9, sizeof(int)); // writes accelerometer, gyro values & mag if 9150
                  toPrintableArr(_free_imu_raw_values, 9, sizeof(int), buffer);
                #elif HAS_MPU6050() || HAS_MPU6000()   // MPU6050
                  //freeIMU.accgyro.getMotion6(&_free_imu_raw_values[0], &_free_imu_raw_values[1], &_free_imu_raw_values[2], &_free_imu_raw_values[3], &_free_imu_raw_values[4], &_free_imu_raw_values[5]);
                  freeIMU.getRawValues(_free_imu_raw_values);
                  // writeArr(_free_imu_raw_values, 6, sizeof(int)); // writes accelerometer, gyro values & mag if 9150
                  toPrintableArr(_free_imu_raw_values, 6, sizeof(int), buffer);
                #elif HAS_ALTIMU10() || HAS_ADA_10_DOF()
                  freeIMU.getRawValues(_free_imu_raw_values);
                  // writeArr(_free_imu_raw_values, 9, sizeof(int)); // writes accelerometer, gyro values & mag of Altimu 10        
                  toPrintableArr(_free_imu_raw_values, 9, sizeof(int), buffer);
                #endif

                #if IS_9DOM() && (!HAS_MPU9150()  && !HAS_MPU9250() && !HAS_ALTIMU10() && !HAS_ADA_10_DOF() && !HAS_LSM9DS0() && !HAS_LSM9DS1())
                  freeIMU.magn.getValues(&_free_imu_raw_values[0], &_free_imu_raw_values[1], &_free_imu_raw_values[2]);
                  // writeArr(_free_imu_raw_values, 3, sizeof(int));
                  toPrintableArr(_free_imu_raw_values, 3, sizeof(int), buffer);
                #endif
                // Serial.println();
                _reply(buffer, output_to < 1 ? 1 : output_to, to_node_id);
                memset(buffer, 0, sizeof(buffer));
            }
        }
        else if(cmd == 'q') {
            // uint8_t count = _serialBusyWait(); // Expects a char representing count
            uint8_t count = command.substring(1).toInt(); // grab the char after the 'q' char. It's the count.
            char* buffer = output_to > 1 ? _free_imu_network_data : _free_imu_serial_data;
            for(uint8_t i=0; i<count; i++) {
                freeIMU.getQ(_free_imu_quaternions, _free_imu_val);
                // serialPrintFloatArr(_free_imu_quaternions, 4);
                // Serial.println("");
                toPrintableFloatArr(_free_imu_quaternions, 4, buffer);
                _reply(buffer, output_to < 1 ? 1 : output_to, to_node_id);
                memset(buffer, 0, sizeof(buffer));
            }
        }
        else if(cmd == 'z') {
            float val_array[19] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
            // uint8_t count = _serialBusyWait(); // Expects a char representing count
            uint8_t count = command.substring(1).toInt(); // grab the char after the 'z' char. It's the count.
            char big_buffer[APP_BUFFER_SIZE * 3];
            char* big_buffer_pointer;

            for(uint8_t i=0; i<count; i++) {
                freeIMU.getQ(_free_imu_quaternions, _free_imu_val);
                val_array[15] = freeIMU.sampleFreq;        
                //freeIMU.getValues(_free_imu_val);       
                val_array[7] = (_free_imu_val[3] * M_PI/180); // gyro X
                val_array[8] = (_free_imu_val[4] * M_PI/180); // gyro Y
                val_array[9] = (_free_imu_val[5] * M_PI/180); // gyro Z
                val_array[4] = (_free_imu_val[0]); // accel X
                val_array[5] = (_free_imu_val[1]); // accel Y
                val_array[6] = (_free_imu_val[2]); // accel Z
                val_array[10] = (_free_imu_val[6]); // mag X
                val_array[11] = (_free_imu_val[7]); // mag Y
                val_array[12] = (_free_imu_val[8]); // mag Z
                val_array[0] = (_free_imu_quaternions[0]); // q1
                val_array[1] = (_free_imu_quaternions[1]); // q2
                val_array[2] = (_free_imu_quaternions[2]); // q3
                val_array[3] = (_free_imu_quaternions[3]); // q4
                //val_array[15] = millis();
                val_array[16] = _free_imu_val[9]; // mag heading
                val_array[18] = _free_imu_val[11]; // motion detect value

                #if HAS_PRESS()
                   // with baro
                   val_array[17] = _free_imu_val[10]; // estimated altitude
                   val_array[13] = (freeIMU.getBaroTemperature());
                   val_array[14] = (freeIMU.getBaroPressure());
                #elif HAS_MPU6050()
                   val_array[13] = (freeIMU.DTemp/340.) + 35.;
                #elif HAS_MPU9150()  || HAS_MPU9250()
                   val_array[13] = ((float) freeIMU.DTemp) / 333.87 + 21.0;
                #elif HAS_LSM9DS0()
                    val_array[13] = 21.0 + (float) freeIMU.DTemp/8.; //degrees C
                #elif HAS_LSM9DS1()    
                    val_array[13] = ((float) freeIMU.DTemp/256. + 25.0); //degrees C
                #elif HAS_ITG3200()
                   val_array[13] = freeIMU.rt;
                #endif

                big_buffer_pointer = (char*) big_buffer;
                // serialPrintFloatArr(val_array,19);
                toPrintableFloatArr(val_array, 19, big_buffer_pointer);
                // #if HAS_GPS
                //   val_array[0] = (float) gps.hdop.value();
                //   val_array[1] = (float) gps.hdop.isValid();
                //   val_array[2] = (float) gps.location.lat();
                //   val_array[3] = (float) gps.location.lng();
                //   val_array[4] = (float) gps.location.isValid();
                //   val_array[5] = (float) gps.altitude.meters();
                //   val_array[6] = (float) gps.altitude.isValid();
                //   val_array[7] = (float) gps.course.deg();
                //   val_array[8] = (float) gps.course.isValid();
                //   val_array[9] = (float) gps.speed.kmph();
                //   val_array[10] = (float) gps.speed.isValid();
                //   val_array[11] = (float) gps.charsProcessed();
                //   // serialPrintFloatArr(val_array,12);
                //   // Serial.print('\n');
                //   smartDelay(20);
                // #else
                  // Serial.print('\n');
                _reply(big_buffer_pointer, output_to < 1 ? 1 : output_to, to_node_id);
                memset(big_buffer, 0, sizeof(big_buffer));
                // #endif
                //Add in for teensy and Arduino101
                delay(10);
            }
        } 
        else if(cmd == 'a') {
            //a
            float val_array[19] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
            // uint8_t count = _serialBusyWait(); // Expects a char representing count
            uint8_t count = command.substring(1).toInt(); // grab the char after the 'a' char. It's the count.
            String output = String("");
            char big_buffer[APP_BUFFER_SIZE * 3];
            char* big_buffer_pointer;

            for(uint8_t i=0; i < count; i++) {
                freeIMU.getQ(_free_imu_quaternions, _free_imu_val);
                val_array[15] = freeIMU.sampleFreq;

                val_array[7] = (_free_imu_val[3] * M_PI/180); // gyro X
                val_array[8] = (_free_imu_val[4] * M_PI/180); // gyro Y
                val_array[9] = (_free_imu_val[5] * M_PI/180); // gyro Z
                val_array[4] = (_free_imu_val[0]); // accel X
                val_array[5] = (_free_imu_val[1]); // accel Y
                val_array[6] = (_free_imu_val[2]); // accel Z
                val_array[10] = (_free_imu_val[6]); // mag X
                val_array[11] = (_free_imu_val[7]); // mag Y
                val_array[12] = (_free_imu_val[8]); // mag Z
                val_array[0] = kFilters[0].measureRSSI(_free_imu_quaternions[0]); // q1
                val_array[1] = kFilters[1].measureRSSI(_free_imu_quaternions[1]); // q2
                val_array[2] = kFilters[2].measureRSSI(_free_imu_quaternions[2]); // q3
                val_array[3] = kFilters[3].measureRSSI(_free_imu_quaternions[3]); // q4

                val_array[16] = _free_imu_val[9]; // mag heading
                val_array[18] = _free_imu_val[11]; // motion detect value
                        
                #if HAS_PRESS() 
                   // with baro
                   val_array[17] = _free_imu_val[10]; // estimated altitude
                   val_array[13] = (freeIMU.getBaroTemperature());
                   val_array[14] = (freeIMU.getBaroPressure());
                #elif HAS_MPU6050()
                   val_array[13] = (freeIMU.DTemp/340.) + 35.;
                #elif HAS_MPU9150()  || HAS_MPU9250()
                   val_array[13] = ((float) freeIMU.DTemp) / 333.87 + 21.0;
                #elif HAS_LSM9DS0()
                    val_array[13] = 21.0 + (float) freeIMU.DTemp/8.; //degrees C
                #elif HAS_LSM9DS1()    
                    val_array[13] = ((float) freeIMU.DTemp/256. + 25.0); //degrees C
                #elif HAS_ITG3200()
                   val_array[13] = freeIMU.rt;
                #endif
                // serialPrintFloatArr(val_array, 19);
                big_buffer_pointer = (char*) big_buffer;
                toPrintableFloatArr(val_array, 19, big_buffer_pointer);

                // #if HAS_GPS
                //   val_array[0] = (float) gps.hdop.value();
                //   val_array[1] = (float) gps.hdop.isValid();
                //   val_array[2] = (float) gps.location.lat();
                //   val_array[3] = (float) gps.location.lng();
                //   val_array[4] = (float) gps.location.isValid();
                //   val_array[5] = (float) gps.altitude.meters();
                //   val_array[6] = (float) gps.altitude.isValid();
                //   val_array[7] = (float) gps.course.deg();
                //   val_array[8] = (float) gps.course.isValid();
                //   val_array[9] = (float) gps.speed.kmph();
                //   val_array[10] = (float) gps.speed.isValid();
                //   val_array[11] = (float) gps.charsProcessed();
                //   // serialPrintFloatArr(val_array,12);
                //   Serial.print('\n');
                //   smartDelay(20);
                // #else
                  // Serial.print('\n');
                // #endif
                _reply(big_buffer_pointer, output_to < 1 ? 1 : output_to, to_node_id);
                memset(big_buffer, 0, sizeof(big_buffer));
            }
        }

        // #if HAS_EEPPROM
        //     #ifndef CALIBRATION_H

        // else if(cmd == 'c') {
        //     const uint8_t eepromsize = sizeof(float) * 6 + sizeof(int) * 6;
        //     while(Serial.available() < eepromsize) ; // wait until all calibration data are received
        //     EEPROM.write(FREEIMU_EEPROM_BASE, FREEIMU_EEPROM_SIGNATURE);
        //     for(uint8_t i = 1; i<(eepromsize + 1); i++) {
        //       EEPROM.write(FREEIMU_EEPROM_BASE + i, (char) Serial.read());
        //     }
        //     freeIMU.calLoad(); // reload calibration
        //     // toggle LED after calibration store.
        //     setRGB(0x00, 0xff, 0x00, false);
        //     delay(1000);
        //     setRGB(0x00, 0x00, 0x00, false);
        // } 
        // else if(cmd == 'x') {
        //     EEPROM.write(FREEIMU_EEPROM_BASE, 0); // reset signature
        //     freeIMU.calLoad(); // reload calibration
        // }

        //     #endif
        // #endif
        else if(cmd == 'C') { // check calibration values
            char big_buffer[APP_BUFFER_SIZE * 3];
            char* big_buffer_pointer = (char*) big_buffer;
            String output = String("");

            char* output_pointer;
            int current_size = 0;
            // Serial.print("acc offset: ");
            // Serial.print(freeIMU.acc_off_x);
            // Serial.print(",");
            // Serial.print(freeIMU.acc_off_y);
            // Serial.print(",");
            // Serial.print(freeIMU.acc_off_z);
            // Serial.print("\n");
            output = "acc offset: ";
            output += freeIMU.acc_off_x;
            output += ',';
            output += freeIMU.acc_off_y;
            output += ',';
            output += freeIMU.acc_off_z;

            output_pointer = const_cast<char*>(output.c_str());
            memset(big_buffer, 0, sizeof(big_buffer));
            current_size = strlen(output_pointer);
            memcpy(big_buffer_pointer, output_pointer, current_size);

            // _reply(big_buffer_pointer, output_to < 1 ? 1 : output_to, to_node_id);

            // Serial.print("magn offset: ");
            // Serial.print(freeIMU.magn_off_x);
            // Serial.print(",");
            // Serial.print(freeIMU.magn_off_y);
            // Serial.print(",");
            // Serial.print(freeIMU.magn_off_z);
            // Serial.print("\n");

            output = "\nmagn offset: ";
            output += freeIMU.magn_off_x;
            output += ',';
            output += freeIMU.magn_off_y;
            output += ',';
            output += freeIMU.magn_off_z;

            output_pointer = const_cast<char*>(output.c_str());
            memcpy(big_buffer_pointer, output_pointer + current_size, strlen(output_pointer));
            current_size += strlen(output_pointer);

            // _reply(buffer, output_to < 1 ? 1 : output_to, to_node_id);

            // Serial.print("acc scale: ");
            // Serial.print(freeIMU.acc_scale_x);
            // Serial.print(",");
            // Serial.print(freeIMU.acc_scale_y);
            // Serial.print(",");
            // Serial.print(freeIMU.acc_scale_z);
            // Serial.print("\n");

            output = "\nacc scale: ";
            output += freeIMU.acc_scale_x;
            output += ',';
            output += freeIMU.acc_scale_y;
            output += ',';
            output += freeIMU.acc_scale_z;

            output_pointer = const_cast<char*>(output.c_str());
            memcpy(big_buffer_pointer, output_pointer + current_size, strlen(output_pointer));
            current_size += strlen(output_pointer);

            // _reply(buffer, output_to < 1 ? 1 : output_to, to_node_id);

            // output = String("");

            // Serial.print("magn scale: ");
            // Serial.print(freeIMU.magn_scale_x);
            // Serial.print(",");
            // Serial.print(freeIMU.magn_scale_y);
            // Serial.print(",");
            // Serial.print(freeIMU.magn_scale_z);
            // Serial.print("\n");

            output = "\nmagn scale: ";
            output += freeIMU.magn_scale_x;
            output += ',';
            output += freeIMU.magn_scale_y;
            output += ',';
            output += freeIMU.magn_scale_z;

            output_pointer = const_cast<char*>(output.c_str());
            memcpy(big_buffer_pointer, output_pointer + current_size, strlen(output_pointer));

            _reply(big_buffer_pointer, output_to < 1 ? 1 : output_to, to_node_id);
            memset(big_buffer, 0, sizeof(big_buffer));
        }
        else if(cmd == 'd') { // debugging outputs
            String output = String("");
            char* buffer_pointer  = output_to > 1 ? _free_imu_network_data : _free_imu_serial_data;
            char result_buffer[APP_BUFFER_SIZE * 3];
            char* result_buffer_pointer;

            // while(1) {
                // freeIMU.getRawValues(_free_imu_raw_values);
                // sprintf(_free_imu_serial_data, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,", _free_imu_raw_values[0], _free_imu_raw_values[1], _free_imu_raw_values[2], _free_imu_raw_values[3], _free_imu_raw_values[4], _free_imu_raw_values[5], _free_imu_raw_values[6], _free_imu_raw_values[7], _free_imu_raw_values[8], _free_imu_raw_values[9], _free_imu_raw_values[10]);
                // Serial.print(_free_imu_serial_data);
                // Serial.print('\n');
                // freeIMU.getQ(_free_imu_quaternions, _free_imu_val);
                // serialPrintFloatArr(_free_imu_quaternions, 4);
                // Serial.println("");
                // freeIMU.getYawPitchRoll(_free_imu_ypr);
                // Serial.print("Yaw: ");
                // Serial.print(_free_imu_ypr[0]);
                // Serial.print(" Pitch: ");
                // Serial.print(_free_imu_ypr[1]);
                // Serial.print(" Roll: ");
                // Serial.print(_free_imu_ypr[2]);
                // Serial.println("");

                freeIMU.getRawValues(_free_imu_raw_values);
                sprintf(_free_imu_serial_data, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,", _free_imu_raw_values[0], _free_imu_raw_values[1], _free_imu_raw_values[2], _free_imu_raw_values[3], _free_imu_raw_values[4], _free_imu_raw_values[5], _free_imu_raw_values[6], _free_imu_raw_values[7], _free_imu_raw_values[8], _free_imu_raw_values[9], _free_imu_raw_values[10]);
                // Serial.print(_free_imu_serial_data);
                // Serial.print('\n');
                output = String(_free_imu_serial_data);
                output += '\n';
                // buffer = const_cast<char*>(output.c_str());
                // _reply(buffer, output_to < 1 ? 1 : output_to, to_node_id);

                // output = String("");
                // buffer = const_cast<char*>(output.c_str());

                freeIMU.getQ(_free_imu_quaternions, _free_imu_val);
                // serialPrintFloatArr(_free_imu_quaternions, 4);
                toPrintableFloatArr(_free_imu_quaternions, 4, buffer_pointer);
                // Serial.println("");
                // _reply(buffer, output_to < 1 ? 1 : output_to, to_node_id);
                output += String(buffer_pointer);
                output += '\n';

                // output = String("");

                freeIMU.getYawPitchRoll(_free_imu_ypr);
                // Serial.print("Yaw: ");
                // Serial.print(_free_imu_ypr[0]);
                // Serial.print(" Pitch: ");
                // Serial.print(_free_imu_ypr[1]);
                // Serial.print(" Roll: ");
                // Serial.print(_free_imu_ypr[2]);
                // Serial.println("");

                output += "Yaw: ";
                output += _free_imu_ypr[0];
                output += " Pitch: ";
                output += _free_imu_ypr[1];
                output += " Roll: ";
                output += _free_imu_ypr[2];

                // buffer = const_cast<char*>(output.c_str());
                // _reply(buffer, output_to < 1 ? 1 : output_to, to_node_id);
                result_buffer_pointer = const_cast<char*>(output.c_str());
                _reply(result_buffer_pointer, output_to < 1 ? 1 : output_to, to_node_id);
                memset(result_buffer_pointer, 0, sizeof(result_buffer_pointer));
                memset(buffer_pointer, 0, sizeof(buffer_pointer));
                output = "";

            // }
        }
        else if (cmd = 'D') {
            sendSampleLegacy(input_from, output_to, to_node_id);
        }
    }
    // SET_NODE_ID:0x0001 Where range is 0x0001 to 0xfffe. 0xffff is reserved for broadcasts.
    if (command.length() == 19 && command.startsWith("SET_NODE_ID:")) {
        int node_id = inputString.substring(14, 19).toInt(); // Discard 0x chars

        #ifdef DEBUG
            Serial.print("Setting local destination node ID to 0x");
            Serial.println(node_id, HEX);
        #endif

        if (node_id < 0x0001) node_id = 0x0001;
        if (node_id > 0xfffe) node_id = 0xfffe;

        setDestAddress(node_id);
        if (output_to > 0) {
            _reply("SET_NODE_ID:OK", output_to, to_node_id);
        }
    }

    // SET_DEST_ID:0x0001 Where range is 0x0001 to 0xfffe. 0xffff is reserved for broadcasts.
    else if (command.length() == 19 && command.startsWith("SET_DEST_ID:")) {
        int dest_id = inputString.substring(14, 19).toInt(); // Discard 0x chars

        if (dest_id < 0x0001) dest_id = 0x0001;
        if (dest_id > 0xfffe) dest_id = 0xfffe;

        #ifdef DEBUG
            Serial.print("Setting local destination node ID to 0x");
            Serial.println(dest_id, HEX);
        #endif

        setDestAddress(dest_id);
        if (output_to > 0) {
            _reply("SET_DEST_ID:OK", output_to, to_node_id);
        }
    }

    // SET_PAN_ID:0x1234 Where range is 0x0001 to 0xfffe. 0xffff is reserved.
    else if (command.length() == 18 && command.startsWith("SET_PAN_ID:")) {

        int pan_id = inputString.substring(13, 18).toInt(); // Discard 0x chars

        if (pan_id < 0x0001) pan_id = 0x0001;
        if (pan_id > 0xfffe) pan_id = 0xfffe;

        #ifdef DEBUG
            Serial.print("Setting PAN ID to 0x");
            Serial.println(pan_id, HEX);
        #endif

        setPanId(pan_id);
        if (output_to > 0) {
            _reply("SET_PAN_ID:OK", output_to, to_node_id);
        }
    }

        // SET_CHANNEL:0x0b Where range is 0x0b to 0x1a
    else if (command.length() == 17 && command.startsWith("SET_CHANNEL:")) {
        int channel = inputString.substring(14, 17).toInt(); // Discard 0x chars

        if (channel < 0x0b) channel = 0x0b;
        if (channel > 0x1a) channel = 0x1a;
        #ifdef DEBUG
            Serial.print("Setting local channel to 0x");
            Serial.println(channel, HEX);
        #endif

        setChannel(channel);
        if (output_to > 0) {
            _reply("SET_CHANNEL:OK", output_to, to_node_id);
        }
    }

    // SET_ENDPOINT:0x01 Where range is 0x01 to 0x0f
    else if (command.length() == 18 && command.startsWith("SET_ENDPOINT:")) {
        int endpoint = inputString.substring(15, 18).toInt(); // Discard 0x chars

        if (endpoint < 1) endpoint = 1;
        if (endpoint > 0x0f) endpoint = 0x0f;

        #ifdef DEBUG
            Serial.print("Setting local endpoint ID to 0x");
            Serial.println(endpoint, HEX);
        #endif
        setEndpoint(endpoint);
        if (output_to > 0) {
            _reply("SET_ENDPOINT:OK", output_to, to_node_id);
        }
    }

    else if (command.startsWith("SET_CONFIG")) {
        #ifdef DEBUG
            Serial.print("Setting networking config... ");
        #endif

        updateNetworkingConfig();

        #ifdef DEBUG
            Serial.println("OK");
        #endif

        if (output_to > 0) {
            _reply("SET_CONFIG:OK", output_to, to_node_id);
        }
    }

    

    // SET_RGB:0x00:0x00:0x00 Where range is 0x00 to 0xff
    else if (command.length() == 23 && command.startsWith("SET_RGB:")) {
        int r = command.substring(10,  12).toInt();
        int g = command.substring(15, 17).toInt();
        int b = command.substring(20, 23).toInt();

        if (r < 0) r = 0;
        if (g < 0) g = 0;
        if (b < 0) b = 0;

        if (r > 0xff) r = 0xff;
        if (g > 0xff) g = 0xff;
        if (b > 0xff) b = 0xff;

        #ifdef DEBUG
            Serial.print("Setting RGB to ");
            Serial.print(r);
            Serial.print(",");
            Serial.print(g);
            Serial.print(",");
            Serial.println(b);
        #endif

        setRGB(r, g, b, false);
    }
    // SET_HSV:0x000:0x00:0x00 where H range is 0x000 to 0x167 (359 dec), S range is 0x00 to 0x64 (100 dec), V range is 0x00 to 0x64 (100 dec)
    else if (command.length() == 24 && command.startsWith("SET_HSV:")) {
        int h = command.substring(10, 12).toInt();
        int s = command.substring(15, 17).toInt();
        int v = command.substring(20, 23).toInt();

        if (h < 0) h = 0;
        if (s < 0) s = 0;
        if (v < 0) v = 0;

        if (h > 359) h = 359;
        if (s > 100) s = 100;
        if (v > 100) v = 100;

        #ifdef DEBUG
            Serial.print("Setting HSV to ");
            Serial.print(h);
            Serial.print(",");
            Serial.print(s);
            Serial.print(",");
            Serial.println(v);
        #endif

        setHSV(h, s, v, false);
    }

    else if (command.startsWith("TEST_RGB")) {
        #ifdef DEBUG
            Serial.print("Testing RGB LED...");
        #endif
        rgbTest();
        #ifdef DEBUG
            Serial.println("OK");
        #endif
    }

    else if (command.startsWith("TEST_HSV")) {
        #ifdef DEBUG
            Serial.print("Testing HSV Method...");
        #endif
        hsvTest();
        #ifdef DEBUG
            Serial.println("OK");
        #endif
    }
}

void FemtoCore::_reply(char* message, byte output_to, int dest_node_id) {
    static char net_message[APP_BUFFER_SIZE];
    switch(output_to) {
        case 3:
            // To Network (broadcast);
            
            sprintf(net_message, "=%s", message);
            broadcast((char*)net_message);
            break;
        case 2:
            // To Network (send)
            sprintf(net_message, "=%s", message);
            send((char*)net_message, dest_node_id);
            break;
        case 1:
            // To Serial w/ new line
            Serial.println(message);
            break;
        default:
            // To Serial w/o new line
            Serial.print(message);
            break;
    }
}

char FemtoCore::_serialBusyWait() {
  while(!Serial.available()) {
    ; // do nothing until ready
  }
  return Serial.read();
}


/* *** Timer for TC5 (RGB Blue pin) - BOF *** */
/**
 * Timer example from "nonintetic"
 * See gitst https://gist.github.com/nonsintetic/ad13e70f164801325f5f552f84306d6f
 */
//this function gets called by the interrupt at <sampleRate>Hertz
void TC5_Handler (void) {

    FemtoCore::handleRGB();

    // END OF YOUR CODE
    TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
}

/* 
 *  TIMER SPECIFIC FUNCTIONS FOLLOW
 *  you shouldn't change these unless you know what you're doing
 */

//Configures the TC to generate output events at the sample frequency.
//Configures the TC in Frequency Generation mode, with an event output once
//each time the audio sample frequency period expires.
void tcConfigure(int sampleRate)
{
    // Enable GCLK for TCC2 and TC5 (timer counter input clock)
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
    while (GCLK->STATUS.bit.SYNCBUSY);

    tcReset(); //reset TC5

    // Set Timer counter Mode to 16 bits
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
    // Set TC5 mode as match frequency
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
    //set prescaler and enable TC5
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
    //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
    TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
    while (tcIsSyncing());

    // Configure interrupt request
    NVIC_DisableIRQ(TC5_IRQn);
    NVIC_ClearPendingIRQ(TC5_IRQn);
    NVIC_SetPriority(TC5_IRQn, 0);
    NVIC_EnableIRQ(TC5_IRQn);

    // Enable the TC5 interrupt request
    TC5->COUNT16.INTENSET.bit.MC0 = 1;
    while (tcIsSyncing()); //wait until TC5 is done syncing
} 

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing()
{
    return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter()
{
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
    while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC5 
void tcReset()
{
    TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (tcIsSyncing());
    while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}

/* *** Timer for TC5 (RGB Blue pin) - EOF *** */

/* *** Serial incomming (RX), Arduino built-in handler - BOF *** */
/**
 * Arduino built-in event handler for incomming (Rx) Serial data. In our case, data from Serial USB (CDC).
 * See https://www.arduino.cc/en/Tutorial/SerialEvent
 */
void serialEvent() {
    // Read from Serial input...

    while (Serial && Serial.available()) {

        char inChar = (char) Serial.read();

        inputString += inChar;

        if (inChar == '\n') {
            #ifdef DEBUG
                Serial.println("GLOBAL serialEvent() " + inputString);
            #endif

            FemtoCore::stringComplete = true;
            FemtoCore::handleSerialRx();

            break;
        }
        
    }
}

void resetBuffer(char* bufferData, int bufferSize) {
  char filler = (char) 0;
  
  memset(bufferData, filler, bufferSize);
  bufferData[bufferSize] = '\0';
}
/* *** Serial incomming (RX), Arduino built-in handler - EOF *** */