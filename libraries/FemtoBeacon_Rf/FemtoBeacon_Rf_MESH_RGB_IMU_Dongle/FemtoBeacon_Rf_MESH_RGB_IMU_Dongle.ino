/**
 * FemtoBeacon wirless IMU and LPS platform.
 * Mesh networked IMU demo. Uses Atmel's LWM library (ported for use w/ Arduino)
 *
 * @author A. Alibno <aalbino@femtoduino.com>
 * @version 1.0.1
 */

 /**
 * FemtoBeacon Dongle (r2.0.6 and higher) 
 * Arduino Pin  | Chip port   | Color channel
 * -------------+-------------+--------------
 * D5~          | PA06        | Red (Fake PWM via timer, as it's not an anlogue pin)
 * D6~          | PA07        | Green
 * D21          | PA27        | Blue
 * 
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <RTCZero.h>

#define Serial SERIAL_PORT_USBVIRTUAL

/** BEGIN Atmel's LightWeight Mesh stack. **/

    #define DEFAULT_ANTENNA 1 // 1 = Chip antenna, 2 = uFL antenna

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


/** BEGIN RGB Pins **/
volatile int LED_R = 5;
volatile int LED_G = 6;
volatile int LED_B = 21;

volatile bool LED_R_STATE = false;
volatile bool LED_G_STATE = false;
volatile bool LED_B_STATE = false;

volatile long LED_R_TICK = 0;
volatile long LED_G_TICK = 0;
volatile long LED_B_TICK = 0;

// Vary the Duty Cycle to mix colors!
volatile long MAX_DUTY_CYCLE = 48; // Should be fast enough to fake PWM the Red pin.

volatile long LED_R_DUTY_CYCLE = MAX_DUTY_CYCLE; // (Brightness: 1 = ON, MAX_DUTY_CYCLE = OFF)
volatile long LED_G_DUTY_CYCLE = MAX_DUTY_CYCLE;
volatile long LED_B_DUTY_CYCLE = MAX_DUTY_CYCLE;

volatile long LAST_TICK = 0;
volatile long CURRENT_TICK = 0;

bool RGB_TIMER_STATE = false;

static void setRGB(byte R, byte G, byte B);
static void setRGB(byte R, byte G, byte B, bool forceHandling);

//static void handleRGB();
//static void handleColorPWM(volatile int &led, volatile long &tick, volatile long &duty_cycle, volatile bool &state);

/** END RGB Pins **/

/* Create an RTC object */
RTCZero rtc;
volatile unsigned long milliseconds = 0;
volatile unsigned long last_ms = 0;
volatile unsigned long current_ms = 0;

volatile bool shouldBeSleeping = false;
volatile int network_error_count = 0;

byte seconds = 0;
byte minutes = 0;
byte hours = 0;

byte day = 1; // Day 1
byte month = 1; // January
byte year = 17; // 2017


char delimeter = ',';
char filler = (char) 0;



unsigned long start, finish, elapsed;

void setup() {
  
  // put your setup code here, to run once:

  current_ms = millis();
  last_ms = current_ms;
  
  setupRGB();
  setRGB(255, 0, 0); // Red
  setupSerialComms();
  
  setRGB(0, 0, 0, true); // Off
  Serial.print("Starting LwMesh...");
  setupMeshNetworking();
  
  delay(500);

  Serial.println("OK.");
  
  Serial.print("Starting RTC...");
  
  // RTC initialization
  rtc.begin();
  setRGB(96, 48, 0); // Yellow
  
  delay(100);
  Serial.println("OK.");
  setRGB(0, 255, 0); // Green
}

static void setupRGB() {
  tcConfigure(MAX_DUTY_CYCLE);
  tcStartCounter();
  
  CURRENT_TICK = micros();
  LAST_TICK = CURRENT_TICK;
  //pins driven by analogWrite do not need to be declared as outputs
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
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


static void setRGB(byte R, byte G, byte B, bool forceHandling) {
  // Map 0-255 to 1000-0
  LED_R_DUTY_CYCLE = map(R, 0, 255, MAX_DUTY_CYCLE, 1);
  LED_G_DUTY_CYCLE = map(G, 0, 255, MAX_DUTY_CYCLE, 1);
  LED_B_DUTY_CYCLE = map(B, 0, 255, MAX_DUTY_CYCLE, 1);

  if (forceHandling) {
    handleRGB();
  }
}

static void setRGB(byte R, byte G, byte B) {
  setRGB(R, G, B, true);
}

static void handleRGB() {
  CURRENT_TICK = micros();
  
  if (CURRENT_TICK > LAST_TICK + 1) {
    LAST_TICK = CURRENT_TICK;

    handleColorPWM(LED_R, LED_R_TICK, LED_R_DUTY_CYCLE, LED_R_STATE);
    handleColorPWM(LED_G, LED_G_TICK, LED_G_DUTY_CYCLE, LED_G_STATE);
    handleColorPWM(LED_B, LED_B_TICK, LED_B_DUTY_CYCLE, LED_B_STATE);
    
  }
}

static void handleColorPWM(volatile int &led, volatile long &tick, volatile long &duty_cycle, volatile bool &state) {

  if (duty_cycle >= MAX_DUTY_CYCLE)
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

    if (tick >= (MAX_DUTY_CYCLE - duty_cycle)) {
      state = true;
      tick = 0;

      
    }
  }
}

/**
 * Timer example from "nonintetic"
 * See gitst https://gist.github.com/nonsintetic/ad13e70f164801325f5f552f84306d6f
 */
//this function gets called by the interrupt at <sampleRate>Hertz
void TC5_Handler (void) {

  handleRGB();
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
