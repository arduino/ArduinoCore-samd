/**
   FemtoBeacon wirless IMU and LPS platform.
   Mesh networked IMU demo.

   @author A. Alibno <aalbino@femtoduino.com>
   @version 1.0.1
*/
/**
   Requires:
     https://github.com/femtoduino/FreeIMU-Updates libraries, 
     https://github.com/femtoduino/libraries-atmel-lwm,
     https://github.com/femtoduino/RTCZero
     avr/dtostrf header
     
   This sketch assumes the following FreeIMU.h values:

     - MARG should be 4 (DCM)
     - MAG_DEC needs to be set to your location's magnetic declination (degrees)
     - Calibrate your IMU using the FreeIMU GUI tool (should generate a calibration.h file, include alongside this sketch)


   On FemtoBeacon PRO:
   Pin 5    PA06(R) = Red
   Pin 6    PA07(G) = Green
   Pin 21   PA27(B) = Blue
   
   Pin 19   PA09/DIG1 = RF1
   Pin 20   PA12/DIG2 = RF2
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <Arduino.h>
#include <Wire.h>
#include <avr/dtostrf.h>
#include <SPI.h>

#include <RTCZero.h>


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

/** BEGIN RGB LED stuff **/
#define LED_R 5
#define LED_G 6
#define LED_B 21

class rgb_color {

  private:
    int my_r;
    int my_g;
    int my_b;
  public:
    rgb_color (int red, int green, int blue)
      :
        my_r(red),
        my_g(green),
        my_b(blue)
    {
    }

    int r() const {return my_r;}
    int b() const {return my_b;}
    int g() const {return my_g;}
};

/*instances of fader can fade between two colors*/
class fader {

  private:
    int r_pin;
    int g_pin;
    int b_pin;

  public:
    /* construct the fader for the pins to manipulate.
     * make sure these are pins that support Pulse
     * width modulation (PWM), these are the digital pins
     * denoted with a tilde(~) common are ~3, ~5, ~6, ~9, ~10 
     * and ~11 but check this on your type of arduino. 
     */ 
    fader( int red_pin, int green_pin, int blue_pin)
      :
        r_pin(red_pin),
        g_pin(green_pin),
        b_pin(blue_pin)
    {
    }

    /*fade from rgb_in to rgb_out*/
    void fade( const rgb_color& in,
               const rgb_color& out,
               unsigned n_steps = 256,  //default take 256 steps
               unsigned time    = 10)   //wait 10 ms per step
    {
      int red_diff   = out.r() - in.r();
      int green_diff = out.g() - in.g();
      int blue_diff  = out.b() - in.b();
      for ( unsigned i = 0; i < n_steps; ++i){
        /* output is the color that is actually written to the pins
         * and output nicely fades from in to out.
         */
        rgb_color output ( in.r() + i * red_diff / n_steps,
                           in.g() + i * green_diff / n_steps,
                           in.b() + i * blue_diff/ n_steps);
        /*put the analog pins to the proper output.*/
        if (output.r() > 0) {
          analogWrite( r_pin, 255 - output.r() );
        }

        if (output.g() > 0) {
          analogWrite( g_pin, 255 - output.g() );
        }

        if (output.b() > 0) {
          analogWrite( b_pin, 255 - output.b() );
        }
        
        delay(time);
      }
    }

};

fader f (LED_R, LED_G, LED_B);
/*colors*/
rgb_color yellow( 255, 128,   0 );
rgb_color orange( 255,  32,   0 );
rgb_color red   ( 255,   0,   0 );
rgb_color cyan  (   0, 128, 255 );
rgb_color blue  (   0,   0, 255 );
rgb_color purple( 128,   0, 255 );
rgb_color pink  ( 255,   0, 255 );
rgb_color green (   0, 255,   0 );
/** END RGB LED stuff **/


/** BEGIN Networking vars **/
extern "C" {
  void                      println(char *x) {
    Serial.println(x);
    Serial.flush();
  }
}

#ifdef NWK_ENABLE_SECURITY
#define APP_BUFFER_SIZE     (NWK_MAX_PAYLOAD_SIZE - NWK_SECURITY_MIC_SIZE)
#else
#define APP_BUFFER_SIZE     NWK_MAX_PAYLOAD_SIZE
#endif

// Address must be set to 1 for the first device, and to 2 for the second one.
#define APP_ADDRESS         3 // Each coin (node) should have a unique integer address > 1
#define DEST_ADDRESS        1 // The RF Dongle node uses Address 1 (this is of course, for simplicity's sake)
#define APP_ENDPOINT        1 // What callback endpoint number we are using.
#define APP_PANID           0x01
#define APP_SECURITY_KEY    "TestSecurityKey0"
#define APP_CHANNEL         0x1a

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
float eulers[3]; // Hold euler angles (360 deg).
float values[10];
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


volatile bool is_sensor_on = 1;
volatile bool is_timestamp_on = 1;
volatile bool is_wireless_ok = 1;

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

//  SYSCTRL->VREG.bit.RUNSTDBY = 1; // Regulator, run in normal mode when standby mode is activated.
//  SYSCTRL->DFLLCTRL.bit.RUNSTDBY = 1; // Enable the DFLL48M clock in standby mode!

  current_ms = millis();
  last_ms = current_ms;

  /*int wait = 0;
    while (!Serial) {
    wait++;

    if (wait > F_CPU) {
      break;
    }
    }*/

//  pinMode(PIN_INT, INPUT);
//  pinMode(PIN_FSYNC, OUTPUT);
//  digitalWrite(PIN_FSYNC, HIGH);
//  delay(10);
//  digitalWrite(PIN_FSYNC, LOW);

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

  // RTC initialization
  rtc.begin();

  // Sensor initialization
  setupSensors();

  // Setup interrupt/wake/sleep
  setupSleep();

  // LED setup
  setupLed();

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
  while (!Serial);


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
  PHY_SetChannel(APP_CHANNEL);
  PHY_SetRxState(true);
  NWK_OpenEndpoint(APP_ENDPOINT, receiveMessage);
}

void setupSensors() {
  Wire.begin();

  delay(10);
  
  sensors.init(true); // the parameter enable or disable fast mode
  delay(10);
}

void setupLed() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);  
}


void loop() {

#ifdef OUTPUT_SERIAL
//  Serial.print("*");
#endif
  //digitalWrite(PIN_FSYNC, HIGH);
  //digitalWrite(PIN_FSYNC, LOW);


  handleNetworking();
  
}

void handleLed() {
  /*fade colors*/
//  f.fade( red,    orange,   256, 5);
//  f.fade( orange, yellow,   256, 5);
//  f.fade( yellow, green,    256, 5);
//  f.fade( green,  cyan,     256, 5);
//  f.fade( cyan,   blue,     256, 5);
//  f.fade( blue,   pink,     256, 5);
//  f.fade( pink,   red,      256, 5);
}
void handleNetworking()
{
  SYS_TaskHandler();
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
  if (APP_ADDRESS > 1 && !send_message_busy && !shouldBeSleeping) {
    digitalWrite(LED_R, LOW);

//      #ifdef DEBUG
//      #ifdef OUTPUT_SERIAL
//          Serial.print("Node #");
//          Serial.print(APP_ADDRESS);
//          Serial.println(" handleNetworking() ->sendMessage()");
//      #endif
//      #endif
      // We are reading the sensors only when we can send data.
      // @TODO implement FIFO Stack of sensor data to transmit.

      if (is_sensor_on == true && is_wireless_ok == true) {
        digitalWrite(LED_R, HIGH);
        digitalWrite(LED_G, HIGH);
        digitalWrite(LED_B, LOW);
        handleSensors();
      }
      sendMessage();
  }

  
}

void handleSensors()
{
  byte bufferIndex = 0;

#ifdef DEBUG
#ifdef OUTPUT_SERIAL
  Serial.println("handleSensors()");
#endif
#endif
  current_ms = millis();

  sensors.getYawPitchRoll180(ypr);
  sensors.getEuler360deg(eulers);
  sensors.getValues(values);

//  baro = sensors.getBaroAlt();
//  temp = sensors.getBaroTemperature();
//  pressure = sensors.getBaroPressure();


  //// Use dtostrf?
  // Copy ypr to buffer.
  resetBuffer();

  // ...We need a wide enough size (8 should be enough to cover negative symbol and decimal).
  // ...Precision is 2 decimal places.

  // Timestamp YYYY-MM-DDTHH:II:SS.sss
  /*char tstamp[23];
    sprintf(tstamp,
    "%04d-%02d-%02dT%02d:%02d:%02d.%03d",
    rtc.getYear(), rtc.getMonth(), rtc.getDay(),
    rtc.getHours(), rtc.getMinutes(), rtc.getSeconds(),
    ((int) milliseconds * .001));
    //String timestamp = sprintf("%04d", rtc.getYear()); // + "-" + sprintf("%02d", rtc.getMonth()) + "-" + sprintf("%02d", rtc.getDay()) + "T" + sprintf("%02d", rtc.getHours())( + ":" sprintf("%02d", rtc.getMinutes()) + ":" + sprintf("%02d", rtc.getSeconds()) + "." + sprintf("%03d", (int)milliseconds * .001);
  */

  dtostrf(current_ms, 10, 0, &bufferData[bufferIndex]);
  bufferIndex += 10;
  bufferData[bufferIndex] = delimeter;

  // Sensor data:

  // ...Yaw
  ++bufferIndex;
  dtostrf(ypr[0], 8, 2, &bufferData[bufferIndex]);
  bufferIndex += 8;
  bufferData[bufferIndex] = delimeter;

  // ...Pitch
  ++bufferIndex;
  dtostrf(ypr[1], 8, 2, &bufferData[bufferIndex]);
  bufferIndex += 8;
  bufferData[bufferIndex] = delimeter;

  // ...Roll
  ++bufferIndex;
  dtostrf(ypr[2], 8, 2, &bufferData[bufferIndex]);
  bufferIndex += 8;
  bufferData[bufferIndex] = delimeter;

  // Euler 1
  ++bufferIndex;
  dtostrf(eulers[0], 8, 2, &bufferData[bufferIndex]);
  bufferIndex += 8;
  bufferData[bufferIndex] = delimeter;
  // Euler 2
  ++bufferIndex;
  dtostrf(eulers[1], 8, 2, &bufferData[bufferIndex]);
  bufferIndex += 8;
  bufferData[bufferIndex] = delimeter;
  // Euler 3
  ++bufferIndex;
  dtostrf(eulers[2], 8, 2, &bufferData[bufferIndex]);
  bufferIndex += 8;
  bufferData[bufferIndex] = delimeter;


  // Values[0]
  ++bufferIndex;
  dtostrf(values[0], 8, 2, &bufferData[bufferIndex]);
  bufferIndex += 8;
  bufferData[bufferIndex] = delimeter;

  // Values[1]
  ++bufferIndex;
  dtostrf(values[1], 8, 2, &bufferData[bufferIndex]);
  bufferIndex += 8;
  bufferData[bufferIndex] = delimeter;

  // Values[2]
  ++bufferIndex;
  dtostrf(values[2], 8, 2, &bufferData[bufferIndex]);
  bufferIndex += 8;
  bufferData[bufferIndex] = delimeter;

  // Values[3]
  ++bufferIndex;
  dtostrf(values[3], 8, 2, &bufferData[bufferIndex]);
  bufferIndex += 8;
  bufferData[bufferIndex] = delimeter;

  // Values[4]
  ++bufferIndex;
  dtostrf(values[4], 8, 2, &bufferData[bufferIndex]);
  bufferIndex += 8;
  bufferData[bufferIndex] = delimeter;

  // Values[5]
  ++bufferIndex;
  dtostrf(values[5], 8, 2, &bufferData[bufferIndex]);
  bufferIndex += 8;
  bufferData[bufferIndex] = delimeter;

  // Values[6]
  ++bufferIndex;
  dtostrf(values[6], 8, 2, &bufferData[bufferIndex]);
  bufferIndex += 8;
  bufferData[bufferIndex] = delimeter;

  // Values[7]
  ++bufferIndex;
  dtostrf(values[7], 8, 2, &bufferData[bufferIndex]);
  bufferIndex += 8;
  bufferData[bufferIndex] = delimeter;

  // Values[8]
  ++bufferIndex;
  dtostrf(values[8], 8, 2, &bufferData[bufferIndex]);
  bufferIndex += 8;
  bufferData[bufferIndex] = delimeter;

  // Values[9]
  ++bufferIndex;
  dtostrf(values[9], 8, 2, &bufferData[bufferIndex]);
//  bufferIndex += 8;
//  bufferData[bufferIndex] = delimeter;
  
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
  //Serial.println("sendMessage()");
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
  //Serial.print("sendMessageConfirm() req->status is ");
#endif
#endif

  send_message_busy = false;
  is_wireless_ok = false;
  if (NWK_NO_ACK_STATUS == req->status) {
    
    #ifdef OUTPUT_SERIAL
    Serial.println("NWK_NO_ACK_STATUS");
    #endif;
    
  } else if (NWK_NO_ROUTE_STATUS == req->status) {
    
    #ifdef OUTPUT_SERIAL
    Serial.println("NWK_NO_ROUTE_STATUS");
    #endif
  } else if (NWK_SUCCESS_STATUS == req->status) {
    network_error_count = 0;
    is_wireless_ok = true;
    
    //send_message_busy = false;
    #ifdef OUTPUT_SERIAL
        Serial.println("NWK_SUCCESS_STATUS");
    #endif
  
  } else {
    
    #ifdef OUTPUT_SERIAL
    Serial.print(" . ");
    #endif
    
    ++network_error_count;
    if (network_error_count > 100) {
      #ifdef OUTPUT_SERIAL
      Serial.println("NWK_ERROR_STATUS ...Going to sleep.");
      
      #endif
      network_error_count = 0;
      sleepMode();
    }
    
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
#endif;

  String str((char*)ind->data);

  if (str.length() > 0)
  {
    if (str.equals("RESET")) {
      // Reset
      sensors.RESET();
      sensors.RESET_Q();
      sensors.init(true);
      
    } else if (str.equals("SON")) {
      is_sensor_on = true;
    } else if (str.equals("SOFF")) {
      is_sensor_on = false;
    } else if (str.equals("TON")) {
      is_timestamp_on = true;
    } else if (str.equals("TOFF")) {
      is_timestamp_on = false;
    } else if (str.equals("TSET")) {
      // @TODO parse incomming timestamp string
      // and set day, month, year, hours, minutes, seconds, milliseconds.
      int spaceIndex = str.indexOf("T");
      String dmy = str.substring(0, spaceIndex);
      String hms = str.substring(spaceIndex + 1);

      int hyphenIndex = dmy.indexOf("-");
      int secondHyphenIndex = dmy.indexOf("-", hyphenIndex + 1);

      year = (byte)dmy.substring(0, hyphenIndex).toInt();
      month = (byte)dmy.substring(hyphenIndex, secondHyphenIndex).toInt();
      day = (byte)dmy.substring(secondHyphenIndex + 1).toInt();

      int colonIndex = hms.indexOf(":");
      int secondColonIndex = hms.indexOf(":", colonIndex + 1);

      hours = (byte)hms.substring(0, colonIndex).toInt();
      minutes = (byte)hms.substring(colonIndex, secondColonIndex).toInt();

      String secondFractional = hms.substring(secondColonIndex + 1);

      int dotIndex = secondFractional.indexOf(".");

      if (dotIndex > 0) {
        seconds = (byte)secondFractional.substring(0, dotIndex).toInt();

        // Add the board's millis() return value to this.
        milliseconds = secondFractional.substring(dotIndex + 1).toInt();
      } else {
        seconds = (byte)secondFractional.toInt();
      }

      rtc.setHours(hours);
      rtc.setMinutes(minutes);
      rtc.setSeconds(seconds);

      rtc.setDay(day);
      rtc.setMonth(month);
      rtc.setYear(year);

    } else {
      // Ignore...
    }
  }

#ifdef OUTPUT_SERIAL
  Serial.println(str);
#endif

  return true;
}

void setupSleep() {



  // Datasheet says when Framebuffer is cleared, PB00 is triggered.
  //  attachInterrupt(digitalPinToInterrupt(24), wakeUp, LOW); // Connects to PB00 (IRQ to AT86RF233 radio module)
  //
  //  // Set EIC (External Interrupt Controller) to wake up the MCU on an external interrupt from EIC channel 0, Pin PB00
  //  EIC->WAKEUP.reg = EIC_WAKEUP_WAKEUPEN0;
  //
  //  // Tie the
  //  pinMode(24, INPUT); // PB00
  
  

}

void sleepMode() {
  // Set internal AT86RF233 tranciever to SLEEP before
  // SAM R21 enters STANDBY
#ifdef OUTPUT_SERIAL
  Serial.println("Internal AT86RF233 going into standby mode...");
#endif
  // Set AT86RF233 Radio module to Standby mode.
  NWK_SleepReq();
  delay(1000);

  // @TODO Set the MPU-9250 to SLEEP mode
  //sensors.
  is_sensor_on = false;

  // Go to sleep
  // See https://github.com/arduino/ArduinoCore-samd/issues/142
  // Configure the regulator to run in normal mode when in standby mode
  // Otherwise it defaults to low power mode and can only supply 50uA
  shouldBeSleeping = true;

  // See https://github.com/arduino-libraries/RTCZero/blob/master/examples/SleepRTCAlarm/SleepRTCAlarm.ino
  // Add alarm to wake up
  // ...set alarm for 10 seconds into the future
    uint32_t uEpoch = rtc.getEpoch();
    uint32_t uNextEpoch = uEpoch + 120; // 2 minutes into the future

//  int CurrentTime = rtc.getSeconds();
//  int AlarmTime = CurrentTime + 10; // Add 10 seconds
//  AlarmTime = AlarmTime % 60; // Checks for roll over at 60 seconds, and corrects

  #ifdef OUTPUT_SERIAL
  Serial.print("Current time is ");
  Serial.print(uEpoch);
  Serial.print(". Next alarm time (epoch) is ");
  Serial.print(uNextEpoch);
  Serial.println(". Setting alarm...");
  Serial.end();
  
  Serial.println("SAM R21 Entering standby mode!");
  Serial.end();
  USBDevice.detach();
  delay(1000);
  #endif

  //  rtc.setAlarmEpoch(uNextEpoch);
  rtc.attachInterrupt(wakeUp);
//  rtc.setAlarmSeconds(AlarmTime); // Wake on the 30th second of every minute;
  rtc.setAlarmEpoch(uNextEpoch);
  rtc.enableAlarm(rtc.MATCH_SS); // Match seconds only


  rtc.standbyMode();

  wakeUp();
}

void wakeUp() {


  #ifdef OUTPUT_SERIAL
    USBDevice.attach();
    delay(1000);
    while(!Serial);
    Serial.begin(115200);
    Serial.println("Woke up... Continue Mesh networking.");
    Serial.end();
  #endif

  shouldBeSleeping = false;
  is_sensor_on = true;
  NWK_WakeupReq();
  delay(1000);

  #ifdef OUTPUT_SERIAL
  Serial.println("Networking re-enabled. Resetting sensors...");
  Serial.end();
  #endif

  sensors.RESET();
  sensors.RESET_Q();
  sensors.init(true);
  
  #ifdef OUTPUT_SERIAL
  Serial.println("Sensors reset.");
  Serial.end();
  #endif
  
/*
  // Setup Serial comms again.

  #ifdef OUTPUT_SERIAL
  USBDevice.attach();
  delay(1000); // Delay to allow USB stuff to happen...
  while (!Serial);
  Serial.begin(500000);
  

  Serial.println("WOKE UP.");
  delay(1000);
  #endif

  //  rtc.detachInterrupt();

  // Wake Up the AT86RF233 tranciever.
  //  NWK_WakeupReq();

  // @TODO Wake up the MPU-9250
  is_sensor_on = true;

  // Wake up micro controller
  shouldBeSleeping = false;
*/
}
