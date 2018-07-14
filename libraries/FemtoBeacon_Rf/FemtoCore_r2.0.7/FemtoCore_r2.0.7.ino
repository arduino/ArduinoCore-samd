#define Serial SERIAL_PORT_USBVIRTUAL // Our "Serial" object is a Serial USB object.


/**
 * FOR THE DONGLE: Comment out IS_FEMTOBEACON_COIN
 * FOR THE COIN:   Un-comment IS_FEMTOBEACON_COIN
 */
#define IS_FEMTOBEACON_COIN // Comment this out if uploading to FemtoBeacon dongle instead.

/** 
 * Include our FemtoCore library! 
 */
#include "FemtoCore.h"







/**
 * Is this a coin, or a dongle? Set a node address at 0x02 or higher for each coin.
 */

#ifdef IS_FEMTOBEACON_COIN
  int myNodeId = 0x02;
#else
  int myNodeId = 0x01;
#endif

#ifdef IS_FEMTOBEACON_COIN
  int destNodeId = 0x01; // Coin sends to dongle (0x01)
#else
  int destNodeId = 0x02; // Dongle sends to coin(s) (0x02 and up);
#endif







/**
 * Usage:
 * 
 * For this demo, plug in the dongle to one USB port, and the coin to another.
 * Use PuTTY or Minicom as serial monitor for the dongle (115200 baud).
 * Use the Arduino Serial monitor for the coin (with newline enabled, at 115200 baud, although SerialUSB doesn't do anything with the baud rate anyway).
 * Type "Hello" into the Arduino Serial monitor, then press enter. It should send "Hello" to the dongle's serial monitor.
 */

void setup() {
  while(!Serial);

  // Initialize our hardware!
  FemtoCore::init(

    myNodeId, // My node ID. Coins are 0x02 and up.
    destNodeId, // Destination node ID (to dongle)
    
    0x01, // The callback endpoint ID to use.
    0x01, // The broadcast PAN ID we belong to.
    0x1a, // The channel we are on (think walkie-talkie channels).

    "TestSecurityKey0" // The security key to use for AES encrypted comms.
  );

}

void loop() {
  
  FemtoCore::handleNetworking(); // Required to call SYS_TaskHandler() of LwMesh stack library (handles networking stuff).
  FemtoCore::handleSerial();
}

