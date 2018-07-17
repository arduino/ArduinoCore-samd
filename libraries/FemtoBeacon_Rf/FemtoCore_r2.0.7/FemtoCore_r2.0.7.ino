#define Serial SERIAL_PORT_USBVIRTUAL // Our "Serial" object is a Serial USB object.
#include "FemtoCore.h"

/**
 * Is this a coin, or a dongle? Set a node address at 0x02 or higher for each coin.
 */

bool is_coin = true;    // Is FemtoBeacon COIN     O
//bool is_coin = false; // Is FemtoBeacon DONGLE   =[]

int myNodeId = is_coin ? 0x02 : 0x01;
int destNodeId = is_coin ? 0x01 : 0x02;


/**
 * Usage:
 * 
 * For this demo, plug in the dongle to one USB port, and the coin to another.
 * Use PuTTY or Minicom as serial monitor for the dongle (115200 baud).
 * Use the Arduino Serial monitor for the dongle (with newline enabled, at 115200 baud, although SerialUSB doesn't do anything with the baud rate anyway).
 * Type ":v" into the Arduino Serial monitor, then press enter. It should send ":v" to the coin, and the coin will return the FreeIMU version it has.
 */

void setup() {
  #ifdef ENABLE_SERIAL
    while(!Serial);
  #endif
  // Initialize our hardware!
  FemtoCore::init(

    myNodeId, // My node ID. Coins are 0x02 and up.
    destNodeId, // Destination node ID (to dongle)
    
    0x01, // The callback endpoint ID to use.
    0x01, // The broadcast PAN ID we belong to.
    0x1a, // The channel we are on (think walkie-talkie channels).

    "TestSecurityKey0", // The security key to use for AES encrypted comms.
    is_coin // Are we a coin, or a dongle?
  );

}

void loop() {
  
  FemtoCore::handleNetworking(); // Required to call SYS_TaskHandler() of LwMesh stack library (handles networking stuff).
  FemtoCore::handleSerial();
}

