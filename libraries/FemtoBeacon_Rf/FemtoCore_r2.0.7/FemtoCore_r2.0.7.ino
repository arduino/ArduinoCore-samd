#define Serial SERIAL_PORT_USBVIRTUAL // Our "Serial" object is a Serial USB object.

// Open FemtoCore.h and comment out DEBUG and ENABLE_SERIAL if uploading sketch to a node that won't need SerialUSB data.
#include "FemtoCore.h"

/**
 * Is this a coin, or a dongle? Set a node address at 0x02 or higher for each coin.
 */

// true  = Is FemtoBeacon COIN     O
// false = Is FemtoBeacon DONGLE   =[]
bool is_coin = true;

int coinId = 0x02; // Default coin to send to. Also sets the Coin node ID if is_coin = true

int myNodeId = is_coin ? coinId : 0x01;
int destNodeId = is_coin ? 0x01 : coinId;


/**
 * Usage:
 * 
 * For this demo, plug in the dongle to one USB port, and the coin to another.
 * Use PuTTY or Minicom as serial monitor for the coin (115200 baud).
 * Use the Arduino Serial monitor for the dongle (with newline enabled, at 115200 baud, although SerialUSB doesn't do anything with the baud rate anyway).
 * 
 * Open up the Arduino Serial monitor on the dongle, and then power up the coin(s).
 * When you see "from:0x2:0x1:=INIT_COMPLETE", it means node 0x2 is ready for commands.
 * 
 * When the coin node is ready for commands, type ":v" into the Arduino Serial monitor, then press enter. 
 * It should send ":v" to the coin, and the coin will return the FreeIMU version it has.
 * 
 * To start flow of FemtoBeacon data, type "SET_REPEAT:0x01::D" (where 0x01 enables repeat, and :D is the command to repeat)
 * To stop flow of FemtoBeacon data, type "SET_REPEAT:0x00" (where 0x00 disables repeat)
 * 
 * 
 * See COMMANDS.md for full list of commands
 */

void setup() {

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

