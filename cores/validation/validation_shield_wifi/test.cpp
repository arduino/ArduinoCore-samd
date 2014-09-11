/*
  Copyright (c) 2014 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*

 This example connects to an unencrypted Wifi network.
 Then it prints the  MAC address of the Wifi shield,
 the IP address obtained, and other network details.

 Circuit:
 * WiFi shield attached

 created 13 July 2010
 by dlf (Metodo2 srl)
 modified 31 May 2012
 by Tom Igoe
 */
 #include "Arduino.h"
 #include <SPI.h>
 #include <WiFi.h>

void printCurrentNet();
void printWifiData();
void httpRequest();

char ssid[] = "";     //  your network SSID (name)
char pass[] = "";  // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

WiFiClient client;

void setup() {
  WiFi = WiFiClass();
  client = WiFiClient();

  delay(500); // Waiting for initialization

//  SPI.begin();
  //Initialize SERIAL_PORT_MONITOR and wait for port to open:
  SERIAL_PORT_MONITOR.begin(9600);
  while (!SERIAL_PORT_MONITOR) {
    ; // wait for SERIAL_PORT_MONITOR port to connect. Needed for Leonardo only
  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    SERIAL_PORT_MONITOR.println("WiFi shield not present");
    // don't continue:
    while(true);
  }

 // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) {
    SERIAL_PORT_MONITOR.print("Attempting to connect to WPA SSID: ");
    SERIAL_PORT_MONITOR.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the data:
  SERIAL_PORT_MONITOR.print("You're connected to the network");
  printCurrentNet();
  printWifiData();

}

void loop() {
  // check the network connection once every 10 seconds:
  delay(10000);
  printCurrentNet();

  // Trying to connect to http://hasthelargehadroncolliderdestroyedtheworldyet.com/
  SERIAL_PORT_MONITOR.println("Trying to connect to : www.hasthelargehadroncolliderdestroyedtheworldyet.com :");
  httpRequest();
  while( client.available() )
  {
    SERIAL_PORT_MONITOR.print((char)(client.read()));
  }
  SERIAL_PORT_MONITOR.println("END");
}

void printWifiData() {
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
    SERIAL_PORT_MONITOR.print("IP Address: ");
  SERIAL_PORT_MONITOR.println(ip);
  SERIAL_PORT_MONITOR.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  SERIAL_PORT_MONITOR.print("MAC address: ");
  SERIAL_PORT_MONITOR.print(mac[5],HEX);
  SERIAL_PORT_MONITOR.print(":");
  SERIAL_PORT_MONITOR.print(mac[4],HEX);
  SERIAL_PORT_MONITOR.print(":");
  SERIAL_PORT_MONITOR.print(mac[3],HEX);
  SERIAL_PORT_MONITOR.print(":");
  SERIAL_PORT_MONITOR.print(mac[2],HEX);
  SERIAL_PORT_MONITOR.print(":");
  SERIAL_PORT_MONITOR.print(mac[1],HEX);
  SERIAL_PORT_MONITOR.print(":");
  SERIAL_PORT_MONITOR.println(mac[0],HEX);

}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  SERIAL_PORT_MONITOR.print("SSID: ");
  SERIAL_PORT_MONITOR.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  SERIAL_PORT_MONITOR.print("BSSID: ");
  SERIAL_PORT_MONITOR.print(bssid[5],HEX);
  SERIAL_PORT_MONITOR.print(":");
  SERIAL_PORT_MONITOR.print(bssid[4],HEX);
  SERIAL_PORT_MONITOR.print(":");
  SERIAL_PORT_MONITOR.print(bssid[3],HEX);
  SERIAL_PORT_MONITOR.print(":");
  SERIAL_PORT_MONITOR.print(bssid[2],HEX);
  SERIAL_PORT_MONITOR.print(":");
  SERIAL_PORT_MONITOR.print(bssid[1],HEX);
  SERIAL_PORT_MONITOR.print(":");
  SERIAL_PORT_MONITOR.println(bssid[0],HEX);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  SERIAL_PORT_MONITOR.print("signal strength (RSSI):");
  SERIAL_PORT_MONITOR.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  SERIAL_PORT_MONITOR.print("Encryption Type:");
  SERIAL_PORT_MONITOR.println(encryption,HEX);
  SERIAL_PORT_MONITOR.println();
}

void httpRequest() {
  // close any connection before send a new request.
  // This will free the socket on the WiFi shield
  client.stop();

  // if there's a successful connection:
  if (client.connect("www.hasthelargehadroncolliderdestroyedtheworldyet.com", 80)) {
    SERIAL_PORT_MONITOR.println("connecting...");
    // send the HTTP PUT request:
    client.println("GET / HTTP/1.1");
    client.println("Host: www.hasthelargehadroncolliderdestroyedtheworldyet.com");
    //client.println("User-Agent: ArduinoWiFi/1.1");
    client.println("Connection: close");
    client.println();

    if( client.connected() )
    {
      SERIAL_PORT_MONITOR.println("\tClient connected");
      while( client.available() == 0 )
      {
        //Waiting for data
        if( !client.connected() )
        {
          SERIAL_PORT_MONITOR.println("\tClient disconnected !");
          break;
        }
      }
    }
    else
    {
      SERIAL_PORT_MONITOR.println("\tClient not connected");
    }

  }
  else {
    // if you couldn't make a connection:
    SERIAL_PORT_MONITOR.println("\tconnection failed");
  }
}
