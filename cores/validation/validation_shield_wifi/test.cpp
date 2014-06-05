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

char ssid[] = "AVRGUEST";     //  your network SSID (name) 
char pass[] = "MicroController";  // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

WiFiClient client;

void setup() {
  WiFi = WiFiClass();
  client = WiFiClient();

  delay(500); // Waiting for initialization

//  SPI.begin();
  //Initialize Serial5 and wait for port to open:
  Serial5.begin(9600); 
  while (!Serial5) {
    ; // wait for Serial5 port to connect. Needed for Leonardo only
  }
  
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial5.println("WiFi shield not present"); 
    // don't continue:
    while(true);
  } 
  
 // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) { 
    Serial5.print("Attempting to connect to WPA SSID: ");
    Serial5.println(ssid);
    // Connect to WPA/WPA2 network:    
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
   
  // you're connected now, so print out the data:
  Serial5.print("You're connected to the network");
  printCurrentNet();
  printWifiData();

}

void loop() {
  // check the network connection once every 10 seconds:
  delay(10000);
  printCurrentNet();

  // Trying to connect to http://hasthelargehadroncolliderdestroyedtheworldyet.com/
  Serial5.println("Trying to connect to : www.hasthelargehadroncolliderdestroyedtheworldyet.com :");
  httpRequest();
  while( client.available() )
  {
    Serial5.print((char)(client.read()));
  }
  Serial5.println("END");
}

void printWifiData() {
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
    Serial5.print("IP Address: ");
  Serial5.println(ip);
  Serial5.println(ip);
  
  // print your MAC address:
  byte mac[6];  
  WiFi.macAddress(mac);
  Serial5.print("MAC address: ");
  Serial5.print(mac[5],HEX);
  Serial5.print(":");
  Serial5.print(mac[4],HEX);
  Serial5.print(":");
  Serial5.print(mac[3],HEX);
  Serial5.print(":");
  Serial5.print(mac[2],HEX);
  Serial5.print(":");
  Serial5.print(mac[1],HEX);
  Serial5.print(":");
  Serial5.println(mac[0],HEX);
 
}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial5.print("SSID: ");
  Serial5.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);    
  Serial5.print("BSSID: ");
  Serial5.print(bssid[5],HEX);
  Serial5.print(":");
  Serial5.print(bssid[4],HEX);
  Serial5.print(":");
  Serial5.print(bssid[3],HEX);
  Serial5.print(":");
  Serial5.print(bssid[2],HEX);
  Serial5.print(":");
  Serial5.print(bssid[1],HEX);
  Serial5.print(":");
  Serial5.println(bssid[0],HEX);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial5.print("signal strength (RSSI):");
  Serial5.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial5.print("Encryption Type:");
  Serial5.println(encryption,HEX);
  Serial5.println();
}

void httpRequest() {
  // close any connection before send a new request.
  // This will free the socket on the WiFi shield
  client.stop();

  // if there's a successful connection:
  if (client.connect("www.hasthelargehadroncolliderdestroyedtheworldyet.com", 80)) {
    Serial5.println("connecting...");
    // send the HTTP PUT request:
    client.println("GET / HTTP/1.1");
    client.println("Host: www.hasthelargehadroncolliderdestroyedtheworldyet.com");
    //client.println("User-Agent: ArduinoWiFi/1.1");
    client.println("Connection: close");
    client.println();

    if( client.connected() )
    {
      Serial5.println("\tClient connected");
      while( client.available() == 0 )
      {
        //Waiting for data
        if( !client.connected() )
        {
          Serial5.println("\tClient disconnected !");
          break;
        }
      }
    }
    else
    {
      Serial5.println("\tClient not connected");
    }

  }
  else {
    // if you couldn't make a connection:
    Serial5.println("\tconnection failed");
  }
}
