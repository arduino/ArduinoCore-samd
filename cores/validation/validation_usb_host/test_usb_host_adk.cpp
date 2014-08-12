/*
  Copyright (c) 2012 Arduino.  All right reserved.

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

#define ARDUINO_MAIN
//#include "variant.h"
#include "Arduino.h" 
#include <stdio.h>
#include <adk.h>


USBHost usb;
ADK adk(&usb,"Arduino SA",
            "Arduino_Terminal",
            "Arduino Terminal for Android",
            "1.0",
            "http://labs.arduino.cc/uploads/ADK/ArduinoTerminal/ThibaultTerminal_ICS_0001.apk",
            //"http://www.circuitsathome.com",
            "1");

void setup(void)
{
  SERIAL_PORT_MONITOR.begin( 115200 );
  while (!SERIAL_PORT_MONITOR); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  SERIAL_PORT_MONITOR.println("\r\nADK demo start");

  if (usb.Init() == -1)
	SERIAL_PORT_MONITOR.println("OSC did not start.");

  delay(20);
}

#define RCVSIZE 128



void loop1(void)
{
  uint8_t rcode;
  uint8_t msg[64] = { 0x00 };
  const char* recv = "Received: "; 

   usb.Task();
   
   if( adk.isReady() == false ) {
     return;
   }
   uint16_t len = 64;
   
   SERIAL_PORT_MONITOR.println("ADK Ready");
   rcode = adk.RcvData((uint8_t *)&len, msg);
   if( rcode & ( rcode != USB_ERRORFLOW )) {
     SERIAL_PORT_MONITOR.print("Data rcv. :");
     SERIAL_PORT_MONITOR.println(rcode, DEC );
   } 
   if(len > 0) {
     SERIAL_PORT_MONITOR.println("Data Packet.");

    for( uint8_t i = 0; i < len; i++ ) {
      SERIAL_PORT_MONITOR.print((char)msg[i]);
    }
    /* sending back what was received */    
    rcode = adk.SndData( strlen( recv ), (uint8_t *)recv );    
    rcode = adk.SndData( strlen(( char * )msg ), msg );

   }//if( len > 0 )...

   delay( 1000 );       
}


void loop(void)
{
	uint8_t buf[RCVSIZE];
	uint32_t nbread = 0;
	char helloworld[] = "Hello World!\r\n";

	usb.Task();

	if( adk.isReady() == false ) {
		return;
	}
	/* Write hello string to ADK */
	adk.SndData(strlen(helloworld), (uint8_t *)helloworld);

	delay(1000);

	/* Read data from ADK and print to UART */
	adk.RcvData((uint8_t *)&nbread, buf);
	if (nbread > 0)
	{
		SERIAL_PORT_MONITOR.print("RCV: ");
		for (uint32_t i = 0; i < nbread; ++i)
		{
			SERIAL_PORT_MONITOR.print((char)buf[i]);
		}
		SERIAL_PORT_MONITOR.print("\r\n");
	}	
}


// void loop2(void)
// {
//   uint8_t msg[3];
//  
// //	if (acc.isConnected()) {
//                 SERIAL_PORT_MONITOR.print("Accessory connected. ");
// 		int len = adk.RcvData(msg, (uint8_t)sizeof(msg));  //, 1);
//                 SERIAL_PORT_MONITOR.print("Message length: ");
//                 SERIAL_PORT_MONITOR.println(len, DEC);
//         }
//  
// 	delay(100);
// }