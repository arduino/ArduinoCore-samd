#include "arduino_secrets.h"
#include "thingProperties.h"
#include <Arduino_MKRIoTCarrier.h>
MKRIoTCarrier carrier; 


void app_setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500); 
 
  // Initialize Azure connection
  initAzureConnection();
  
  Serial.println("Azure CosmosDB Weather Station initialized");
  delay(1000);
  
  CARRIER_CASE = false;
  carrier.begin();
  carrier.display.setRotation(0);
  delay(1500);
}