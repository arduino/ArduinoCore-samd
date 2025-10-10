/*
  Azure CosmosDB Weather Data Client
  Header file for Azure cloud connectivity
*/

#ifndef THING_PROPERTIES_H
#define THING_PROPERTIES_H

#include "arduino_secrets.h"
#include <WiFiNINA.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>

// Sensor data variables
extern float humidity;
extern int light;
extern float pressure; 
extern float temperature;
extern String weather_report;

// Azure CosmosDB client
extern WiFiSSLClient wifiClient;
extern HttpClient azureClient;
// Azure functions
void initAzureConnection();
bool sendWeatherDataToAzure();
void updateAzureData();

#endif // THING_PROPERTIES_H