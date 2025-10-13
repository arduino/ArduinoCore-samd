/* 
  Azure CosmosDB Weather Data Client
  Sends weather station data to Azure CosmosDB via REST API
*/

#include "thingProperties.h"
#include "AzureAuth.h"

// Sensor data variables
float humidity = 0.0f;
int light = 0;
float pressure = 0.0f;
float temperature = 0.0f;
String weather_report = "";

// Azure CosmosDB HTTP client
WiFiSSLClient wifiClient;
HttpClient azureClient = HttpClient(wifiClient, AZURE_COSMOSDB_HOST, 443);

unsigned long lastDataSend = 0;
const unsigned long sendInterval = 10000; // Send every 10 seconds

void initAzureConnection(){
  // Initialize WiFi connection
  WiFi.begin(SECRET_SSID, SECRET_PASS);
  
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected to WiFi. IP: ");
  Serial.println(WiFi.localIP());
  
  // Synchronize time with NTP
  Serial.print("Synchronizing time with NTP...");
  unsigned long startTime = millis();
  while (WiFi.getTime() == 0 && (millis() - startTime) < 20000) { // Increased timeout to 20 seconds
    delay(500); // Longer delay between checks
    Serial.print(".");
  }
  if (WiFi.getTime() > 0) {
    Serial.println();
    Serial.print("Time synchronized: ");
    Serial.println(WiFi.getTime());
  } else {
    Serial.println(" Failed to sync time - using fallback");
  }
  
  // Set HTTP client timeout
  azureClient.setTimeout(10000);
}

bool sendWeatherDataToAzure() {
  // Create JSON payload with current timestamp
  JsonDocument doc;
  doc["id"] = String("mkr-") + String(millis()); // Unique ID for CosmosDB
  doc["deviceId"] = "mkr-weather-station-001";
  doc["timestamp"] = WiFi.getTime();
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["pressure"] = pressure;
  doc["light"] = light;
  doc["weather_report"] = weather_report;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  Serial.println("Sending to Azure CosmosDB:");
  Serial.println(jsonString);
  
  // Get current date/time in RFC 1123 format
  String dateTime = getRFC1123DateTime();
  
  // Prepare Azure CosmosDB REST API request
  String resourceType = "docs";
  String resourceLink = "dbs/" + String(AZURE_COSMOSDB_DATABASE) + "/colls/" + String(AZURE_COSMOSDB_CONTAINER);
  String path = "/" + resourceLink + "/docs";
  
  // Generate authorization token - Azure expects lowercase verb!
  String authToken = generateAzureAuthToken("post", resourceType, resourceLink, dateTime, AZURE_COSMOSDB_KEY);
  
  azureClient.beginRequest();
  azureClient.post(path);
  azureClient.sendHeader("Content-Type", "application/json");
  azureClient.sendHeader("Authorization", authToken);
  azureClient.sendHeader("x-ms-date", dateTime);
  azureClient.sendHeader("x-ms-version", "2018-12-31");
  azureClient.sendHeader("x-ms-documentdb-partitionkey", "[\"mkr-weather-station-001\"]");
  azureClient.sendHeader("Content-Length", jsonString.length());
  azureClient.print(jsonString);
  azureClient.endRequest();
  
  // Read response
  int statusCode = azureClient.responseStatusCode();
  String response = azureClient.responseBody();
  
  Serial.print("Azure Response Code: ");
  Serial.println(statusCode);
  Serial.print("Response: ");
  Serial.println(response);
  
  return (statusCode == 201); // Created
}

void updateAzureData() {
  if (millis() - lastDataSend >= sendInterval) {
    if (sendWeatherDataToAzure()) {
      Serial.println("✓ Weather data sent to Azure successfully");
    } else {
      Serial.println("✗ Failed to send weather data to Azure");
    }
    lastDataSend = millis();
  }
}

