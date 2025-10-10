#ifndef ARDUINO_SECRETS_H
#define ARDUINO_SECRETS_H

// WiFi network credentials
#define SECRET_SSID "YOUR_WIFI_NETWORK_NAME"
#define SECRET_PASS "YOUR_WIFI_PASSWORD"

// Azure CosmosDB credentials
// Get these from your Azure portal
#define AZURE_COSMOSDB_HOST "YOUR_COSMOSDB_ACCOUNT.documents.azure.com"
#define AZURE_COSMOSDB_KEY "type=master&ver=1.0&sig=YOUR_PRIMARY_KEY"
#define AZURE_COSMOSDB_DATABASE "WeatherStation"
#define AZURE_COSMOSDB_CONTAINER "SensorData"

#endif // ARDUINO_SECRETS_H