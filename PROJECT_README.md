# Arduino Azure Weather Station

Een weerstation project voor de Arduino MKR WiFi 1010 met IoT Carrier, dat weerdata direct naar **Azure CosmosDB** stuurt.

## Vereisten

- Arduino MKR WiFi 1010
- Arduino MKR IoT Carrier  
- PlatformIO (of Arduino IDE)
- Azure account met CosmosDB

## Voordelen van Azure integratie

✅ **Eigen controle** - Volledige controle over je data en dashboard  
✅ **Schaalbaarheid** - Azure services schalen automatisch  
✅ **Custom dashboard** - Bouw je eigen web dashboard met Azure tools  
✅ **Data Analytics** - Gebruik Azure Analytics voor geavanceerde data analyse  
✅ **Geen vendor lock-in** - Gebruik standaard REST APIs  

## Setup

### 1. Project Setup
```bash
git clone <repository-url>
cd ArduinoCore-samd
```

### 2. Azure CosmosDB Setup

#### Stap 1: Maak een CosmosDB Account
1. Ga naar [Azure Portal](https://portal.azure.com)
2. Klik "Create a resource" → "Databases" → "Azure Cosmos DB"
3. Kies **API**: "Core (SQL)" 
4. **Account Name**: bijv. `weather-station-db`
5. **Location**: kies dichtstbijzijnde regio
6. **Capacity mode**: "Provisioned throughput" (goedkoper voor kleine projecten)
7. Klik "Review + Create"

#### Stap 2: Maak Database en Container
1. In je CosmosDB account, ga naar "Data Explorer"
2. Klik "New Database":
   - **Database id**: `WeatherStation`
   - **Provision database throughput**: aan (400 RU/s)
3. Klik "New Container":
   - **Database id**: gebruik bestaande `WeatherStation`
   - **Container id**: `SensorData`
   - **Partition key**: `/deviceId`

#### Stap 3: Verkrijg Connection Details
1. Ga naar "Keys" in het menu
2. Noteer:
   - **URI**: `https://jouw-account.documents.azure.com:443/`
   - **Primary Key**: lange string beginnend met...

### 3. Arduino Secrets Configuration
1. Kopieer `src/arduino_secrets_template.h` naar `src/arduino_secrets.h`
2. Vul je gegevens in:
   ```cpp
   #define SECRET_SSID "JouwWiFiNaam"
   #define SECRET_PASS "JouwWiFiWachtwoord"
   
   #define AZURE_COSMOSDB_HOST "jouw-account.documents.azure.com"
   #define AZURE_COSMOSDB_KEY "type=master&ver=1.0&sig=JOUW_PRIMARY_KEY_HIER"
   #define AZURE_COSMOSDB_DATABASE "WeatherStation"
   #define AZURE_COSMOSDB_CONTAINER "SensorData"
   ```

### 4. Build en Upload
Met PlatformIO:
```bash
pio run --target upload
```

Monitor de Serial output:
```bash
pio device monitor
```

## Functionaliteit

### Sensor Data
Dit project leest elke 10 seconden:
- **Temperatuur** (°C) - HTS221 sensor
- **Luchtvochtigheid** (%) - HTS221 sensor  
- **Luchtdruk** (kPa) - LPS22HB sensor
- **Lichtsterkte** (lux) - APDS9960 sensor

### Azure Data Format
Data wordt verzonden als JSON naar CosmosDB:
```json
{
  "id": "1678901234567",
  "deviceId": "mkr-weather-station-001", 
  "timestamp": 1678901234,
  "temperature": 22.5,
  "humidity": 45.2,
  "pressure": 101.3,
  "light": 250,
  "weather_report": "Clear"
}
```

## Dashboard Development

### Optie 1: Azure Data Explorer
1. Ga naar je CosmosDB → "Data Explorer"
2. Query je data:
   ```sql
   SELECT * FROM c WHERE c.timestamp > DateTimeAdd("hour", -1, GetCurrentDateTime())
   ORDER BY c.timestamp DESC
   ```

### Optie 2: Power BI Dashboard  
1. Installeer Power BI Desktop
2. Verbind met CosmosDB als data source
3. Maak visualisaties van temperatuur, luchtvochtigheid trends

### Optie 3: Custom Web Dashboard
1. Gebruik CosmosDB REST API
2. Framework naar keuze (React, Vue, Angular)
3. Voorbeeld query endpoint:
   ```
   GET https://jouw-account.documents.azure.com/dbs/WeatherStation/colls/SensorData/docs
   ```

## Bestandsstructuur

- `src/main.cpp` - Entry point
- `src/setup.cpp` - Initialisatie functies  
- `src/loop.cpp` - Hoofdloop met sensor uitlezing
- `src/thingProperties.cpp/h` - Azure CosmosDB client
- `src/arduino_secrets_template.h` - Template voor credentials
- `platformio.ini` - PlatformIO build configuratie

## Troubleshooting

### WiFi verbinding problemen
- Controleer SSID en wachtwoord in `arduino_secrets.h`
- Zorg dat de 2.4GHz band beschikbaar is (5GHz wordt niet ondersteund)

### Azure verbinding problemen  
- Controleer CosmosDB Host URL (zonder https:// prefix)
- Verificeer Primary Key in authorization header
- Test met Azure portal of database/container bestaan
- Check Serial Monitor voor HTTP response codes

### Build problemen
- Controleer of alle libraries correct geïnstalleerd zijn
- Probeer `pio run --target clean` gevolgd door `pio run`

## Kosten

**Azure CosmosDB** (bij 1 meting per 10 seconden):
- Database: ~€5/maand (400 RU/s provisioned)  
- Storage: <€1/maand (sensor data is klein)
- **Totaal**: ~€6/maand voor continue monitoring

**Alternatief goedkoper**: Switch naar "Serverless" mode voor <€1/maand bij lage volumes.