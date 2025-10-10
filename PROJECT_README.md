# Arduino IoT Weather Station Project

Een weerstation project voor de Arduino MKR WiFi 1010 met IoT Carrier, dat data publiceert naar Arduino IoT Cloud.

## Vereisten

- Arduino MKR WiFi 1010
- Arduino MKR IoT Carrier  
- PlatformIO (of Arduino IDE)
- Arduino IoT Cloud account

## Setup

### 1. Project Setup
```bash
git clone <repository-url>
cd ArduinoCore-samd
```

### 2. Arduino IoT Cloud Setup
1. Ga naar [Arduino IoT Cloud](https://create.arduino.cc/iot/)
2. Maak een nieuw Thing aan
3. Voeg de volgende variabelen toe:
   - `temperature` (float, read-only)
   - `humidity` (float, read-only) 
   - `pressure` (float, read-only)
   - `light` (int, read-only)
4. Configureer een MKR WiFi 1010 device
5. Noteer je Device Key

### 3. Secrets Configuration
1. Kopieer `src/arduino_secrets_template.h` naar `src/arduino_secrets.h`
2. Vul je WiFi credentials in:
   ```cpp
   #define SECRET_SSID "JouwWiFiNaam"
   #define SECRET_PASS "JouwWiFiWachtwoord"  
   #define SECRET_DEVICE_KEY "JouwDeviceKeyVanIoTCloud"
   ```

### 4. Build en Upload
Met PlatformIO:
```bash
pio run --target upload
```

## Functionaliteit

Dit project leest de volgende sensoren van de IoT Carrier:
- **Temperatuur** (HTS221 sensor)
- **Luchtvochtigheid** (HTS221 sensor)  
- **Luchtdruk** (LPS22HB sensor)
- **Lichtsterkte** (APDS9960 sensor)

De data wordt elke 10 seconden gepubliceerd naar Arduino IoT Cloud en getoond op het ingebouwde display.

## Bestandsstructuur

- `src/main.cpp` - Entry point
- `src/setup.cpp` - Initialisatie functies
- `src/loop.cpp` - Hoofdloop met sensor uitlezing
- `src/thingProperties.cpp/h` - Arduino IoT Cloud configuratie
- `src/arduino_secrets_template.h` - Template voor credentials
- `platformio.ini` - PlatformIO build configuratie

## Troubleshooting

### WiFi verbinding problemen
- Controleer SSID en wachtwoord in `arduino_secrets.h`
- Zorg dat de 2.4GHz band beschikbaar is (5GHz wordt niet ondersteund)

### IoT Cloud verbinding problemen  
- Controleer of de Device Key correct is
- Zorg dat je Thing variabelen overeenkomen met de code

### Build problemen
- Controleer of alle libraries correct geïnstalleerd zijn
- Probeer een `pio run --target clean` gevolgd door `pio run`