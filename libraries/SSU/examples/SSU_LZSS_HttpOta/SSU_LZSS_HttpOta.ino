/*
    Small example sketch demonstrating how to perform OTA via HTTP/S
    utilizing a MKRGSM 1400 and the storage on the integrated
    SARA U-201 GSM module.

    Please, be careful because no verification is done on the
    received OTA file, apart size verification of the transmitted
    bytes using the HTTP Content-Length header.

    For production-grade OTA procedure you might want to implement
    a content verification procedure using a CRC calculation
    or an hash (eg. MD5 or SHA256) comparison.

    Circuit:
    * MKR GSM 1400 board
    * Antenna
    * SIM card with a data plan

    Steps to update a sketch:

    1) Create a new sketch or update an existing one to be updated over-the-air.
       The sketch needs to contain also the code below for future OTAs.
       The sketch must include the SSU library via
       #include <SSU.h>

    2) In the IDE select: Sketch -> Export compiled Binary.

    3) Open the location of the sketch (Sketch -> Show Sketch Folder) and compress it
       with a lzss tool
       (eg. https://github.com/arduino-libraries/ArduinoIoTCloud/blob/master/extras/tools/lzss.py).

    4) Upload the .lzss file to your HTTP/S server.

    5) Upload this sketch after configuring the server, port and filename variables.

    The sketch will download the OTA file, store it into the U-201 storage, and
    will reset the board to trigger the SSU update procedure.


    created 25 June 2020
    by Giampaolo Mancini
*/

#include <MKRGSM.h>

// This includes triggers the firmware update procedure
// in the bootloader after reset.
#include <SSU.h>

// Do not change! SSU will look for these files!
constexpr char UPDATE_FILE_NAME[] = "UPDATE.BIN.LZSS";
static constexpr char CHECK_FILE_NAME[] = "UPDATE.OK";

#include "arduino_secrets.h"
const char PINNUMBER[] = SECRET_PINNUMBER;
// APN data
const char GPRS_APN[] = SECRET_GPRS_APN;
const char GPRS_LOGIN[] = SECRET_GPRS_LOGIN;
const char GPRS_PASSWORD[] = SECRET_GPRS_PASSWORD;

// Change to GSMClient for non-SSL/TLS connection.
// Not recommended.
GSMSSLClient client;
GPRS gprs;
GSM gsmAccess;

GSMFileUtils fileUtils;

bool isHeaderComplete = false;
String httpHeader;

bool isDownloadComplete = false;
unsigned int fileSize = 0;
unsigned int totalWritten = 0;

constexpr char server[] = "example.org";
constexpr int port = 443;

// Name of the new firmware file to be updated.
constexpr char filename[] = "update.lzss";


void setup()
{
    unsigned long timeout = millis();

    Serial.begin(9600);
    while (!Serial && millis() - timeout < 5000)
        ;

    Serial.println("Starting OTA Update via HTTP and Arduino SSU.");
    Serial.println();

    bool connected = false;

    Serial.print("Connecting to cellular network... ");
    while (!connected) {
        if ((gsmAccess.begin(PINNUMBER) == GSM_READY) && (gprs.attachGPRS(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD) == GPRS_READY)) {
            connected = true;
        } else {
            Serial.println("Not connected");
            delay(1000);
        }
    }

    Serial.println("Connected.");
    Serial.println();

    // Modem has already been initialized in the sketch:
    // begin FileUtils without MODEM initialization.
    fileUtils.begin(false);

    Serial.print("Connecting to ");
    Serial.print(server);
    Serial.print(":");
    Serial.print(port);
    Serial.print("... ");
    if (client.connect(server, port)) {
        Serial.println("Connected.");
        Serial.print("Downloading ");
        Serial.println(filename);
        Serial.print("... ");
        // Make the HTTP request:
        client.print("GET /");
        client.print(filename);
        client.println(" HTTP/1.1");
        client.print("Host: ");
        client.println(server);
        client.println("Connection: close");
        client.println();
    } else {
        Serial.println("Connection failed");
    }
}

void loop()
{
    while (client.available()) {
        // Skip the HTTP header
        if (!isHeaderComplete) {
            const char c = client.read();
            httpHeader += c;
            if (httpHeader.endsWith("\r\n\r\n")) {
                isHeaderComplete = true;

                // Get the size of the OTA file from the
                // HTTP Content-Length header.
                fileSize = getContentLength();

                Serial.println();
                Serial.print("HTTP header complete. ");
                Serial.print("OTA file size is ");
                Serial.print(fileSize);
                Serial.println(" bytes.");
                if (fileSize == 0) {
                    Serial.println("Unable to get OTA file size.");
                    while (true)
                        ;
                }
            }
        } else {
            // Read the OTA file in len-bytes blocks to preserve RAM.
            constexpr size_t len { 512 };
            char buf[len] { 0 };

            // Read len bytes from HTTP client...
            uint32_t read = client.readBytes(buf, len);
            // and append them to the update file.
            uint32_t written = fileUtils.appendFile(UPDATE_FILE_NAME, buf, read);

            if (written != read) {
                Serial.println("Error while saving data.");
                while (true)
                    ;
            }

            // Update the received byte counter
            totalWritten += written;

            // Check for full file received and stored
            isDownloadComplete = totalWritten == fileSize;

            Serial.print("Received: ");
            Serial.print(totalWritten);
            Serial.print("/");
            Serial.println(fileSize);
        }
    }
    if (isDownloadComplete) {
        Serial.println();
        Serial.println("Download complete.");
        Serial.println("Enabling checkpoint.");
        Serial.println();

        // Create the checkpoint file: will be removed by SSU
        // after successful update.
        auto status = fileUtils.downloadFile(CHECK_FILE_NAME, { 0 }, 1);
        if (status != 1) {
            Serial.println("Unable to create checkpoint file.");
            while (true)
                ;
        }

        Serial.println("Resetting MCU in order to trigger SSU...");
        Serial.println();
        delay(500);
        NVIC_SystemReset();
    }
}

int getContentLength()
{
    const String contentLengthHeader = "Content-Length:";
    const auto contentLengthHeaderLen = contentLengthHeader.length();

    auto indexContentLengthStart = httpHeader.indexOf(contentLengthHeader);
    if (indexContentLengthStart < 0) {
        Serial.println("Unable to find Content-Length header (Start)");
        return 0;
    }
    auto indexContentLengthStop = httpHeader.indexOf("\r\n", indexContentLengthStart);
    if (indexContentLengthStart < 0) {
        Serial.println("Unable to find Content-Length header (Stop)");
        return 0;
    }
    auto contentLength = httpHeader.substring(indexContentLengthStart + contentLengthHeaderLen + 1, indexContentLengthStop);

    contentLength.trim();
    return contentLength.toInt();
}
