#include "AzureAuth.h"
#include <sha/sha256.h>

// Base64 encoding characters
static const String base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

String base64Encode(const String& input) {
    String encoded = "";
    int val = 0, valb = -6;
    
    for (unsigned int i = 0; i < input.length(); i++) {
        val = (val << 8) + input[i];
        valb += 8;
        while (valb >= 0) {
            encoded += base64_chars[(val >> valb) & 0x3F];
            valb -= 6;
        }
    }
    
    if (valb > -6) {
        encoded += base64_chars[((val << 8) >> (valb + 8)) & 0x3F];
    }
    
    while (encoded.length() % 4) {
        encoded += '=';
    }
    
    return encoded;
}

String base64Decode(const String& input) {
    String decoded = "";
    int in_len = input.length();
    int i = 0;
    int in = 0;
    unsigned char char_array_4[4], char_array_3[3];

    while (in_len-- && (input[in] != '=') && 
           (isalnum(input[in]) || (input[in] == '+') || (input[in] == '/'))) {
        char_array_4[i++] = input[in]; in++;
        if (i == 4) {
            for (i = 0; i < 4; i++)
                char_array_4[i] = base64_chars.indexOf(char_array_4[i]);

            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

            for (i = 0; (i < 3); i++)
                decoded += (char)char_array_3[i];
            i = 0;
        }
    }

    if (i) {
        for (int j = i; j < 4; j++)
            char_array_4[j] = 0;

        for (int j = 0; j < 4; j++)
            char_array_4[j] = base64_chars.indexOf(char_array_4[j]);

        char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
        char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

        for (int j = 0; (j < i - 1); j++) decoded += (char)char_array_3[j];
    }

    return decoded;
}

String getRFC1123DateTime() {
    // Get current time from WiFi (seconds since Unix epoch)
    unsigned long epochTime = WiFi.getTime();
    
    if (epochTime == 0) {
        Serial.println("ERROR: No NTP time available!");
        return "";
    }
    
    // Convert epoch to proper RFC 1123 format
    // RFC 1123 format: "Sun, 06 Nov 1994 08:49:37 GMT"
    
    const char* daysOfWeek[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    const char* months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", 
                           "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    
    // Proper date calculation from epoch time
    unsigned long totalDays = epochTime / 86400;
    int dayOfWeek = (totalDays + 4) % 7; // Jan 1, 1970 was a Thursday (4)
    
    // Calculate year, month, day properly
    int year = 1970;
    unsigned long daysRemaining = totalDays;
    
    // Calculate year
    while (true) {
        int daysInYear = 365;
        if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
            daysInYear = 366; // Leap year
        }
        
        if (daysRemaining >= daysInYear) {
            daysRemaining -= daysInYear;
            year++;
        } else {
            break;
        }
    }
    
    // Calculate month and day
    int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    
    // Adjust February for leap year
    if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
        daysInMonth[1] = 29;
    }
    
    int month = 1;
    while (month <= 12 && daysRemaining >= daysInMonth[month - 1]) {
        daysRemaining -= daysInMonth[month - 1];
        month++;
    }
    
    int dayOfMonth = daysRemaining + 1;
    
    // Calculate time components
    unsigned long secondsToday = epochTime % 86400;
    int hours = secondsToday / 3600;
    int minutes = (secondsToday % 3600) / 60;
    int seconds = secondsToday % 60;
    
    // Build RFC 1123 string with exact format - Azure expects LOWERCASE!
    String rfc1123 = String(daysOfWeek[dayOfWeek]) + ", " +
                     (dayOfMonth < 10 ? "0" : "") + String(dayOfMonth) + " " +
                     String(months[month-1]) + " " +
                     String(year) + " " +
                     (hours < 10 ? "0" : "") + String(hours) + ":" +
                     (minutes < 10 ? "0" : "") + String(minutes) + ":" +
                     (seconds < 10 ? "0" : "") + String(seconds) + " GMT";
    
    // Azure expects lowercase date format!
    rfc1123.toLowerCase();
    
    Serial.print("RFC 1123 date: ");
    Serial.println(rfc1123);
    
    return rfc1123;
}

String urlEncode(const String& str) {
    String encoded = "";
    for (unsigned int i = 0; i < str.length(); i++) {
        char c = str.charAt(i);
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
            encoded += c;
        } else {
            encoded += '%';
            if (c < 16) encoded += '0';
            encoded += String(c, HEX);
        }
    }
    return encoded;
}

// Proper HMAC-SHA256 signature generation for Azure CosmosDB
String generateAzureAuthToken(const String& verb, const String& resourceType, 
                             const String& resourceLink, const String& dateTime, 
                             const String& masterKey) {
    
    // Use the exact same dateTime format as in the header (no lowercase conversion)
    // Azure CosmosDB expects the date in signature to match the x-ms-date header exactly
    
    // Create the string to sign according to Azure CosmosDB REST API spec
    String stringToSign = verb + "\n" +
                         resourceType + "\n" + 
                         resourceLink + "\n" +
                         dateTime + "\n" +
                         "\n"; // Empty string for additional headers
    
    Serial.print("String to sign: ");
    Serial.println(stringToSign);
    
    // First, decode the master key from base64
    String decodedKey = base64Decode(masterKey);
    Serial.print("Decoded key length: ");
    Serial.println(decodedKey.length());
    
    // Create HMAC-SHA256 hash using cryptosuite2
    Sha256.initHmac((uint8_t*)decodedKey.c_str(), decodedKey.length());
    Sha256.print(stringToSign);
    uint8_t* hmacResult = Sha256.resultHmac();
    
    Serial.print("HMAC result (first 8 bytes): ");
    for(int i = 0; i < 8; i++) {
        Serial.print(hmacResult[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Convert HMAC result to base64 string
    String hmacResultString = "";
    for (int i = 0; i < 32; i++) {
        hmacResultString += (char)hmacResult[i];
    }
    String signature = base64Encode(hmacResultString);
    
    return "type=master&ver=1.0&sig=" + signature;
}