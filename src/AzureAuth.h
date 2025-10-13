#ifndef AZURE_AUTH_H
#define AZURE_AUTH_H

#include <Arduino.h>
#include <WiFiNINA.h>

// Base64 encode and decode functions for Arduino
String base64Encode(const String& input);
String base64Decode(const String& input);

// Simple HMAC-SHA256 for Azure CosmosDB authorization
String generateAzureAuthToken(const String& verb, const String& resourceType, 
                             const String& resourceLink, const String& dateTime, 
                             const String& masterKey);

// Generate RFC 1123 date string
String getRFC1123DateTime();

// URL encode string
String urlEncode(const String& str);

#endif