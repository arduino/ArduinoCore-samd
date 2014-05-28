#include "Arduino.h"
#include <Wire.h>
const uint8_t addressTemp = 0x4Ful;
uint16_t temp = 0;
uint8_t a, b;
void setup()
{
  Serial5.begin( 115200 );
  Serial5.println("Wire init");
  Wire.begin();
}

void loop()
{
  Wire.beginTransmission(addressTemp);
    Wire.write((uint8_t) 0x00);
  Wire.endTransmission();

  delay(100);

  Wire.requestFrom(addressTemp, 2);
  Serial5.print((char)13);
  Serial5.print("Temperature : ");

  a = Wire.read();
  b = Wire.read();

  temp = b << 7;
  temp |= a;
  temp >>= 7;
  
  Serial5.print(temp);
}
