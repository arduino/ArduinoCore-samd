#include "Arduino.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


const uint8_t addressTemp = 0x4Ful;
const uint8_t addressLCD = 0x20ul;
const uint8_t addressRTC = 0x68ul;
uint16_t valueTemp = 0;
uint8_t a, b;

LiquidCrystal_I2C lcd(addressLCD,16,2);  // set the LCD address to 0x20(Cooperate with 3 short circuit caps) for a 16 chars and 2 line display

struct timeRTC
{
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  
  uint8_t dayMonth;
  uint8_t dayWeek;
  uint8_t month;
  uint8_t year;
} timeRtc;

// Convert normal decimal to numbers binary coded decimal
uint8_t decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
uint8_t bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}

void updateTime()
{
    Wire.beginTransmission(addressRTC);
    //Wire.write((uint8_t)0x3F);
    Wire.write((uint8_t)0x00);
    Wire.endTransmission();

    delay(10);

    Wire.requestFrom(addressRTC, 7);
    
    timeRtc.second = bcdToDec(Wire.read());
    timeRtc.minute = bcdToDec(Wire.read());
    timeRtc.hour = bcdToDec(Wire.read());
    
    timeRtc.dayWeek = bcdToDec(Wire.read());
    timeRtc.dayMonth = bcdToDec(Wire.read());
    timeRtc.month = bcdToDec(Wire.read());
    timeRtc.year = bcdToDec(Wire.read());
}

void setup()
{
  Serial5.begin( 115200 );
  Serial5.println("Wire init");
  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.home();

  pinMode(2, INPUT_PULLUP);
}

void LCDSpecialPrint(uint8_t value)
{
  if(value < 10)
  {
    lcd.print('0');
  }

  lcd.print(value);
}

void loop()
{
  Wire.beginTransmission(addressTemp);
    Wire.write((uint8_t) 0x00);
  Wire.endTransmission();

  delay(10);

  Wire.requestFrom(addressTemp, 2);
  Serial5.print((char)13);  // Erase current line
  Serial5.print("Temperature : ");

  a = Wire.read();
  b = Wire.read();

  valueTemp = a << 7;
  valueTemp |= b;
  valueTemp >>= 7;
  
  Serial5.print(a);
  Serial5.print(" | ");
  Serial5.print(b);
  
  updateTime();
  lcd.setCursor(0, 0);
  lcd.print(" ");
  LCDSpecialPrint(valueTemp);
  lcd.print((char)0xDF);
  lcd.print("C  ");

  LCDSpecialPrint(timeRtc.month);
  lcd.print("/");
  LCDSpecialPrint(timeRtc.dayMonth);
  lcd.print("/");
  LCDSpecialPrint(timeRtc.year);
  lcd.print(" ");

  lcd.setCursor(0, 1);
  lcd.print("    ");
  LCDSpecialPrint(timeRtc.hour);
  lcd.print(":");
  LCDSpecialPrint(timeRtc.minute);
  lcd.print(":");
  LCDSpecialPrint(timeRtc.second);
}
