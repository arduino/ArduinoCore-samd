#include <Wire.h>
#include "LiquidCrystal_I2C.h"

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup()
{
  Serial5.begin( 9600 ) ;
  
  lcd.init();

  lcd.backlight();
  lcd.print("Hello world :)")
}

void loop()
{
  
  delay(1000);
}
