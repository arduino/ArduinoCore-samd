
#include "arduino_secrets.h"
#include "thingProperties.h"
#include <Arduino_MKRIoTCarrier.h>

extern MKRIoTCarrier carrier;

void app_loop() {
  ArduinoCloud.update();
  carrier.Buttons.update();
 
  while(!carrier.Light.colorAvailable()) {
    delay(5);
  }
  int none;
  carrier.Light.readColor(none, none, none, light);
  
  temperature = carrier.Env.readTemperature();
  humidity = carrier.Env.readHumidity();
  pressure = carrier.Pressure.readPressure();
 
 
  if (carrier.Buttons.onTouchDown(TOUCH0)) {
    carrier.display.fillScreen(ST77XX_WHITE);
    carrier.display.setTextColor(ST77XX_RED);
    carrier.display.setTextSize(2);
 
    carrier.display.setCursor(30, 110);
    carrier.display.print("Temp: ");
    carrier.display.print(temperature);
    carrier.display.print(" C");
  }
 
  if (carrier.Buttons.onTouchDown(TOUCH1)) {
    carrier.display.fillScreen(ST77XX_WHITE);
    carrier.display.setTextColor(ST77XX_RED);
    carrier.display.setTextSize(2);
 
    carrier.display.setCursor(30, 110);
    carrier.display.print("Humi: ");
    carrier.display.print(humidity);
    carrier.display.print(" %");
  }
 
  if (carrier.Buttons.onTouchDown(TOUCH2)) {
    carrier.display.fillScreen(ST77XX_WHITE);
    carrier.display.setTextColor(ST77XX_RED);
    carrier.display.setTextSize(2);
 
    carrier.display.setCursor(30, 110);
    carrier.display.print("Light: ");
    carrier.display.print(light);
  }
 
  if (carrier.Buttons.onTouchDown(TOUCH3)) {
    carrier.display.fillScreen(ST77XX_WHITE);
    carrier.display.setTextColor(ST77XX_RED);
    carrier.display.setTextSize(2);
 
    carrier.display.setCursor(30, 110);
    carrier.display.print("Pressure: ");
    carrier.display.print(pressure);
    
  }
 
  if (humidity >= 60 && temperature >= 15) {
    weather_report = "It is very humid outside";
    
  }else if (temperature >= 15 && light >= 700) {
    weather_report = "Warm and sunny outside";
    
  }else if (temperature <= 16 && light >= 700) {
    weather_report = "A little cold, but sunny outside";
  }
  
  else{
   weather_report = "Weather is normal"; 
  }
 
}