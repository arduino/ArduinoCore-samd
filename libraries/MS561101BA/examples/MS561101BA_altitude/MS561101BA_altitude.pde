/*
MS561101BA_altitude.pde - Computes altitude from sea level using pressure readings from the sensor.
The algorithm uses the Hypsometric formula as explained in http://keisan.casio.com/has10/SpecExec.cgi?path=06000000.Science%2F02100100.Earth%20science%2F12000300.Altitude%20from%20atmospheric%20pressure%2Fdefault.xml&charset=utf-8

Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>

Development of this code has been supported by the Department of Computer Science,
Universita' degli Studi di Torino, Italy within the Piemonte Project
http://www.piemonte.di.unito.it/


This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/


//#define DEBUG_V

#include <Wire.h>
//#include <DebugUtils.h>
#include <MS561101BA.h>


#define MOVAVG_SIZE 32

MS561101BA baro = MS561101BA();

float movavg_buff[MOVAVG_SIZE];
int movavg_i=0;

const float sea_press = 1013.25;
float press, temperature;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(1000);

  // Suppose that the CSB pin is connected to GND.
  // You'll have to check this on your breakout schematics
  baro.init(MS561101BA_ADDR_CSB_LOW); 
  delay(100);
  
  // populate movavg_buff before starting loop
  for(int i=0; i<MOVAVG_SIZE; i++) {
    movavg_buff[i] = baro.getPressure(MS561101BA_OSR_4096);
  }
}

void loop() {
  Serial.print(" temp: ");
  temperature = baro.getTemperature(MS561101BA_OSR_4096);
  Serial.print(temperature);
  Serial.print(" degC pres: ");
  
  press = baro.getPressure(MS561101BA_OSR_4096);
  pushAvg(press);
  press = getAvg(movavg_buff, MOVAVG_SIZE);
  Serial.print(press);
  Serial.print(" mbar altitude: ");
  Serial.print(getAltitude(press, temperature));
  Serial.println(" m");
}

float getAltitude(float press, float temp) {
  //return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
  return ((pow((sea_press / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
}

void pushAvg(float val) {
  movavg_buff[movavg_i] = val;
  movavg_i = (movavg_i + 1) % MOVAVG_SIZE;
}

float getAvg(float * buff, int size) {
  float sum = 0.0;
  for(int i=0; i<size; i++) {
    sum += buff[i];
  }
  return sum / size;
}
