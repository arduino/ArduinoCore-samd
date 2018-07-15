//*****************************************************************************
//**
//** This code is based on the example code in the original AP_Baro library
//**
//*****************************************************************************

#include <stdint.h>
//#include <FastSerial.h>
#include <SPI.h>
#include <AP_Baro_MS5611.h>
//#include <AP_Baro.h> // ArduPilot Mega ADC Library
#include <Arduino.h>
//FastSerialPort0(Serial);

AP_Baro_MS5611 baro;

void setup()
{  
	Serial.begin(115200);
	Serial.println("ArduPilot Mega MeasSense Barometer library test");

	delay(1000);

  pinMode(63, OUTPUT);
  digitalWrite(63, HIGH);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32); // 500khz for debugging, increase later

  baro.init();
}

void loop()
{
    int32_t pres;
    int32_t temp;

    update_and_print();
    delay(250);
}

void update_and_print()
{
    int32_t pres;
    float temp;

    baro.read();

    float press1 = baro.get_pressure()/100.;
    temp = baro.get_temperature()/100.;
    Serial.print( press1);Serial.print("    ");Serial.println(temp);

    delay(100);
}
