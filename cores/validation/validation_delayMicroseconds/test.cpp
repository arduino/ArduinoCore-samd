/*
  Copyright (c) 2014 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Arduino.h"

void delay_1121(unsigned int us);
void delay_1678(unsigned int us);

#define DM1(x) delayMicroseconds(x)

float loopTime = 0.0;
float cumulativeError = 0.0;

#define LOOPS 10000

void detectLoopTime() {
	Serial.flush();
	unsigned long start = micros();
	for (register uint16_t i = 0; i < LOOPS; i++) {
		__asm__ volatile ("");
	}
	unsigned long end = micros();
	loopTime = ((float)(end - start));
	Serial.println("Loop time us  = " + String(loopTime, 5));

	Serial.flush();
	start = micros();
	for (register uint16_t i = 0; i < LOOPS; i++) {
		asm volatile ("nop");
	}
	end = micros();
	float nopTime = ((float)(end - start - loopTime)) / ((float)(LOOPS)); // us
	Serial.println("          NOP = " + String(nopTime, 5));
}

//static inline void delay1us() __attribute__((always_inline, unused));
//static inline void delay1us() {
	//asm volatile ("nop");
	//asm volatile ("nop");
	//asm volatile ("nop");
	//asm volatile ("nop");
//
	//asm volatile ("nop");
	//asm volatile ("nop");
	//asm volatile ("nop");
	//asm volatile ("nop");
//
	//asm volatile ("nop");
	//asm volatile ("nop");
	//asm volatile ("nop");
	//asm volatile ("nop");
//
	//asm volatile ("nop");
	//asm volatile ("nop");
	//asm volatile ("nop");
	//asm volatile ("nop");
//}

//#define D1US() delay1us()
//#define D4US() D1US();D1US();D1US();D1US();
//#define D20US() D4US();D4US();D4US();D4US();D4US();
//#define D100US() D20US();D20US();D20US();D20US();D20US();


#define ConstantTest(x) {                              \
	Serial.flush();                                    \
	unsigned long start = micros();                    \
	for (int i = 0; i < LOOPS; i++) {                  \
		DM1(x);                                          \
	}                                                  \
	unsigned long end = micros();                      \
	float delta = ((float)(end - start - loopTime));   \
	delta /= (float)(LOOPS);                           \
	Serial.print(String(delta, 5));                    \
	cumulativeError += abs(delta-x);                   \
}

void GenericTest(unsigned int d) {
	Serial.flush();
	unsigned long start = micros();
	for (int i = 0; i < LOOPS; i++) {
		DM1(d);
	}
	unsigned long end = micros();
	float delta = ((float)(end - start - loopTime));
	delta /= (float)(LOOPS);
	Serial.print(String(delta, 5));
	cumulativeError += abs(delta-d);
}

void VolatileTest(volatile unsigned int d) {
	Serial.flush();
	unsigned long start = micros();
	for (int i = 0; i < LOOPS; i++) {
		DM1(d);
	}
	unsigned long end = micros();
	float delta = ((float)(end - start - loopTime));
	delta /= (float)(LOOPS);
	Serial.print(String(delta, 5));
	cumulativeError += abs(delta-d);
}

#define TEST(x) {                  \
	if (x < 10) Serial.print(" "); \
	Serial.print(x);               \
	Serial.print("uS\t");          \
	ConstantTest(x);               \
	Serial.print(" \t");           \
	GenericTest(x);                \
	Serial.print(" \t");           \
	VolatileTest(x);               \
	Serial.println();              \
}

void setup() {
	Serial.begin(115200);

	detectLoopTime();
	TEST(1);
	TEST(2);
	TEST(3);
	TEST(4);
	TEST(5);
	TEST(10);
	TEST(0);
	
	Serial.println(cumulativeError);
}

void loop() { }

