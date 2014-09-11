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

class Base
{
	public:
	virtual void f() = 0;
	virtual ~Base() {}
};

class Derived : public Base
{
	public:
	virtual void f() {};
	virtual ~Derived() {}
};


void setup() {
	// This will cause "pure virtual function call" error
	// __cxa_pure_virtual
	Derived *a = new Derived();
	a->f();
}

struct X {
	X() {
		Serial.print("Hello!");
	};
};

void foo() {
	// This will cause delayed static member initialization
	// __cxa_guard_acquire
	// __cxa_guard_release
	static X a;
}

void loop() {
	foo();
}
