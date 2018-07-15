#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
time.py - Tests the output coming from an Arduino with FreeIMU for speed. 
Load the Arduino with the FreeIMU_serial program.

Copyright (C) 2012 Fabio Varesano <fvaresano@yahoo.it>

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

"""

import time
import serial
from struct import unpack
from binascii import unhexlify
from subprocess import call

burst = 32

print "\n\nWelcome to the FreeIMU timer routine!\nCopyright © Fabio Varesano 2012.\nReleased under GPL v3 - See http://www.gnu.org/copyleft/gpl.html\n\n"

print "Please load the FreeIMU_serial program from the FreeIMU library examples on your Arduino. Once you correctly installed the FreeIMU library, the examples are available from File->Examples->FreeIMU in the Arduino IDE.\nWhen done, close the Arduino IDE and its serial monitor."
raw_input('Hit Enter to continue.')

arduino_port = raw_input('Insert the serial port which connects to the Arduino (See in the Arduino IDE Tools->Serial Port if in doubt): ')


# instantiate a serial port object. port gets opened by default, no need to explicitly open it.
ser = serial.Serial(
	port= raw_input,
	baudrate=38400,
        timeout=1
)

ser.flushInput()

if ser.isOpen():
  print "Arduino serial port opened correctly"
# we rely on the unhandled serial exception which will stop the program in case of problems during serial opening

ser.write("v") # ask version
print "\nFreeIMU library version informations:", 
print ser.readline()

print "\nThe program will now start sampling debugging values and timing them.\n"
raw_input('Hit Enter to continue.')


buff = [0.0 for i in range(9)]

start = time.time()
tot_readings = 0

try:
  print "Sampling from FreeIMU and timing readings"
  while True:
    ser.write("r" + str(burst))
    ser.readline()
    tot_readings = tot_readings + 1
    if(tot_readings % 100 == 0):
      tot_time = time.time() - start
      print "%d readings obtained. Frequency %f over %d seconds. Hit CTRL+C to interrupt." % (tot_readings, tot_readings / tot_time, tot_time)
      
      
except KeyboardInterrupt:
  ser.close()

