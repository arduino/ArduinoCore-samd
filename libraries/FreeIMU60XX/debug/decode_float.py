import time, string
import serial

from struct import unpack
from binascii import unhexlify

# Decode an hex representation of a float to a float
#
# See:
# http://stackoverflow.com/questions/1592158/python-convert-hex-to-float/1...
# http://stackoverflow.com/questions/4315190/single-precision-big-endian-f...
def decode_float(s):
  """Other possible implementation. Don't know what's better
  #from ctypes import *
  s = s[6:8] + s[4:6] + s[2:4] + s[0:2] # reverse the byte order
  i = int(s, 16)                   # convert from hex to a Python int
  cp = pointer(c_int(i))           # make this into a c integer
  fp = cast(cp, POINTER(c_float))  # cast the int pointer to a float pointer
  return fp.contents.value         # dereference the pointer, get the float
  """
  return unpack('<f', unhexlify(s))[0]


f = open("log.txt", "r")
w = open("log_quat.txt", "w")

for line in f:
  elements = line.split(",")
  if len(elements) == 5: #quaternions line
    for elem in elements[0:4]:
      #print elem
      val = decode_float(elem)
      w.write("%f" % (val))
      w.write(",")
    w.write("\n")
  else:
    w.write(line)
    
w.close()
f.close()