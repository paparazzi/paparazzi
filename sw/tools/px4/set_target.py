#!/usr/bin/python
import sys
import serial

print sys.argv[2]
portname = sys.argv[1]
if sys.argv[2] == "fbw" :
  line = "pprz0"
else :
  line = "pprz1"

#print line.rstrip() + " " + str(len(line))
ser = serial.Serial(portname)
ser.write(line)

