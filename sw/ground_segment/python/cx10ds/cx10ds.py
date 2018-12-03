#!/usr/bin/env python
#
# MIT License
#
# Copyright (c) 2018 Nagy Arpad Peter
# Copyright (c) 2018 Gautier Hattenberger (for Paparazzi UAV)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import socket
from time import sleep

class CX10DS:
    def __init__(self, verbose=False):
        self.verbose = verbose

        self.IP = "192.168.4.1"
        self.PORT = 8033
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.header = b'\xcc' #first byte of every payload
        self.footer = b'\x33' # last byte of every payload
        self.message = ''

        self.throttle = 128 # controls the throttle range 0-255
        self.rudder = 128 # controls the rudder range 48-208 ?? CHECK
        self.aileron = 128 #todo: range check
        self.elevator = 128 #todo: range check
        self.crc = 0 #crc calculated from thr, rdr, ail, elev, mode xor product
        self.mode = 0 # 0 = idle, 1 = takeoff, 2 = land

        self.trim_roll = 0
        self.trim_pitch = 0
        self.trim_yaw = 0

    def __del__(self):
        self.sock.close()

    def set_cmd(self, aileron, elevator, rudder, throttle, mode):
        self.throttle = self.valid_range(throttle)
        self.rudder = self.valid_range(rudder+self.trim_yaw)
        self.aileron = self.valid_range(aileron+self.trim_roll)
        self.elevator = self.valid_range(elevator+self.trim_pitch)
        self.mode = mode
        #print("commands: {} {} {} {}".format(self.aileron, self.elevator, self.rudder, self.throttle))

    #sends out a message to the given IP/PORT
    def send(self):
        self.createMSG()
        try:
            self.sock.sendto(self.message, (self.IP, self.PORT))
            if self.verbose:
                print(":".join(x.encode('hex') for x in self.message))
        except:
            if self.verbose:
                print("cx10ds disconnected")

    # crc calculation
    def crc_calculate(self):
        self.crc = self.throttle ^ self.rudder ^ self.aileron ^ self.elevator ^ self.mode

    # create the message from the actual values of the virtual remote, must be called before each send
    def createMSG(self):
        self.crc_calculate()
        data = chr(self.aileron) + chr(self.elevator) + chr(self.throttle) + chr(self.rudder) + chr(self.mode) + chr(self.crc)

        self.message = self.header + data + self.footer

    # validate range with constraints
    def valid_range(self, value, MIN = 0, MAX = 255):
        return max(min(MAX, value), MIN)

    def set_trim(self):
        self.trim_roll = self.aileron - 128
        self.trim_pitch = self.elevator - 128
        self.trim_yaw = self.rudder - 128
        if self.verbose:
            print("trim: {} {} {}".format(self.trim_roll, self.trim_pitch, self.trim_yaw))


