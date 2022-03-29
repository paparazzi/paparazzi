#!/usr/bin/python3
#
# This file is part of PPRZLINK.
# 
# PPRZLINK is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# PPRZLINK is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with PPRZLINK.  If not, see <https://www.gnu.org/licenses/>.
#

#
# capture data from hot-wire anemometer PCE-423 (through USB cable)
# 

import serial
import struct
import logging
import time

logger = logging.getLogger("anemometer_PCE423")

STX_A = 0xAA
STX_B = 0xBB

STATE_SYNC = 0
STATE_GOT_A = 1
STATE_GOT_B = 2

FRAME_LENGTH = 80
DATA_LENGTH = FRAME_LENGTH - 2 # frame - STX

PCE423_BAUDRATE = 9600

class AnemometerPCE423():
    def __init__(self, device='/dev/ttyUSB0', callback=None):
        self.state = STATE_SYNC
        self.callback = callback
        self.running = True
        self.speed = 0. # airspeed in m/s
        self.temperature = 0. # temperature in °C
        self.line_1 = 0. # data displayed on line 1
        self.line_2 = 0. # data displayed on line 2
        self.time = 0. # timestamp
        try:
            self.ser = serial.Serial(device, PCE423_BAUDRATE)
        except:
            logging.error("unable to open serial port '{}'".format(device))
            exit(0)

    def stop(self):
        logger.info("Stop reading PCE-423")
        self.running = False
        self.ser.close()

    def __del__(self):
        try:
            self.ser.close()
        except:
            pass

    def run(self):
        """Star reading frames from PCE-423"""
        while self.running:
            c = self.ser.read(1) # read data to find sync bytes
            b = struct.unpack("B", c)[0]
            if self.state == STATE_SYNC and b == STX_A:
                self.state = STATE_GOT_A
            elif self.state == STATE_GOT_A:
                if b == STX_B:
                    self.state = STATE_GOT_B

                    # sync OK
                    self.time = time.time()
                    data = self.ser.read(DATA_LENGTH)
                    if len(data) >= DATA_LENGTH:
                        sub = data[0:4*4]
                        self.line_1, self.line_2, self.speed, self.temperature = struct.unpack('ffff', sub)
                        if self.callback is not None:
                            self.callback(self.time, self.speed, self.temperature)
                        logging.debug('{:f} {:f} {:f} {:f}'.format(self.line_1, self.line_2, self.speed, self.temperature))
                    else:
                        # error, not enough data
                        logging.error("frame error")
                    # goto sync
                    self.state = STATE_SYNC
                else:
                    # sync error, b != STB_B
                    self.state = STATE_SYNC
            else:
                logging.info("wait sync")

    def get_speed(self):
        return self.speed

    def get_temperature(self):
        return self.temperature


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--device", help="device name", dest='dev', default='/dev/ttyUSB0')
    args = parser.parse_args()

    def cb(time, speed, temp):
        print("time: {:.2f}, speed: {:.4f}, temperature: {:.2f}".format(time, speed, temp))

    print("Starting interface for PCE-423 anemometer")
    pce423 = AnemometerPCE423(args.dev, cb)
    try:
        pce423.run()
    except (KeyboardInterrupt, SystemExit):
        print('Shutting down...')
        pce423.stop()

