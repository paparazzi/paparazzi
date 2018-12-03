#!/usr/bin/env python
#
# MIT License
#
# Copyright (c) 2018 Gautier Hattenberger
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

from cx10ds import CX10DS

import sys
from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage 


class Cx10ds2ivy:
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.step = 0.1 # period in seconds
        self.button_trim = False

        self._cx10 = CX10DS(verbose)

        # Start IVY interface
        self._interface = IvyMessagesInterface("Cx10ds2ivy")

        # bind to JOYSTICK message
        def joystick_cb(ac_id, msg):
            aileron = int(msg['axis1'])
            elevator = int(msg['axis2'])
            rudder = int(msg['axis3'])
            if int(msg['button1']) == 255:
                throttle = int(msg['axis4']) # direct throttle reading
            else:
                throttle = 128
                direction = int(msg['button1'])-1
                throttle_incr = self._cx10.valid_range(int(msg['axis4']), 0, 127)
                throttle = 128 + direction * throttle_incr # up
            mode = int(msg['button2'])
            if msg['button4'] == '1' and not self.button_trim:
                self._cx10.set_trim()
            self.button_trim = (msg['button4'] == '1')

            self._cx10.set_cmd(aileron,elevator,rudder,throttle,mode)

            if self.verbose:
                print("Throttle {0}".format(throttle))
                print("Rudder {0}".format(rudder))
                print("elevator {0}".format(elevator))
                print("aileron {0}".format(aileron))
                print("Mode {0}".format(mode))

        self._interface.subscribe(joystick_cb, PprzMessage("ground", "JOYSTICK"))

    def __del__(self):
        self.stop()

    def stop(self):
        # Stop IVY interface
        if self._interface is not None:
            self._interface.shutdown()

    # main loop
    def run(self):
        try:
            while True:
                # TODO: make better frequency managing
                self._cx10.send()
                sleep(self.step)

        except KeyboardInterrupt:
            if self.verbose:
                print("Exiting..")
            self.stop()


# example for using the class
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="CX10DS Remote Control")
    parser.add_argument('-v', '--verbose', dest='verbose', default=False, action='store_true', help="display debug messages")
    args = parser.parse_args()

    rmt = Cx10ds2ivy(verbose=args.verbose)
    rmt.run()

