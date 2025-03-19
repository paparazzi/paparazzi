#!/usr/bin/env python3
#
# Copyright (C) 2025 Gautier Hattenberger <gautier.hattenberger@enac.fr>
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
#

import sys
from os import path, getenv

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage


class GuidedMode(object):
    '''
    a collection of commonly used functions in guided mode
    '''
    FLAG_ABS_POS    = 0x00
    FLAG_XY_OFFSET  = 0x01
    FLAG_XY_BODY    = 0x02
    FLAG_Z_OFFSET   = 0x04
    FLAG_YAW_OFFSET = 0x08
    FLAG_XY_VEL     = 0x20
    FLAG_Z_VEL      = 0x40
    FLAG_YAW_VEL    = 0x80

    def __init__(self, interface=None):
        self._interface = interface
        if self._interface is None:
            self._interface = IvyMessagesInterface("guided mode")

    def shutdown(self):
        if self._interface is not None:
            self._interface.shutdown()
            self._interface = None

    def __del__(self):
        self.shutdown()

    def guided_cmd(self, ac_id: int, flag: int, x: float, y: float, z: float, yaw: float):
        '''
        generic guided mode function
        messages are sent in RAW_DATALINK so that the server can log it
        '''
        msg = PprzMessage("datalink", "GUIDED_SETPOINT_NED")
        msg['ac_id'] = ac_id
        msg['flags'] = flag
        msg['x'] = x
        msg['y'] = y
        msg['z'] = z
        msg['yaw'] = yaw
        self._interface.send_raw_datalink(msg)

    def goto_ned(self, ac_id, north, east, down, heading=0.0):
        """
        goto a local NorthEastDown position in meters (if already in GUIDED mode)
        """
        self.guided_cmd(ac_id, self.FLAG_ABS_POS, north, east, down, heading)

    def goto_enu(self, ac_id, east, north, up, heading=0.0):
        """
        goto a local EastNorthUp position in meters (if already in GUIDED mode)
        """
        self.guided_cmd(ac_id, 0x0, north, east, -up, heading)

    def goto_ned_relative(self, ac_id, north, east, down, yaw=0.0):
        """
        goto a local NorthEastDown position relative to current position in meters (if already in GUIDED mode)
        """
        self.guided_cmd(ac_id, self.FLAG_XY_OFFSET | self.FLAG_Z_OFFSET | self.FLAG_YAW_OFFSET, north, east, down, yaw)

    def goto_enu_relative(self, ac_id, east, north, up, yaw=0.0):
        """
        goto a local EastNorthUp position relative to current position in meters (if already in GUIDED mode)
        """
        self.guided_cmd(ac_id, self.FLAG_XY_OFFSET | self.FLAG_Z_OFFSET | self.FLAG_YAW_OFFSET, north, east, -up, yaw)

    def goto_body_relative(self, ac_id, forward, right, down, yaw=0.0):
        """
        goto to a position relative to current position and heading in meters (if already in GUIDED mode)
        """
        self.guided_cmd(ac_id, self.FLAG_XY_BODY | self.FLAG_Z_OFFSET | self.FLAG_YAW_OFFSET, forward, right, down, yaw)

    def move_at_ned_vel(self, ac_id, north=0.0, east=0.0, down=0.0, yaw=0.0):
        """
        move at specified NorthEastDown velocity in meters/sec with absolute heading (if already in GUIDED mode)
        """
        self.guided_cmd(ac_id, self.FLAG_XY_VEL | self.FLAG_Z_VEL, north, east, down, yaw)

    def move_at_enu_vel(self, ac_id, east=0.0, north=0.0, up=0.0, yaw=0.0):
        """
        move at specified EastNorthUp velocity in meters/sec with absolute heading (if already in GUIDED mode)
        """
        self.guided_cmd(ac_id, self.FLAG_XY_VEL | self.FLAG_Z_VEL, north, east, -up, yaw)

    def move_at_body_vel(self, ac_id, forward=0.0, right=0.0, down=0.0, yaw=0.0):
        """
        move at specified velocity in meters/sec with absolute heading (if already in GUIDED mode)
        """
        self.guided_cmd(ac_id, self.FLAG_XY_BODY | self.FLAG_XY_VEL | self.FLAG_Z_VEL, forward, right, down, yaw)

    def move_at_body_vel_yaw_rate(self, ac_id, forward=0.0, right=0.0, down=0.0, yaw_rate=0.0):
        """
        move at specified velocity in meters/sec with yaw rate (if already in GUIDED mode)
        """
        self.guided_cmd(ac_id, self.FLAG_XY_BODY | self.FLAG_XY_VEL | self.FLAG_Z_VEL | self.FLAG_YAW_VEL, forward, right, down, yaw_rate)



def main():
    from math import radians
    from time import sleep
    import argparse

    parser = argparse.ArgumentParser(description="Guided mode example")
    parser.add_argument("-i", "--ac_id", dest='ac_id', default=1, type=int, help="aircraft ID")
    args = parser.parse_args()

    print(f"Starting guided mode example for A/C id '{args.ac_id}'")
    g = None
    try:
        g = GuidedMode()
        sleep(0.1)
        print("goto ned")
        g.goto_ned(args.ac_id, north=2.0, east=2.0, down=-3.0, heading=radians(90))
        sleep(5)
        print("goto ned relative")
        g.goto_ned_relative(args.ac_id, north=-2.0, east=-2.0, down=1.0, yaw=-radians(45))
        sleep(5)
        print("goto body relative")
        g.goto_body_relative(args.ac_id, forward=0.0, right=1.0, down=0.0)
        sleep(5)
        print("move at ned velocity")
        g.move_at_ned_vel(args.ac_id, north=0.5)
        sleep(5)
        print("move at body frame velocity")
        g.move_at_body_vel(args.ac_id, forward=-0.5)
        sleep(5)
        print("stop moving")
        g.move_at_ned_vel(args.ac_id)
        sleep(1)
    except KeyboardInterrupt:
        print("Stopping on request")
    except Exception as e:
        print("Failing with error:", e)
    finally:
        if g is not None:
            print("[guided] Shutting down ivy interface...")
            g.shutdown()


if __name__ == '__main__':
    main()
