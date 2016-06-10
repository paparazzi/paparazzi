#!/usr/bin/env python

from __future__ import print_function

import sys
from os import path, getenv

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
from settings_xml_parse import PaparazziACSettings

from math import radians
from time import sleep


class Guidance(object):
    def __init__(self, ac_id, verbose=False):
        self.ac_id = ac_id
        self.verbose = verbose
        self._interface = None
        self.auto2_index = None
        try:
            settings = PaparazziACSettings(self.ac_id)
        except Exception as e:
            print(e)
            return
        try:
            self.auto2_index = settings.name_lookup['auto2'].index
        except Exception as e:
            print(e)
            print("auto2 setting not found, mode change not possible.")
        self._interface = IvyMessagesInterface("guided mode example")

    def shutdown(self):
        if self._interface is not None:
            print("Shutting down ivy interface...")
            self._interface.shutdown()
            self._interface = None

    def __del__(self):
        self.shutdown()

    def set_guided_mode(self):
        """
        change auto2 mode to GUIDED.
        """
        if self.auto2_index is not None:
            msg = PprzMessage("ground", "DL_SETTING")
            msg['ac_id'] = self.ac_id
            msg['index'] = self.auto2_index
            msg['value'] = 19  # AP_MODE_GUIDED
            print("Setting mode to GUIDED: %s" % msg)
            self._interface.send(msg)

    def set_nav_mode(self):
        """
        change auto2 mode to NAV.
        """
        if self.auto2_index is not None:
            msg = PprzMessage("ground", "DL_SETTING")
            msg['ac_id'] = self.ac_id
            msg['index'] = self.auto2_index
            msg['value'] = 13  # AP_MODE_NAV
            print("Setting mode to NAV: %s" % msg)
            self._interface.send(msg)

    def goto_ned(self, north, east, down, heading=0.0):
        """
        goto a local NorthEastDown position in meters (if already in GUIDED mode)
        """
        msg = PprzMessage("datalink", "GUIDED_SETPOINT_NED")
        msg['ac_id'] = self.ac_id
        msg['flags'] = 0x00
        msg['x'] = north
        msg['y'] = east
        msg['z'] = down
        msg['yaw'] = heading
        print("goto NED: %s" % msg)
        # embed the message in RAW_DATALINK so that the server can log it
        self._interface.send_raw_datalink(msg)

    def goto_ned_relative(self, north, east, down, yaw=0.0):
        """
        goto a local NorthEastDown position relative to current position in meters (if already in GUIDED mode)
        """
        msg = PprzMessage("datalink", "GUIDED_SETPOINT_NED")
        msg['ac_id'] = self.ac_id
        msg['flags'] = 0x0D
        msg['x'] = north
        msg['y'] = east
        msg['z'] = down
        msg['yaw'] = yaw
        print("goto NED relative: %s" % msg)
        self._interface.send_raw_datalink(msg)

    def goto_body_relative(self, forward, right, down, yaw=0.0):
        """
        goto to a position relative to current position and heading in meters (if already in GUIDED mode)
        """
        msg = PprzMessage("datalink", "GUIDED_SETPOINT_NED")
        msg['ac_id'] = self.ac_id
        msg['flags'] = 0x0E
        msg['x'] = forward
        msg['y'] = right
        msg['z'] = down
        msg['yaw'] = yaw
        print("goto body relative: %s" % msg)
        self._interface.send_raw_datalink(msg)

    def move_at_ned_vel(self, north=0.0, east=0.0, down=0.0, yaw=0.0):
        """
        move at specified velocity in meters/sec with absolute heading (if already in GUIDED mode)
        """
        msg = PprzMessage("datalink", "GUIDED_SETPOINT_NED")
        msg['ac_id'] = self.ac_id
        msg['flags'] = 0x60
        msg['x'] = north
        msg['y'] = east
        msg['z'] = down
        msg['yaw'] = yaw
        print("move at vel NED: %s" % msg)
        self._interface.send_raw_datalink(msg)
        
    def move_at_body_vel(self, forward=0.0, right=0.0, down=0.0, yaw=0.0):
        """
        move at specified velocity in meters/sec with absolute heading (if already in GUIDED mode)
        """
        msg = PprzMessage("datalink", "GUIDED_SETPOINT_NED")
        msg['ac_id'] = self.ac_id
        msg['flags'] = 0x62
        msg['x'] = forward
        msg['y'] = right
        msg['z'] = down
        msg['yaw'] = yaw
        print("move at vel body: %s" % msg)
        self._interface.send_raw_datalink(msg)


if __name__ == '__main__':
    ac_id = 11
    try:
        g = Guidance(ac_id)
        sleep(0.1)
        g.set_guided_mode()
        sleep(0.2)
        g.goto_ned(north=2.0, east=2.0, down=-3.0, heading=radians(90))
        sleep(10)
        g.goto_ned_relative(north=-2.0, east=-2.0, down=1.0, yaw=-radians(45))
        sleep(10)
        g.goto_body_relative(forward=0.0, right=1.0, down=0.0)
        sleep(10)
        g.move_at_ned_vel(north=0.5)
        sleep(3)
        g.move_at_body_vel(forward=-0.5)
        sleep(3)
        g.set_nav_mode()
        sleep(0.2)
    except KeyboardInterrupt:
        print("Stopping on request")
    g.shutdown()
