#!/usr/bin/env python

from __future__ import print_function

import sys
from os import path, getenv
from time import sleep

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage


class WaypointMover(object):
    def __init__(self, verbose=False):
        self.verbose = verbose
        self._interface = IvyMessagesInterface("WaypointMover")

    def shutdown(self):
        print("Shutting down ivy interface...")
        self._interface.shutdown()

    def __del__(self):
        self.shutdown()

    def move_waypoint(self, ac_id, wp_id, lat, lon, alt):
        msg = PprzMessage("ground", "MOVE_WAYPOINT")
        msg['ac_id'] = ac_id
        msg['wp_id'] = wp_id
        msg['lat'] = lat
        msg['long'] = lon
        msg['alt'] = alt
        print("Sending message: %s" % msg)
        self._interface.send(msg)


if __name__ == '__main__':
    try:
        wm = WaypointMover()
        # sleep shortly in order to make sure Ivy is up, then message sent before shutting down again
        sleep(0.1)
        wm.move_waypoint(ac_id=202, wp_id=3, lat=43.563, lon=1.481, alt=172.0)
        sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping on request")
    wm.shutdown()
