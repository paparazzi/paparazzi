#!/usr/bin/env python

from __future__ import print_function

import sys
from os import path, getenv

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprz_connect import PprzConnect, PprzConfig
from guided_mode import GuidedMode
from settings import PprzSettingsManager

def main():
    from time import sleep
    from math import radians
    import argparse
    parser = argparse.ArgumentParser(description="Guided mode example")
    parser.add_argument("-i", "--ac_id", dest='ac_id', default=1, type=int, help="aircraft ID")
    args = parser.parse_args()

    try:
        connect = PprzConnect()
        sleep(2)
        print(connect._conf_list_by_id)
        conf = connect.conf_by_id(str(args.ac_id))
        settings = PprzSettingsManager(conf.settings, conf.id, connect.ivy)
        guided = GuidedMode(connect.ivy)
        
        settings['auto2'] = 'Guided'

        sleep(0.1)
        print("Goto NED 2, 2, -3, 90°")
        guided.goto_ned(conf.id, north=2.0, east=2.0, down=-3.0, heading=radians(90))
        sleep(5)
        print("Goto NED relative  -2, -2, 1, -45°")
        guided.goto_ned_relative(conf.id, north=-2.0, east=-2.0, down=1.0, yaw=-radians(45))
        sleep(5)
        print("Goto body 0, 1, 0")
        guided.goto_body_relative(conf.id, forward=0.0, right=1.0, down=0.0)
        sleep(5)
        print("Move NED 0.5m/s north")
        guided.move_at_ned_vel(conf.id, north=0.5)
        sleep(5)
        print("Move body -0.5m/s forward")
        guided.move_at_body_vel(conf.id, forward=-0.5)
        sleep(2)

        settings['auto2'] = 'Nav'

    except KeyboardInterrupt:
        print("Stopping on request")
    finally:
        connect.shutdown()


if __name__ == '__main__':
    main()
