#!/usr/bin/env python
#
# Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

'''
Get air traffic information from the opensky-network system (https://opensky-network.org/)

The python API is required and should be installed using the instruction
from https://github.com/openskynetwork/opensky-api

Positions of the UAVs are used to request for the surrounding aircraft
and INTRUDER messages are sent to the IVY bus for display in the GCS
'''


from __future__ import print_function

import sys
from os import path, getenv
from time import sleep, time

from opensky_api import OpenSkyApi

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../..')))
sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage


class OpenSkyTraffic(object):
    def __init__(self, period=10., margin=1., timeout=60., verbose=False):
        self.period = period
        self.margin = margin
        self.timeout = timeout
        self.last_receive = time()
        self.verbose = verbose
        self._interface = IvyMessagesInterface("OpenSkyTraffic")
        self._opensky = OpenSkyApi()
        self.ac_list = {}
        self.intruder_list = {}
        self.running = False
        if self.verbose:
            print("Starting opensky interface...")
        
        # subscribe to FLIGHT_PARAM ground message
        self._interface.subscribe(self.update_ac, PprzMessage("ground", "FLIGHT_PARAM"))

    def stop(self):
        if self.verbose:
            print("Shutting down opensky interface...")
        self.running = False
        if self._interface is not None:
            self._interface.shutdown()

    def __del__(self):
        self.stop()

    def update_ac(self, ac_id, msg):
        # update A/C info
        self.last_receive = time()
        self.ac_list[ac_id] = (self.last_receive, float(msg['lat']), float(msg['long']))

    def filter_ac(self):
        # purge timeout A/C
        timeout = time() - self.timeout
        for ac, (t, lat, lon) in self.ac_list.items():
            if t < timeout:
                del self.ac_list[ac]

    def compute_bbox_list(self):
        bbox = []
        # for now assume that all UAVs are close enough, so merge all boxes into one
        # future improvement could handle groups of UAVs far appart
        lat_min, lat_max, lon_min, lon_max = None, None, None, None
        for ac, (t, lat, lon) in self.ac_list.items():
            if lat_min is None:
                lat_min = lat
                lat_max = lat
                lon_min = lon
                lon_max = lon
            else:
                lat_min = min(lat, lat_min)
                lat_max = max(lat, lat_max)
                lon_min = min(lon, lon_min)
                lon_max = max(lon, lon_max)

        if lat_min is not None:
            bbox.append((lat_min-self.margin, lat_max+self.margin, lon_min-self.margin, lon_max+self.margin))
        return bbox

    def get_intruders(self):
        self.filter_ac()
        bbox = self.compute_bbox_list()
        # request surounding aircraft to opensky-network
        self.intruder_list = {}
        for bb in bbox:
            states = self._opensky.get_states(bbox=bb)
            if states is not None:
                for s in states.states:
                    self.intruder_list[s.callsign] = s

    def send_intruder_msgs(self):
        self.get_intruders()
        def val_or_default(val, default):
            if val is None:
                return default
            else:
                return val
        for i in self.intruder_list:
            intruder = self.intruder_list[i]
            if intruder.callsign is not None and len(intruder.callsign) > 0:
                msg = PprzMessage("ground", "INTRUDER")
                msg['id'] = intruder.icao24
                msg['name'] = intruder.callsign.replace(" ", "")
                msg['lat'] = int(intruder.latitude * 1e7)
                msg['lon'] = int(intruder.longitude * 1e7)
                msg['alt'] = int(val_or_default(intruder.geo_altitude, 0.) * 1000.)
                msg['course'] = val_or_default(intruder.heading, 0.)
                msg['speed'] = val_or_default(intruder.velocity, 0.)
                msg['climb'] = val_or_default(intruder.vertical_rate, 0.)
                msg['itow'] = intruder.time_position
                if self.verbose:
                    print(msg)
                self._interface.send(msg)

    def run(self):
        try:
            self.running = True
            t = 0.
            while self.running:
                t = t + 0.1 # increment timer until next update time is reach
                if t > self.period:
                    self.send_intruder_msgs()
                    t = 0.
                sleep(0.1) # wait a short time
        except KeyboardInterrupt:
            self.stop()
        except Exception as e:
            print("Failing with: "+str(e))
            self.stop()



if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="OpenSky-Network traffic information")
    parser.add_argument('-p', '--period', dest='period', default=10., type=float, help="update period")
    parser.add_argument('-m', '--margin', dest='margin', default=1., type=float, help="margin in degree to define the bounding box")
    parser.add_argument('-t', '--timeout', dest='timeout', default=60., type=float, help="timeout to remove A/C")
    parser.add_argument('-v', '--verbose', dest='verbose', default=False, action='store_true', help="display debug messages")
    args = parser.parse_args()

    osn = OpenSkyTraffic(args.period, args.margin, args.timeout, args.verbose)
    osn.run()

