#!/usr/bin/env python
#
# Copyright (C) 2017 Hector Garcia de Marina <hgdemarina@gmail.com>
#                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

'''
Centralized circular formations employing guidance vector fields (gvf)
'''


import sys
import numpy as np
import json
from time import sleep
from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
sys.path.append(PPRZ_SRC + "/sw/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
from settings_xml_parse import PaparazziACSettings

class Aircraft:
    def __init__(self, ac_id):
        self.initialized_gvf = False
        self.initialized_nav = False
        self.id = ac_id
        self.XY = np.zeros(2)
        self.XYc = np.zeros(2)
        self.a = 0
        self.b = 0

        self.s = 1

        self.sigma = 0

        self.a_index = 0
        self.b_index = 0

class FormationControl:
    def __init__(self, config, freq=10., verbose=False):
        self.config = config
        self.step = 1. / freq
        self.verbose = verbose
        self.ids = self.config['ids']
        self.B = np.array(self.config['topology'])
        self.Zdesired = np.array(self.config['desired_intervehicle_angles_degrees'])*np.pi/180
        self.k = np.array(self.config['gain'])
        self.radius = np.array(self.config['desired_stationary_radius_meters'])
        self.aircraft = [Aircraft(i) for i in self.ids]
        self.sigmas = np.zeros(len(self.aircraft))

        for ac in self.aircraft:
            settings = PaparazziACSettings(ac.id)
            list_of_indexes = ['ell_a', 'ell_b']

            for setting_ in list_of_indexes:
                try:
                    index = settings.name_lookup[setting_].index
                    if setting_ == 'ell_a':
                        ac.a_index = index
                    if setting_ == 'ell_b':
                        ac.b_index = index
                except Exception as e:
                    print(e)
                    print(setting_ + " setting not found, \
                            have you forgotten to check gvf.xml for your settings?")


        # Start IVY interface
        self._interface = IvyMessagesInterface("Circular Formation")

        # bind to NAVIGATION message
        def nav_cb(ac_id, msg):
            if ac_id in self.ids and msg.name == "NAVIGATION":
                ac = self.aircraft[self.ids.index(ac_id)]
                ac.XY[0] = float(msg.get_field(2))
                ac.XY[1] = float(msg.get_field(3))
                ac.initialized_nav = True
        self._interface.subscribe(nav_cb, PprzMessage("telemetry", "NAVIGATION"))

        def gvf_cb(ac_id, msg):
            if ac_id in self.ids and msg.name == "GVF":
                if int(msg.get_field(1)) == 1:
                    ac = self.aircraft[self.ids.index(ac_id)]
                    param = msg.get_field(4)
                    ac.XYc[0] = float(param[0])
                    ac.XYc[1] = float(param[1])
                    ac.a = float(param[2])
                    ac.b = float(param[3])
                    ac.s = float(msg.get_field(2))
                    ac.initialized_gvf = True
        self._interface.subscribe(gvf_cb, PprzMessage("telemetry", "GVF"))


    def __del__(self):
        self.stop()

    def stop(self):
        # Stop IVY interface
        if self._interface is not None:
            self._interface.shutdown()

    def circular_formation(self):
        '''
        circular formation control algorithm
        '''

        ready = True
        for ac in self.aircraft:
            if (not ac.initialized_nav) or (not ac.initialized_gvf):
                if self.verbose:
                    print("Waiting for state of aircraft ", ac.id)
                ready = False

        if not ready:
            return

        i = 0
        for ac in self.aircraft:
            ac.sigma = np.arctan2(ac.XY[1]-ac.XYc[1], ac.XY[0]-ac.XYc[0])
            self.sigmas[i] = ac.sigma
            i = i + 1

        inter_sigma = self.B.transpose().dot(self.sigmas)
        error_sigma = inter_sigma - self.Zdesired

        if np.size(error_sigma) > 1:
            for i in range(0, np.size(error_sigma)):
                if error_sigma[i] > np.pi:
                    error_sigma[i] = error_sigma[i] - 2*np.pi
                elif error_sigma[i] <= -np.pi:
                    error_sigma[i] = error_sigma[i] + 2*np.pi
        else:
            if error_sigma > np.pi:
                error_sigma = error_sigma - 2*np.pi
            elif error_sigma <= -np.pi:
                error_sigma = error_sigma + 2*np.pi


        u = -self.aircraft[0].s*self.k*self.B.dot(error_sigma)

        if self.verbose:
            print("Inter-vehicle errors: ", str(error_sigma*180.0/np.pi).replace('[','').replace(']',''))

        i = 0
        for ac in self.aircraft:
            msga = PprzMessage("ground", "DL_SETTING")
            msga['ac_id'] = ac.id
            msga['index'] = ac.a_index
            msga['value'] = self.radius + u[i]
            msgb = PprzMessage("ground", "DL_SETTING")
            msgb['ac_id'] = ac.id
            msgb['index'] = ac.b_index
            msgb['value'] = self.radius + u[i]

            self._interface.send(msga)
            self._interface.send(msgb)

            i = i + 1

    def run(self):
        try:
            # The main loop
            while True:
                # TODO: make better frequency managing
                sleep(self.step)

                # Run the formation algorithm
                self.circular_formation()

        except KeyboardInterrupt:
            self.stop()

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Circular formation")
    parser.add_argument('config_file', help="JSON configuration file")
    parser.add_argument('-f', '--freq', dest='freq', default=5, type=int, help="control frequency")
    parser.add_argument('-v', '--verbose', dest='verbose', default=False, action='store_true', help="display debug messages")
    args = parser.parse_args()

    with open(args.config_file, 'r') as f:
        conf = json.load(f)
        if args.verbose:
            print(json.dumps(conf))

        fc = FormationControl(conf, freq=args.freq, verbose=args.verbose)
        fc.run()
