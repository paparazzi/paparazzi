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
#

'''
Formation Control for Rotorcraft, remotly piloted from ground with a Joystick
'''

import sys
import numpy as np
import json
from time import sleep
from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage 

import lib_rigid_formations as rf
from scipy import linalg as la

class FC_Rotorcraft:
    def __init__(self, ac_id):
        self.initialized = False
        self.id = ac_id
        self.X = np.zeros(3)
        self.V = np.zeros(3)
        self.timeout = 0

class FC_Joystick:
    def __init__(self):
        self.trans = 0.
        self.trans2 = 0.
        self.rot = 0.
        self.rot2 = 0.
        self.button1 = False
        self.button2 = False
        self.button3 = False
        self.button4 = False

class FormationControl:
    def __init__(self, config, freq=10., use_ground_ref=False, ignore_geo_fence=False, verbose=False):
        self.config = config
        self.step = 1. / freq
        self.sens = self.config['sensitivity']
        self.use_ground_ref = use_ground_ref
        self.enabled = True # run control by default
        self.ignore_geo_fence = ignore_geo_fence
        self.verbose = verbose
        self.ids = self.config['ids']
        self.rotorcrafts = [FC_Rotorcraft(i) for i in self.ids]
        self.joystick = FC_Joystick()
        self.altitude = 2.0 # starts from 1 m high
        self.scale = 1.0
        self.B = np.array(self.config['topology'])
        self.d = np.array(self.config['desired_distances'])
        self.t1 = np.array(self.config['motion']['t1'])
        self.t2 = np.array(self.config['motion']['t2'])
        self.r1 = np.array(self.config['motion']['r1'])
        self.r2 = np.array(self.config['motion']['r2'])
        self.k = np.array(self.config['gains'])

        if self.B.size == 2:
            self.B.shape = (2,1)

        # Check formation settings
        if len(self.ids) != np.size(self.B, 0):
            print("The number of rotorcrafts in the topology and ids do not match")
            return

        if np.size(self.d) != np.size(self.B, 1):
            print("The number of links in the topology and desired distances do not match")
            return

        #if np.size(self.d) != np.size(self.m,1):
        #    print("The number of (columns) motion parameters and relative vectors do not match")
        #    return

        #if np.size(self.m, 0) != 8:
        #    print("The number of (rows) motion parameters must be eight")
        #    return

        if self.config['3D'] == True:
            print("3D formation is not supported yet")
            return

        # Start IVY interface
        self._interface = IvyMessagesInterface("Formation Control Rotorcrafts")

        # bind to INS message
        def ins_cb(ac_id, msg):
            if ac_id in self.ids and msg.name == "INS":
                rc = self.rotorcrafts[self.ids.index(ac_id)]
                i2p = 1. / 2**8     # integer to position
                i2v = 1. / 2**19    # integer to velocity
                rc.X[0] = float(msg['ins_x']) * i2p
                rc.X[1] = float(msg['ins_y']) * i2p
                rc.X[2] = float(msg['ins_z']) * i2p
                rc.V[0] = float(msg['ins_xd']) * i2v
                rc.V[1] = float(msg['ins_yd']) * i2v
                rc.V[2] = float(msg['ins_zd']) * i2v
                rc.timeout = 0
                rc.initialized = True
        if not self.use_ground_ref:
            self._interface.subscribe(ins_cb, PprzMessage("telemetry", "INS"))

        # bind to GROUND_REF message
        def ground_ref_cb(ground_id, msg):
            ac_id = int(msg['ac_id'])
            if ac_id in self.ids:
                rc = self.rotorcrafts[self.ids.index(ac_id)]
                # X and V in NED
                rc.X[0] = float(msg['pos'][1])
                rc.X[1] = float(msg['pos'][0])
                rc.X[2] = -float(msg['pos'][2])
                rc.V[0] = float(msg['speed'][1])
                rc.V[1] = float(msg['speed'][0])
                rc.V[2] = -float(msg['speed'][2])
                rc.timeout = 0
                rc.initialized = True
        if self.use_ground_ref:
            self._interface.subscribe(ground_ref_cb, PprzMessage("ground", "GROUND_REF"))

        # bind to JOYSTICK message
        def joystick_cb(ac_id, msg):
            self.joystick.trans = float(msg['axis1']) * self.sens['t1'] / 127.
            self.joystick.trans2 = float(msg['axis2']) * self.sens['t2'] / 127.
            self.joystick.rot = float(msg['axis3']) * self.sens['r1'] / 127.
            self.altitude = self.sens['alt_min'] + float(msg['axis4']) * (self.sens['alt_max'] - self.sens['alt_min']) / 127.
            if msg['button1'] == '1' and not self.joystick.button1:
                self.scale = min(self.scale + self.sens['scale'], self.sens['scale_max'])
            if msg['button2'] == '1' and not self.joystick.button2:
                self.scale = max(self.scale - self.sens['scale'], self.sens['scale_min'])
            if msg['button4'] == '1' and not self.joystick.button4:
                self.enabled = False
            if msg['button3'] == '1' and not self.joystick.button3:
                self.enabled = True
            self.joystick.button1 = (msg['button1'] == '1')
            self.joystick.button2 = (msg['button2'] == '1')
            self.joystick.button3 = (msg['button3'] == '1')
            self.joystick.button4 = (msg['button4'] == '1')
        self._interface.subscribe(joystick_cb, PprzMessage("ground", "JOYSTICK"))

    def __del__(self):
        self.stop()

    def stop(self):
        # Stop IVY interface
        if self._interface is not None:
            self._interface.shutdown()

    def formation(self):
        '''
        formation control algorithm
        '''
        ready = True
        for rc in self.rotorcrafts:
            if not rc.initialized:
                if self.verbose:
                    print("Waiting for state of rotorcraft ", rc.id)
                    sys.stdout.flush()
                ready = False
            if rc.timeout > 0.5:
                if self.verbose:
                    print("The state msg of rotorcraft ", rc.id, " stopped")
                    sys.stdout.flush()
                ready = False
            if rc.initialized and 'geo_fence' in self.config.keys():
                geo_fence = self.config['geo_fence']
                if not self.ignore_geo_fence:
                    if (rc.X[0] < geo_fence['x_min'] or rc.X[0] > geo_fence['x_max']
                            or rc.X[1] < geo_fence['y_min'] or rc.X[1] > geo_fence['y_max']
                            or rc.X[2] < geo_fence['z_min'] or rc.X[2] > geo_fence['z_max']):
                        if self.verbose:
                            print("The rotorcraft", rc.id, " is out of the fence (", rc.X, ", ", geo_fence, ")")
                            sys.stdout.flush()
                        ready = False

        if not ready:
            return

        dim = 2 * len(self.rotorcrafts)
        X = np.zeros(dim)
        V = np.zeros(dim)
        U = np.zeros(dim)

        i = 0
        for rc in self.rotorcrafts:
            X[i] = rc.X[0]
            X[i+1] = rc.X[1]
            V[i] = rc.V[0]
            V[i+1] = rc.V[1]
            i = i + 2


        Bb = la.kron(self.B, np.eye(2))
        Z = Bb.T.dot(X)
        Dz = rf.make_Dz(Z, 2)
        Dzt = rf.make_Dzt(Z, 2, 1)
        Dztstar = rf.make_Dztstar(self.d * self.scale, 2, 1)
        Zh = rf.make_Zh(Z, 2)
        E = rf.make_E(Z, self.d * self.scale, 2, 1)

        # Shape and motion control
        jmu_t1 = self.joystick.trans * self.t1[0,:]
        jtilde_mu_t1 = self.joystick.trans * self.t1[1,:]
        jmu_r1 = self.joystick.rot * self.r1[0,:]
        jtilde_mu_r1 = self.joystick.rot * self.r1[1,:]
        jmu_t2 = self.joystick.trans2 * self.t2[0,:]
        jtilde_mu_t2 = self.joystick.trans2 * self.t2[1,:]
        jmu_r2 = self.joystick.rot2 * self.r2[0,:]
        jtilde_mu_r2 = self.joystick.rot2 * self.r2[1,:]

        Avt1 = rf.make_Av(self.B, jmu_t1, jtilde_mu_t1)
        Avt1b = la.kron(Avt1, np.eye(2))
        Avr1 = rf.make_Av(self.B, jmu_r1, jtilde_mu_r1)
        Avr1b = la.kron(Avr1, np.eye(2))
        Avt2 = rf.make_Av(self.B, jmu_t2, jtilde_mu_t2)
        Avt2b = la.kron(Avt2, np.eye(2))
        Avr2 = rf.make_Av(self.B, jmu_r2, jtilde_mu_r2)
        Avr2b = la.kron(Avr2, np.eye(2))

        Avb = Avt1b + Avr1b + Avt2b + Avr2b
        Avr = Avr1 + Avr2

        if self.B.size == 2:
            Zhr = np.array([-Zh[1],Zh[0]])
            U = -self.k[1]*V - self.k[0]*Bb.dot(Dz).dot(Dzt).dot(E) + self.k[1]*(Avt1b.dot(Zh) + Avt2b.dot(Zhr)) + np.sign(jmu_r1[0])*la.kron(Avr1.dot(Dztstar).dot(self.B.T).dot(Avr1), np.eye(2)).dot(Zhr)
        else:
            U = -self.k[1]*V - self.k[0]*Bb.dot(Dz).dot(Dzt).dot(E) + self.k[1]*Avb.dot(Zh) + la.kron(Avr.dot(Dztstar).dot(self.B.T).dot(Avr), np.eye(2)).dot(Zh)

        if self.verbose:
            #print "Positions: " + str(X).replace('[','').replace(']','')
            #print "Velocities: " + str(V).replace('[','').replace(']','')
            #print "Acceleration command: " + str(U).replace('[','').replace(']','')
            print "Error distances: " + str(E).replace('[','').replace(']','')
            sys.stdout.flush()

        i = 0
        for ac in self.rotorcrafts:
            msg = PprzMessage("datalink", "DESIRED_SETPOINT")
            msg['ac_id'] = ac.id
            msg['flag'] = 0 # full 3D not supported yet
            msg['ux'] = U[i]
            msg['uy'] = U[i+1]
            msg['uz'] = self.altitude
            self._interface.send(msg)
            i = i+2


    def run(self):
        try:
            # The main loop
            while True:
                # TODO: make better frequency managing
                sleep(self.step)

                for rc in self.rotorcrafts:
                    rc.timeout = rc.timeout + self.step

                # Run the formation algorithm
                if self.enabled:
                    self.formation()

        except KeyboardInterrupt:
            self.stop()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Formation Control for Rotorcrafts")
    parser.add_argument('config_file', help="JSON configuration file")
    parser.add_argument('-f', '--freq', dest='freq', default=10, type=int, help="control frequency")
    parser.add_argument('-gr', '--use_ground_ref', dest='use_ground_ref', default=False, action='store_true', help="use GROUND_REF messages instead of INS messages")
    parser.add_argument('-igf', '--ignore_geo_fence', dest='ignore_geo_fence', default=False, action='store_true', help="ignore the geo fence limits")
    parser.add_argument('-v', '--verbose', dest='verbose', default=False, action='store_true', help="display debug messages")
    args = parser.parse_args()

    # if config file is not a full path, look for files in conf/swarm folder of PAPARAZI_HOME
    if not path.isabs(args.config_file):
        args.config_file = path.join(PPRZ_HOME, path.normpath("conf/swarm"), args.config_file)

    with open(args.config_file, 'r') as f:
        conf = json.load(f)
        if args.verbose:
            print(json.dumps(conf))

        fc = FormationControl(conf, freq=args.freq, use_ground_ref=args.use_ground_ref, ignore_geo_fence=args.ignore_geo_fence, verbose=args.verbose)
        fc.run()

