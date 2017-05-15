#!/usr/bin/env python
from __future__ import print_function

import time
import sys
import wx
import numpy as np
import sys
from os import path, getenv
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage 
from settings_xml_parse import PaparazziACSettings

class aircraft:
    def __init__(self, ac_id):
        self.id = ac_id
        self.XY = np.array([-999, -999])
        self.XYc = np.array([-999, -999])
        self.a = -999
        self.b = -999

        self.s = -999

        self.sigma = -999

        self.a_index = -999
        self.b_index = -999

        self.time = -999

list_ids = []
list_aircraft = []
interface = IvyMessagesInterface("GVF Formation")

def message_recv(ac_id, msg):
    if ac_id in list_ids:
        ac = list_aircraft[list_ids.index(ac_id)]

        if msg.name == 'NAVIGATION':
            ac.XY[0] = float(msg.get_field(2))
            ac.XY[1] = float(msg.get_field(3))

        if msg.name == 'GVF':
            if int(msg.get_field(1)) == 1:
                param = msg.get_field(4).split(',')
                ac.XYc[0] = float(param[0])
                ac.XYc[1] = float(param[1])
                ac.a = float(param[2])
                ac.b = float(param[3])
                ac.s = float(msg.get_field(2))

        if msg.name == 'BAT':
            ac.time = float(msg.get_field(3))
    return

def formation(B, ds, radius, k):

    waiting_for_msgs = 0
    for ac in list_aircraft:
        if ac.a == -999:
            print("Waiting for GVF msg of aircraft ", ac.id)
            waiting_for_msgs = 1
        if ac.XY[0] == -999:
            print("Waiting for NAV msg of aircraft ", ac.id)
            waiting_for_msgs = 1
    
    if waiting_for_msgs == 1:
        return

    sigma = np.zeros(len(list_aircraft))
    i = 0
    for ac in list_aircraft:
        ac.sigma = np.arctan2(ac.XY[1]-ac.XYc[1], ac.XY[0]-ac.XYc[0])
        sigma[i] = ac.sigma
        i = i + 1

    inter_sigma = B.transpose().dot(sigma)
    error_sigma = inter_sigma - ds
    
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


    u = -list_aircraft[0].s*k*B.dot(error_sigma)

    print(list_aircraft[0].time, " ", str(error_sigma*180.0/np.pi).replace('[','').replace(']',''))

    i = 0
    for ac in list_aircraft:
        msga = PprzMessage("ground", "DL_SETTING")
        msga['ac_id'] = ac.id
        msga['index'] = ac.a_index
        msga['value'] = radius + u[i]
        msgb = PprzMessage("ground", "DL_SETTING")
        msgb['ac_id'] = ac.id
        msgb['index'] = ac.b_index
        msgb['value'] = radius + u[i]

        interface.send(msga)
        interface.send(msgb)

        i = i + 1
    return

def main():
    if len(sys.argv) != 6:
        print("Usage: gvfFormationApp topology.txt desired_sigma.txt ids.txt radius k")
        interface.shutdown()
        return

    B = np.loadtxt(sys.argv[1])
    desired_sigmas = np.loadtxt(sys.argv[2])*np.pi/180.0
    ids = np.loadtxt(sys.argv[3])
    radius = float(sys.argv[4])
    k = float(sys.argv[5])

    global list_ids
    list_ids = np.ndarray.tolist(ids)
    map(int, list_ids)

    if np.size(ids) != np.size(B,0):
        print("The ammount of aircrafts in the topology and ids does not match")
        return

    for i in range(0, len(ids)):
        list_aircraft.append(aircraft(int(ids[i])))


    # Ivy
    interface.subscribe(message_recv)

    for ac in list_aircraft:
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
                        have you forgotten gvf.xml in your settings?")

    try:
        while True:
            time.sleep(0.5)
            formation(B, desired_sigmas, radius, k)

    except KeyboardInterrupt:
        interface.shutdown()


if __name__ == '__main__':
    main()
