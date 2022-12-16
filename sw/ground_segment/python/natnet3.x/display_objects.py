#!/usr/bin/env python3
#
# Copyright (C) 2022 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
Receive marker set from optitrack system and display a convex hull for
each selected objects in the GCS with the SHAPE message

Exemple to display "Building_XYZ" object at Enac:
    ./display_objects.py -r 43.5640917,1.4829201 -up z_up -n 'Building_.*'

''' 

import sys
from os import path, getenv
from time import time, sleep
import numpy as np
import re
from scipy.spatial import ConvexHull
import argparse

# import NatNet client
from NatNetClient import NatNetClient

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

# parse args
parser = argparse.ArgumentParser()
parser.add_argument('-n', '--name', dest='name', required=True, help="Regular expression of the names of the object to display")
parser.add_argument('-r', '--ref', dest='ref', required=True, help="Reference position in lat,long format in degrees")
parser.add_argument('-c', '--color', dest='color', default='white', help="Display color")
parser.add_argument('-sn', '--show_name', dest='show_name', action='store_true', help="Display the name of the object")
parser.add_argument('-t', '--timeout', dest='timeout', default=2., type=float, help="Timeout for removing object from scene")
parser.add_argument('-th', '--threshold', dest='threshold', default=0.1, type=float, help="Threshold distance to consider a moving object in meter")
parser.add_argument('-b', '--ivy_bus', dest='ivy_bus', help="Ivy bus address and port")
parser.add_argument('-s', '--server', dest='server', default="127.0.0.1", help="NatNet server IP address")
parser.add_argument('-m', '--multicast_addr', dest='multicast', default="239.255.42.99", help="NatNet server multicast address")
parser.add_argument('-dp', '--data_port', dest='data_port', type=int, default=1511, help="NatNet server data socket UDP port")
parser.add_argument('-cp', '--command_port', dest='command_port', type=int, default=1510, help="NatNet server command socket UDP port")
parser.add_argument('-v', '--verbose', dest='verbose', action='store_true', help="display debug messages")
parser.add_argument('-vv', '--very_verbose', dest='very_verbose', action='store_true', help="display debug messages and optitrack debug messages")
parser.add_argument('-f', '--freq', dest='freq', default=1., type=float, help="transmit frequency")
parser.add_argument('-rp', '--refresh_period', dest='refresh_period', default=4., type=float, help="refresh period at which message is sent event if the object is not moving")
parser.add_argument('-up', '--up_axis', dest='up_axis', choices=['y_up', 'z_up'], default='y_up', help="Optitrack Up axis: y_up or z_up.")
parser.add_argument('-o', '--old_natnet', dest='old_natnet', action='store_true', help="Change the NatNet version to 2.9")

args = parser.parse_args()

# message period
period = 1./args.freq
# mean earth radius from IUGG
R_earth = 6371008.7714
# reference position lat0, long0
lat0, long0 = args.ref.split(",")
lat0 = float(lat0)
R_cos_lat0 = R_earth*np.cos(np.deg2rad(lat0))
long0 = float(long0)
# dictionary of id and received time by set name
markerset = {}
current_index = 1
# index of horizontal X and Y coordinates
X_AXIS = 0
if args.up_axis == 'y_up':
    Y_AXIS = 2
    Y_SIGN = -1
else:
    Y_AXIS = 1
    Y_SIGN = 1

# start ivy interface
if args.ivy_bus is not None:
    ivy = IvyMessagesInterface("display_objects", ivy_bus=args.ivy_bus)
else:
    ivy = IvyMessagesInterface("display_objects")

def is_moving(old_pos, new_pos):
    '''
    check if the markerset position have changed (at least one of the marker)
    '''
    try:
        for po, pn in zip(old_pos, new_pos):
            if abs(po[0]-pn[0]) > args.threshold or abs(po[1]-pn[1]) > args.threshold or abs(po[2] - pn[2]) > args.threshold:
                return True # at least one marker has moved
        return False # object has not moved
    except:
        # if failing, update position
        return True

def receiveMarkerSet(name, posList):
    '''
    callback for markerset with name and marker position as input
    '''
    global current_index

    # check if name is matching regexp
    name = name.decode('utf-8')
    if re.fullmatch(args.name, name) is not None:

        # check if message should be sent (first time or period)
        send = False
        now = time()
        if name in markerset:
            dt = now - markerset[name]['time']
            dt_refresh = now - markerset[name]['time_refresh']
            if dt >= period:
                # period elapsed, check if moved
                markerset[name]['time'] = now
                moved = is_moving(markerset[name]['pos'], posList)
                if moved:
                    send = True
                    markerset[name]['pos'] = posList
            if dt_refresh >= args.refresh_period:
                # refresh period elapsed, send anyway
                send = True
        else:
            send = True
            markerset[name] = {'time_refresh': now, 'time': now, 'id': current_index, 'pos': posList }
            current_index += 1

        if args.very_verbose:
            print(name, posList, time)

        if send:
            if args.verbose and (not args.very_verbose):
                print(name, posList, now)

            # build list of 2D points and compute convex hull
            points = [(pos[X_AXIS], Y_SIGN*pos[Y_AXIS]) for pos in posList]
            hull = ConvexHull(points)
            # build lists of polygon corners to display in lat long
            latitudes = [ int(1e7 * (lat0 + np.rad2deg(points[i][1] / R_earth))) for i in hull.vertices ]
            longitudes = [ int(1e7 * (long0 + np.rad2deg(points[i][0] / R_cos_lat0))) for i in hull.vertices ]

            # send SHAPE message
            shape = PprzMessage("ground", "SHAPE")
            shape['id'] = markerset[name]['id']
            shape['linecolor'] = '"{}"'.format(args.color)
            shape['fillcolor'] = '"{}"'.format(args.color)
            shape['opacity'] = 1 # light
            shape['shape'] = 1 # polygon
            shape['status'] = 0 # create or update
            shape['latarr'] = latitudes
            shape['lonarr'] = longitudes
            shape['radius'] = 0. # not relevant
            if args.show_name:
                shape['text'] = name
            else:
                shape['text'] = '" "'
            ivy.send(shape)
            markerset[name]['time_refresh'] = now
            sleep(0.01)

def check_timeout():
    '''
    check timeout and if needed, remove object from scene and list
    '''
    now = time()
    for name in list(markerset.keys()):
        if now - markerset[name]['time'] > args.timeout:
            shape = PprzMessage("ground", "SHAPE")
            shape['id'] = markerset[name]['id']
            shape['status'] = 1 # delete
            # fill the rest even if not needed
            shape['linecolor'] = '" "'
            shape['fillcolor'] = '" "'
            shape['opacity'] = 0
            shape['shape'] = 1
            shape['latarr'] = [0]
            shape['lonarr'] = [0]
            shape['radius'] = 0.
            shape['text'] = '" "'
            ivy.send(shape)
            del markerset[name]


# start natnet interface
natnet_version = (3,0,0,0)
if args.old_natnet:
    natnet_version = (2,9,0,0)
natnet = NatNetClient(
        server=args.server,
        markerSetListener=receiveMarkerSet,
        dataPort=args.data_port,
        commandPort=args.command_port,
        verbose=args.very_verbose,
        version=natnet_version)


print("Starting Object Display interface at %s" % (args.server))
try:
    # Start up the streaming client.
    # This will run perpetually, and operate on a separate thread.
    natnet.run()
    while True:
        sleep(1)
        check_timeout()
except (KeyboardInterrupt, SystemExit):
    print("Shutting down ivy and natnet interfaces...")
    natnet.stop()
    ivy.shutdown()
except OSError:
    print("Natnet connection error")
    natnet.stop()
    ivy.stop()
    exit(-1)

