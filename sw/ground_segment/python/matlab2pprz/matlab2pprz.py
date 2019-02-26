#!/usr/bin/env python3
#
# Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
Forward rigid body positions and orientation from Natnet to Matlab/Simulink
via UDP ports and receive commands that are forwarded over Ivy
'''

from __future__ import print_function

import sys
from os import path, getenv
from time import time, sleep
import argparse
import math 
import socket
import struct

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")
sys.path.append(PPRZ_HOME + "/sw/ground_segment/python/natnet3.x")

from NatNetClient import NatNetClient

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

# parse args
parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('-ac', action='append', nargs=4,
                    metavar=('rigid_id','ac_id','udp_out'), help='tuple of rigid body, A/C id and UDP output port to Simulink (multiple possible)')
parser.add_argument('-b', '--ivy_bus', dest='ivy_bus', help="Ivy bus address and port")
parser.add_argument('-ns', '--natnet_server', dest='natnet_server', default="127.0.0.1", help="NatNet server IP address")
parser.add_argument('-m', '--multicast_addr', dest='multicast', default="239.255.42.99", help="NatNet server multicast address")
parser.add_argument('-dp', '--data_port', dest='data_port', type=int, default=1511, help="NatNet server data socket UDP port")
parser.add_argument('-cp', '--command_port', dest='command_port', type=int, default=1510, help="NatNet server command socket UDP port")
parser.add_argument('-ss', '--simulink_server', dest='simulink_server', default="127.0.0.1", help="Simulink server IP address")
parser.add_argument('-sp', '--simulink_in_port', dest='simulink_in_port', default=9090, help="Simulink input port")
parser.add_argument('-v', '--verbose', dest='verbose', action='store_true', help="display debug messages")
args = parser.parse_args()

if args.ac is None:
    print("At least one pair of rigid boby / AC id must be declared")
    exit()

# dictionary of ID associations
id_dict = { v[0]: {'ac_id':v[1], 'udp_out':v[2]} for v in args.ac }

# start ivy interface
if args.ivy_bus is not None:
    ivy = IvyMessagesInterface("matlab2pprz", ivy_bus=args.ivy_bus)
else:
    ivy = IvyMessagesInterface("matlab2pprz")
# prepare Ivy message
ivy_msg = PprzMessage("datalink", "JOYSTICK_RAW")

# start socket for communication with Matlab/Simulink
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(2.0)
    sock.bind(('0.0.0.0', args.simulink_in_port)) # FIXME one port for all or one for each ?
except OSError:
    print("Error: unable to open socket")


def receiveRigidBodyList( rigidBodyList, stamp ):
    for (body_id, pos, quat, valid) in rigidBodyList:
        if not valid:
            # skip if rigid body is not valid
            continue

        i = str(body_id)
        if i not in id_dict.keys():
            # unknown A/C ID
            continue
        ac_id = id_dict[i]['ac_id']
        port = int(id_dict[i]['udp_out'])

        # data array = pos + quat
        data = pos + quat

        msg = ac_id+" "+format(stamp,"+0.5f")+" "+" ".join(format(x,"+0.4f") for x in data)
        if args.verbose:
            print('sending to simulink ({},{}): {}'.format(args.simulink_server, port, msg))

        bindata = struct.pack('<fffffffff',float(ac_id),stamp,pos[0],pos[1],pos[2],quat[0],quat[1],quat[2],quat[3])
        sock.sendto(bindata,(args.simulink_server,port))

natnet = NatNetClient(
        server=args.natnet_server,
        rigidBodyListListener=receiveRigidBodyList,
        dataPort=args.data_port,
        commandPort=args.command_port)

def sendCommandMsg(cmd):
    # decode 5 int16 (short) and send other Ivy
    bindata = struct.unpack('<hhhhh',cmd)
    ivy_msg['ac_id'] = bindata[0]
    ivy_msg['roll'] = bindata[1]
    ivy_msg['pitch'] = bindata[2]
    ivy_msg['yaw'] = bindata[3]
    ivy_msg['throttle'] = bindata[4]
    ivy.send(ivy_msg)

print("Starting Natnet3.x/Ivy to Matlab/Simulink bridge between %s and %s" % (args.natnet_server, args.simulink_server))
try:
    # Start up the streaming client.
    # This will run perpetually, and operate on a separate thread.
    natnet.run()
    while True:
        try:
            (cmd, address) = sock.recvfrom(10) # receiving 5 int16 (id, roll, pitch, yaw, thrust)
            if len(cmd) == 10: # something to send
                sendCommandMsg(cmd)
        except socket.timeout:
            pass
except (KeyboardInterrupt, SystemExit):
    print("Shutting down socket, ivy and natnet interfaces...")
    natnet.stop()
    ivy.shutdown()
    sock.close()
except OSError:
    print("Natnet connection error")
    natnet.stop()
    ivy.stop()
    sock.close()
    exit(-1)

