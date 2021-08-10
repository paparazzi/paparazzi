#!/usr/bin/env python3
#
# Copyright (C) 2021 Freek van tienen <freek.v.tienen@gmail.com>
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
Read MAVLink messages from the herelink to get RSSI information
'''


from __future__ import print_function

import sys
from os import path, getenv
from time import time, sleep
import numpy as np
from collections import deque
import argparse
from pymavlink import mavutil

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

# parse args
parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('-ac', '--ac_id', dest='ac_id', type=int, help='Aircraft ID to save the RSSI information to')
parser.add_argument('-b', '--ivy_bus', dest='ivy_bus', help="Ivy bus address and port")
args = parser.parse_args()

if args.ac_id is None:
    print("You need to define an aircraft ID")
    exit()

# start ivy interface
if args.ivy_bus is not None:
    ivy = IvyMessagesInterface("herelink2ivy", ivy_bus=args.ivy_bus)
else:
    ivy = IvyMessagesInterface("herelink2ivy")

# Start a connection listening to a UDP port
conn = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

print("Starting Herelink RSSI for ac_id %d" % (args.ac_id))
try:
    # Start up the streaming client.
    while True:
        msg = conn.recv_match(type='RADIO_STATUS', blocking=True)
        if not msg or msg.get_type() == "BAD_DATA":
            continue
        
        pmsg = PprzMessage("telemetry", "RSSI_COMBINED")
        pmsg['remote_rssi'] = msg.remrssi
        pmsg['tx_power'] = 0
        pmsg['local_rssi'] = msg.rssi
        pmsg['local_noise'] = msg.noise
        pmsg['remote_noise'] = msg.remnoise
        ivy.send(pmsg, args.ac_id)
except (KeyboardInterrupt, SystemExit):
    print("Shutting down ivy interface...")
    ivy.shutdown()
