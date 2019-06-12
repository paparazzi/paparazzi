#!/usr/bin/env python
#
# Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
PPRZLINK Proxy is a tool to allow multiple aircraft simulations
with air-to-air communication using the PPRZLINK v2 protocol
using UDP.
As aircarft can't be identified by their IP addresses (most of the
time, all simulators are running on the same computer), each agent (GCS,
aircraft) are using different ports and the mapping between ports and IDs
is done by this tool.

usage examples:
    ./pprzlink_proxy.py --ac=101:4244:4245 --ac=102:4256:4247
    ./pprzlink_proxy.py --ac=101:4244:4245 --ac=102:4256:4247 --addr=192.168.1.1
    ./pprzlink_proxy.py --ac=101:4244:4245 --ac=102:192.168.1.2:4256:4247 --gcs=192.168.1.2
    ./pprzlink_proxy.py --script=proxy.txt
        where 'proxy.txt' contains a list of parameters with the same format than the command line options (possibly one per line)
'''

from __future__ import print_function

import os
import sys
import time

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PAPARAZZI_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),'../../../')))
sys.path.append(PAPARAZZI_HOME + "/var/lib/python")

import pprzlink.udp

DEFAULT_ADDRESS= "127.0.0.1"

# GCS ID, OUT port, IN port
default_gcs_conf = [0, DEFAULT_ADDRESS, 4243, 4242]


class Proxy:
    def __init__(self, gcs_conf=default_gcs_conf, ac_conf=None, verbose=False):
        # test for valid type of ac_conf
        if ac_conf is None or not isinstance(ac_conf, list) or len(ac_conf) == 0:
            raise ValueError('No valid aircraft configuration specified')

        self.gcs_id = int(gcs_conf[0])
        self.gcs_addr = gcs_conf[1]
        self.gcs = pprzlink.udp.UdpMessagesInterface(self.proccess_gcs_msg, False,
                int(gcs_conf[3]), int(gcs_conf[2]), 'datalink') # crossing ports here
        self.acs = []
        for ac in ac_conf:
            self.acs.append((int(ac[0]), ac[1],
                pprzlink.udp.UdpMessagesInterface(self.proccess_mav_msg, False,
                    int(ac[3]), int(ac[2]), 'telemetry', None))) # crossing ports here
        self.verbose = verbose

    def proccess_mav_msg(self, sender, address, msg, length, receiver_id=None, component_id=None):
        if receiver_id is None:
            return
        if self.verbose:
            print("new message from %i (%s) [%d Bytes]: %s" % (sender, address, length, msg))
        if receiver_id == self.gcs_id:
            # normal telemetry message for the GCS
            if self.verbose:
                print("sending to %i: %s" % (receiver_id, msg))
            self.gcs.send(msg, sender, self.gcs_addr, receiver_id, component_id)
        else:
            for (ac_id, ac_addr, m) in self.acs:
                if receiver_id == ac_id:
                    # send air to air message to correct MAV
                    if self.verbose:
                        print("sending to %i (%i): %s" % (receiver_id, ac_id, msg))
                    m.send(msg, sender, ac_addr, receiver_id, component_id)

    def proccess_gcs_msg(self, sender, address, msg, length, receiver_id=None, component_id=None):
        if receiver_id is None:
            return
        if self.verbose:
            print("new message from %i (%s) [%d Bytes]: %s" % (sender, address, length, msg))
        if msg.broadcasted:
            for (ac_id, ac_addr, m) in self.acs:
                # broadcast to all MAVs
                if self.verbose:
                    print("sending to %i: %s" % (receiver_id, msg))
                m.send(msg, sender, ac_addr, receiver_id)
        else:
            try:
                receiver_id = msg['ac_id']
            except:
                receiver_id = None
            for (ac_id, ac_addr, m) in self.acs:
                if receiver_id == ac_id:
                    # send datalink message to correct MAV
                    if self.verbose:
                        print("sending to %i (%i): %s" % (receiver_id, ac_id, msg))
                    m.send(msg, sender, ac_addr, receiver_id)

    def run(self):
        print ('Starting pprzlink proxy')
        self.gcs.start()
        for (ac_id, ac_addr, m) in self.acs:
            m.start()

        try:
            while True:
                time.sleep(.001)
        except (KeyboardInterrupt, SystemExit):
            print ("Stopping pprzlink proxy")
            for (ac_id, ac_addr, m) in self.acs:
                m.stop()
            self.gcs.stop()
            self.gcs.join()


if __name__ == '__main__':
    import argparse

    class LoadFromFile (argparse.Action):
        '''
        local file parser
        read data from a file and pass them to the argument parser
        '''
        def __call__ (self, parser, namespace, values, option_string = None):
            with values as f:
                parser.parse_args(f.read().split(), namespace)

    # parse args
    parser = argparse.ArgumentParser()
    parser.add_argument('--gcs', dest='gcs_conf', help="GCS configuration with format [ID:][IP:][PORT_OUT:PORT_IN]")
    parser.add_argument('--ac', dest='acs_conf', action='append', default=[], help="AC configuration with format ID:[IP:]PORT_OUT:PORT_IN (multiple possible)")
    parser.add_argument('--addr', dest='address', default=DEFAULT_ADDRESS, help="Set default IP address")
    parser.add_argument('--script', type=open, action=LoadFromFile)
    parser.add_argument('-v', '--verbose', dest='verbose', action='store_true', help="display debug messages")
    args = parser.parse_args()

    # split GCS conf
    gcs_conf = default_gcs_conf
    if args.address is not None:
        gcs_conf[1] = args.address
    if args.gcs_conf is not None:
        gc = args.gcs_conf.split(':')
        if len(gc) == 1 and '.' in gc[0]:
            gcs_conf[1] = gc[0] # IP address
        elif len(gc) == 1:
            gcs_conf[0] = int(gc[0]) # id
        elif len(gc) == 2 and '.' in gc[1]:
            gcs_conf[0] = int(gc[0]) # id
            gcs_conf[1] = gc[1] # IP address
        elif len(gc) == 2:
            gcs_conf[2] = int(gc[0]) # port out
            gcs_conf[3] = int(gc[1]) # port in
        elif len(gc) == 3 and '.' in gc[0]:
            gcs_conf[1] = gc[0] # IP address
            gcs_conf[2] = int(gc[1]) # port out
            gcs_conf[3] = int(gc[2]) # port in
        elif len(gc) == 3:
            gcs_conf[0] = int(gc[0]) # id
            gcs_conf[2] = int(gc[1]) # port out
            gcs_conf[3] = int(gc[2]) # port in
        elif len(gc) == 4:
            gcs_conf[0] = int(gc[0]) # id
            gcs_conf[1] = gc[1] # IP address
            gcs_conf[2] = int(gc[2]) # port out
            gcs_conf[3] = int(gc[3]) # port in
        else:
            print("invalid GCS conf parameters")
            exit()

    # split AC conf
    acs = []
    for ac in args.acs_conf:
        a = ac.split(':')
        if len(a) == 3 and args.address is not None:
            a.insert(1, args.address)
        elif len(a) == 3:
            a.insert(1, DEFAULT_ADDRESS)
        elif len(a) != 4:
            print("invalid number of AC conf parameters")
            exit()
        acs.append(a)


    try:
        proxy = Proxy(gcs_conf, acs, args.verbose)
        proxy.run()
    except ValueError as e:
        print(e)


