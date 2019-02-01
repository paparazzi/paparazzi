#!/usr/bin/env python

from __future__ import absolute_import, print_function, division
from mesonh_atmosphere import MesoNHAtmosphere
import os
import sys
import signal
import time
import socket
import struct
import cmath
import numpy as np
from os import getenv
# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

M_IN_KM = 1000.

atm = None
origin = np.array([0, 0, 0, 0])
scale = np.array([1., 1/M_IN_KM, 1/M_IN_KM, 1/M_IN_KM])

start_time = time.time()


def get_wind(east, north, up):
    t = time.time() - start_time
    print("east :",east)
    print("north :",north)
    print("up :",up)
    loc = np.array([t, up, east, north])
    loc = loc*scale + origin
    print("loc:",loc)
    weast, wnorth, wup = atm.get_wind(loc)
    return weast, wnorth, wup


def ivy_request_callback(sender, msg, resp, *args, **kwargs):
    """
        Ivy Callback for Paparazzi Requests
    """

    if msg.msg_class == "ground" and msg.name == "WORLD_ENV_REQ":
        return worldenv_cb(msg, resp)
    else:
        return None


#def worldenv_cb(m, r):
def worldenv_cb(ac_id, msg):
    """
        Callback for paparazzi WORLD_ENV requests
    """
    # request location (in meters)
    east, north, up = float(msg.get_field(3)),\
        float(msg.get_field(4)),\
        float(msg.get_field(5))
    up *= -1
    # convert in km + translation with mesoNH origin
    weast, wnorth, wup = get_wind(east, north, up)
    print("wind_est:")
    print(weast)
    print(wnorth)
    print(wup)
    msg_back=PprzMessage("ground", "WORLD_ENV")
    msg_back.set_value_by_name("wind_east",weast)
    msg_back.set_value_by_name("wind_north",wnorth)
    msg_back.set_value_by_name("wind_up",wup)
    msg_back.set_value_by_name("ir_contrast",400)
    msg_back.set_value_by_name("time_scale",1)
    msg_back.set_value_by_name("gps_availability",1)
    ivy.send(msg_back,None)


def signal_handler(signal, frame):
    print('\nShutting down IVY...')
    ivy.shutdown()
    print("Done.")


def main():
    # parse arguments
    import argparse as ap

    argp = ap.ArgumentParser(description="Environment variables provider "
                             "for Paparazzi simulation from MesoNH data")

    argp.add_argument("-t", "--time-step", required=True, type=int,
                      help="Duration of a time step between MesoNH Files.")
    argp.add_argument("-f", "--files", required=True, nargs='+',
                      help="MesoNH netCDF files, in temporal ordering")
    argp.add_argument("-x", "--origin-x", required=False, type=float,
                      default=0.,
                      help="Origin translation x.")
    argp.add_argument("-y", "--origin-y", required=False, type=float,
                      default=0.,
                      help="Origin translation y.")
    argp.add_argument("-z", "--origin-z", required=False, type=float,
                      default=0.,
                      help="Origin translation z.")
    arguments = argp.parse_args()

    print(arguments)

    # register signal handler for ctrl+c to stop the program
    signal.signal(signal.SIGINT, signal_handler)

    # origin for translation from paparazzi to mesoNH frame
    global origin
    origin = np.array([0, arguments.origin_z, arguments.origin_x, arguments.origin_y])

    # build atmosphere simulation source
    global atm
    atm = MesoNHAtmosphere(arguments.files, arguments.time_step, tinit=0)

    # init ivy and register callback for WORLD_ENV_REQ and NPS_SPEED_POS
    global ivy
    ivy = IvyMessagesInterface("MesoNH");
    ivy.subscribe(worldenv_cb,'(.* WORLD_ENV_REQ .*)')

    # wait for ivy to stop
    from ivy.std_api import IvyMainLoop

    signal.pause()


if __name__ == '__main__':
    main()
