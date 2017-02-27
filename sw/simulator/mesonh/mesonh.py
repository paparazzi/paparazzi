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
# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_SRC = getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

M_IN_KM = 1000.

atm = None
origin = np.array([0, 0, 0, 0])
scale = np.array([1., 1/M_IN_KM, 1/M_IN_KM, 1/M_IN_KM])

#ivy_req = None
#ivy_mes = None


#nps_udp_socket = None
#nps_udp_dest = None

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


def ivy_callback(acid, msg):
    """
    Ivy callback for non-request paparazzi messages.
    
    NPS_SPEED_POS message is used to now ac position and send wind
    data to NPS
    """
    if msg.msg_class == "telemetry" and msg.name == "NPS_SPEED_POS":
        north, east, down = float(msg.ltpp_x),\
            float(msg.ltpp_y),\
            float(msg.ltpp_z)
        up = -down

        weast, wnorth, wup = get_wind(east, north, up)

        wup *= -1

        send_wind_nps_udp(acid, weast, wnorth, wup)

        # some debug print
        ivy_callback.acid_print[acid] = "[{}] pos {:.2f} {:.2f} {:.2f} wind {:.2f} {:.2f} {:.2f}"\
            .format(acid, east, north, up, weast, wnorth, wup)
        if ivy_callback.last_print + 1 <= time.time():
            for s in ivy_callback.acid_print.viewvalues():
                print(s)
            ivy_callback.last_print = time.time()
            ivy_callback.acid_print.clear()


ivy_callback.last_print = time.time()
ivy_callback.acid_print = {}


#def send_wind_nps_udp(acid, wnorth, weast, wup):
#
#    if acid in send_wind_nps_udp.acids_nosend:
#        return
#    if acid not in nps_udp_dest:
#        print("WARN: Aircraft id {} has no NPS port defined, "
#              "not sending wind (will not be displayed again)."
#              .format(acid),
#              file=sys.stderr)
#        send_wind_nps_udp.acids_nosend[acid] = True
#        return
#
#    elapsed = time.time() - start_time
#    (speed, heading) = cmath.polar(wnorth + weast*1j)
#    values = (elapsed, wnorth, weast, wup, heading, speed, 0x12345678)
#    message = send_wind_nps_udp.packer.pack(*values)
#    nps_udp_socket.sendto(message, nps_udp_dest[acid])
#
#send_wind_nps_udp.packer = struct.Struct("d f f f f f I")
#send_wind_nps_udp.acids_nosend = {}


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
    #argp.add_argument("-n", "--nps-host", required=False, action="store",
    #                  default="127.0.0.1",
    #                  help="NPS host")
    #argp.add_argument("-p", "--nps-ports", required=False, type=int,
    #                  default=[], nargs="+",
    #                  help="NPS Environment packets listen Port(s)")
    #argp.add_argument("-i", "--nps-acids", required=False, type=int,
    #                  default=[], nargs="+",
    #                  help="NPS aircraft id(s), corresponding to nps "
    #                  "port(s) argument.")
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
