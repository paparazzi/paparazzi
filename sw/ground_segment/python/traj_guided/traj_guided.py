#!/usr/bin/env python

import sys
from os import path, getenv
from threading import Event
from time import sleep
import numpy as np

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.message import PprzMessage
from pprz_connect import PprzConnect, PprzConfig
from guided_mode import GuidedMode
from settings import PprzSettingsManager
import trajectory_tools

class TrajectoryManager():

    def __init__(self, traj, ac_id=None, pprz_connect=None, verbose=False):
        '''
        Initialized trajectory manager for a single aircraft.
        By default, the PprzConnect node is created and the first drone to connect is used.
        In order to manage severral trajectories, the connect node should be started
        externaly and given to the manager with a specific aircraft ID.
        '''
        self.traj = traj
        self.ac_id = ac_id
        self.wait_event = Event()
        self.pos_ned = np.zeros((3,1))
        self.dist_threshold = 0.2
        self.pprz_connect = pprz_connect
        self.settings = None
        self.guided = None
        self.verbose = verbose

    def connect(self):
        if self.pprz_connect is None:
            self.pprz_connect = PprzConnect(notify=self.connect_cb)
            if self.verbose:
                print("Waiting aircraft to connect")
            self.wait_event.wait(None)
        # get conf
        if self.ac_id is not None:
            conf = self.pprz_connect.conf_by_id(str(self.ac_id))
            if self.verbose:
                print(f"Found aircraft {conf.id} ({conf.name})")
            self.pprz_connect.ivy.subscribe(self.flight_param_cb, PprzMessage("telemetry", "ROTORCRAFT_FP"))
            self.settings = PprzSettingsManager(conf.settings, conf.id, self.pprz_connect.ivy)
            self.guided = GuidedMode(self.pprz_connect.ivy)
            if self.verbose:
                print(f"Guided interface initialized for AC {conf.id}")
        else:
            print("Connected but no ac_id specified. Leaving")
            exit()

    def connect_cb(self, conf):
        ''' get aircraft config '''
        if self.ac_id is None:
            self.ac_id = int(conf.id)
        if self.ac_id == int(conf.id):
            self.wait_event.set()

    def flight_param_cb(self, sender, msg):
        if self.ac_id == sender:
            x = float(msg['north']) / 2**8
            y = float(msg['east']) / 2**8
            z = -float(msg['up']) / 2**8
            self.pos_ned = np.array([x, y, z])

    def set_guided_mode(self):
        if self.settings is not None:
            self.settings['auto2'] = 'Guided'
            self.guided.move_at_ned_vel(self.ac_id) # set zero speed

    def set_nav_mode(self):
        if self.settings is not None:
            self.settings['auto2'] = 'Nav'

    def shutdown(self):
        if self.pprz_connect is not None:
            self.pprz_connect.shutdown()
        if self.verbose:
            print(f'trajectory manager shutdown')

    def goto_enu_pos(self, east:float, north:float, up:float, heading:float=0.0, blocking=False):
        if self.guided is not None:
            self.guided.goto_enu(self.ac_id, east, north, up, heading)
            if blocking:
                target = np.array([north, east, -up])
                while np.linalg.norm(self.pos_ned - target) > self.dist_threshold:
                    sleep(0.1)

    def goto_ned_pos(self, north:float, east:float, down:float, heading:float=0.0, blocking=False):
        if self.guided is not None:
            self.guided.goto_ned(self.ac_id, north, east, down, heading)
            if blocking:
                target = np.array([north, east, down])
                while np.linalg.norm(self.pos_ned - target) > self.dist_threshold:
                    sleep(0.1)

    def set_full_ned(self, pos_ned:np.ndarray, vel_ned:np.ndarray, accel_ned:np.ndarray, heading:float=0.0):
        if self.guided is not None:
            self.guided.set_full_ned(self.ac_id,
               x=pos_ned[0], y=pos_ned[1], z=pos_ned[2],
               vx=vel_ned[0], vy=vel_ned[1], vz=vel_ned[2],
               ax=accel_ned[0], ay=accel_ned[1], az=accel_ned[2],
               heading=heading)
        if self.verbose:
            print(f'guided full set: p={pos_ned}, v={vel_ned}, a={accel_ned}, h={heading}')


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Follow a trajectory in guided mode")
    parser.add_argument("-i", "--ac_id", dest='ac_id', type=int, help="aircraft ID, first to connect if not specified")
    parser.add_argument("-f", "--freq", dest='freq', default=10., type=float, help="command update frequency")
    parser.add_argument("--transpose", dest='transpose', action=argparse.BooleanOptionalAction, default=False, type=bool, help="input data is in line and should be transposed")
    parser.add_argument("--frame", dest='frame', default='ned', choices=['enu','ned'], help='input data frame type')
    parser.add_argument("-v", "--verbose", dest='verbose', action='store_true', default=False, help="print more info and debug messages")
    parser.add_argument("file", help="input CSV file name")
    args = parser.parse_args()

    frames = {'enu':trajectory_tools.FrameType.ENU, 'ned':trajectory_tools.FrameType.NED}
    data = trajectory_tools.extract_csv(args.file, args.transpose)
    if data is None:
        print("No data. Leaving")
        exit()
    trajectory = trajectory_tools.TrajectoryInterpolator(data, frames[args.frame], verbose=args.verbose)
    period = 1./args.freq
    traj_manager = TrajectoryManager(trajectory, args.ac_id, verbose=args.verbose)

    try:
        traj_manager.connect()
        traj_manager.set_guided_mode()
        sleep(1)
        initial_state = trajectory.get_full_state(trajectory.get_initial_time())
        if args.verbose:
            print(f'initial position: {initial_state[0]}, {initial_state[3]}')
        traj_manager.goto_ned_pos(initial_state[0][0], initial_state[0][1], initial_state[0][2], initial_state[3], blocking=True)
        time_sync = np.arange(trajectory.get_initial_time(), trajectory.get_final_time(), period)
        for t in time_sync:
            pos, vel_w, accel_w, psi = trajectory.get_full_state(t)
            traj_manager.set_full_ned(pos, vel_w, accel_w, psi)
            sleep(period)
        sleep(3)
    except (KeyboardInterrupt,SystemExit):
        print("Stopping on request")
    finally:
        traj_manager.set_nav_mode()
        sleep(1)
        traj_manager.shutdown()


if __name__ == '__main__':
    main()
