#!/usr/bin/env python3
import time
import sys
import os
import math
import argparse
import matplotlib.pyplot as plt

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_HOME + "/sw/ground_segment/python/natnet3.x")

from NatNetClient import NatNetClient


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame(id, position, rotation):
    # print( "Received frame for rigid body", id )
    global pos_x, pos_y, pos_z
    global track_id
    if track_id and id != track_id:
        return
    
    pos_x = position[0]
    pos_y = position[1]
    pos_z = position[2]


def main(args):
    global track_id
    track_id = args.id
    
    global pos_x, pos_y, pos_z
    pos_x, pos_y, pos_z = 0.0, 0.0, 0.0
    fig = plt.figure()
    plt.axis([-6, 6, -6, 6])
    
    # This will create a new NatNet client
    streamingClient = NatNetClient(
        server=args.server,
        multicast=args.multicast,
        commandPort=args.commandPort,
        dataPort=args.dataPort,
        rigidBodyListListener=receiveRigidBodyFrame)
    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    streamingClient.run()
    
    time.sleep(2)
    print('Start tracking')
    if args.outputfile:
        file = open(args.outputfile, 'w')
        file.write('timestamp, x, y, z\n')
    
    old_z = pos_z
    old_x = pos_x
    distance = 0
    start_time = time.time()
    pre_time = time.time()
    
    while plt.fignum_exists(fig.number):
        if args.outputfile:
            data = '{}, {}, {}, {}\n'.format(int((time.time() - start_time) * 1000), pos_x, pos_y, pos_z)
            file.write(data)
            
        h = math.hypot(pos_z - old_z, pos_x - old_x)
        if h > 0.20:
            distance += h
            old_z = pos_z
            old_x = pos_x
        if time.time() - pre_time > 0.5:
            print("distance:%3.4f m; time_step:%d" % (distance, int((time.time() - start_time) * 2)))
            pre_time = time.time()
        plt.plot(pos_z, pos_x, 'ro')
        plt.draw()
        plt.pause(0.001)
        time.sleep(0.01)
    
    streamingClient.stop()
    if args.outputfile:
        file.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--server', default="169.254.201.120")
    parser.add_argument('--multicast', default="239.255.42.99")
    parser.add_argument('--commandPort', type=int, default=1510)
    parser.add_argument('--dataPort', type=int, default=1511)
    parser.add_argument('--id', type=int, default=None)
    parser.add_argument('--outputfile', type=str, default=None)
    args = parser.parse_args()
    
    main(args)
