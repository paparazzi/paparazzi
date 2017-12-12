#!/usr/bin/env python

import os
import sys
import time

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PAPARAZZI_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),'../../../')))
sys.path.append(PAPARAZZI_HOME + "/var/lib/python")

import pprzlink.udp

#DEFAULT_ADDRESS= "127.255.255.255"
DEFAULT_ADDRESS= "127.0.0.1"

# GCS ID, OUT port, IN port
gcs_conf = [0, 4243, 4242]

# list ID, OUT port, IN port
mavs_conf = [
        [102, 4244, 4245],
        [103, 4246, 4247],
        [104, 4248, 4249],
        [2, 4250, 4251]
        ]

verbose = False

class Proxy:
    def __init__(self):
        self.gcs = pprzlink.udp.UdpMessagesInterface(self.proccess_gcs_msg, False, gcs_conf[2], gcs_conf[1], 'datalink') # crossing ports here
        self.mavs = []
        for mav in mavs_conf:
            self.mavs.append((mav[0], pprzlink.udp.UdpMessagesInterface(self.proccess_mav_msg, False, mav[2], mav[1], 'telemetry', None))) # crossing ports here

    def proccess_mav_msg(self, sender, address, msg, length, receiver_id=None, component_id=None):
        if receiver_id is None:
            return
        if verbose:
            print("new message from %i (%s) [%d Bytes]: %s" % (sender, address, length, msg))
        if receiver_id == gcs_conf[0]:
            # normal telemetry message for the GCS
            if verbose:
                print("sending to %i: %s" % (receiver_id, msg))
            self.gcs.send(msg, sender, DEFAULT_ADDRESS, receiver_id, component_id)
        else:
            for (ac_id, m) in self.mavs:
                if receiver_id == ac_id:
                    # send air to air message to correct MAV
                    if verbose:
                        print("sending to %i (%i): %s" % (receiver_id, ac_id, msg))
                    m.send(msg, sender, DEFAULT_ADDRESS, receiver_id, component_id)

    def proccess_gcs_msg(self, sender, address, msg, length, receiver_id=None, component_id=None):
        if receiver_id is None:
            return
        if verbose:
            print("new message from %i (%s) [%d Bytes]: %s" % (sender, address, length, msg))
        if msg.broadcasted:
            for (ac_id, m) in self.mavs:
                # broadcast to all MAVs
                if verbose:
                    print("sending to %i: %s" % (receiver_id, msg))
                m.send(msg, sender, DEFAULT_ADDRESS, receiver_id)
        else:
            try:
                receiver_id = msg['ac_id']
            except:
                receiver_id = None
            for (ac_id, m) in self.mavs:
                if receiver_id == ac_id:
                    # send datalink message to correct MAV
                    if verbose:
                        print("sending to %i (%i): %s" % (receiver_id, ac_id, msg))
                    m.send(msg, sender, DEFAULT_ADDRESS, receiver_id)

    def run(self):
        #print ('Starting proxy for protocol version %s' % (messages_xml_map.PROTOCOL_VERSION))
        print ('Starting proxy')
        self.gcs.start()
        for (ac_id, m) in self.mavs:
            m.start()

        try:
            while True:
                time.sleep(.001)
        except KeyboardInterrupt:
            print ("Stopping Proxy")
            for (ac_id, m) in self.mavs:
                m.stop()
            self.gcs.stop()
            self.gcs.join()



if __name__ == '__main__':
    proxy = Proxy()
    proxy.run()

