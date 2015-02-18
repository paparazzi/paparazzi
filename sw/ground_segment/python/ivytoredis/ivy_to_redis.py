#!/usr/bin/env python

from __future__ import print_function

import redis
import time
import signal
import argparse
import sys
import os

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../../')))
PPRZ_LIB_PYTHON = os.path.join(PPRZ_SRC, "sw/lib/python")
sys.path.append(PPRZ_LIB_PYTHON)

from ivy_msg_interface import IvyMessagesInterface

server = None


class Ivy2RedisServer():
    def __init__(self, redishost, redisport, verbose=False):
        self.verbose = verbose
        self.interface = IvyMessagesInterface(self.message_recv)
        self.r = redis.StrictRedis(host=redishost, port=redisport, db=0)
        self.keep_running = True
        print("Connected to redis server %s on port %i" % (redishost, redisport))

    def message_recv(self, ac_id, msg):
        # if ac_id is not 0 (i.e. telemetry from an aircraft) include it in the key
        # don't add it to the key for ground messages
        if ac_id:
            key = "{0}.{1}.{2}".format(msg.msg_class, msg.name, ac_id)
        else:
            key = "{0}.{1}".format(msg.msg_class, msg.name)
        if self.verbose:
            print("received message, key=%s, msg=%s" % (key, msg.to_json(payload_only=True)))
            sys.stdout.flush()
        self.r.publish(key, msg.to_json(payload_only=True))
        self.r.set(key, msg.to_json(payload_only=True))

    def run(self):
        while self.keep_running:
            time.sleep(0.1)

    def stop(self):
        self.keep_running = False
        self.interface.shutdown()


def signal_handler(signal, frame):
    global server
    server.stop()


def main():
    global server
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--server", help="hostname here redis runs", default="localhost")
    parser.add_argument("-p", "--port", help="port used by redis", type=int, default=6379)
    parser.add_argument("-v", "--verbose", dest="verbose", action="store_true")
    args = parser.parse_args()
    server = Ivy2RedisServer(args.server, args.port, args.verbose)
    signal.signal(signal.SIGINT, signal_handler)
    server.run()

if __name__ == '__main__':
    main()
