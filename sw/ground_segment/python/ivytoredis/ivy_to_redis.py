#!/usr/bin/env python

from __future__ import print_function

import redis
import time
import signal
import argparse
import sys
import os

PPRZ_HOME = os.getenv("PAPARAZZI_HOME")
sys.path.append(PPRZ_HOME + "/sw/lib/python")

from ivy_msg_interface import IvyMessagesInterface

server = None


class Ivy2RedisServer():
    def __init__(self, redishost, redisport, verbose=False):
        self.verbose = verbose
        self.interface = IvyMessagesInterface(self.message_recv)
        self.r = redis.StrictRedis(host=redishost, port=redisport, db=0)
        self.keep_running = True
        print("Connected to redis server %s on port %i" % (redishost, redisport))

    def message_recv(self, msg_class, msg_name, ac_id, values):
        # if ac_id is not 0 (i.e. telemetry from an aircraft) include it in the key
        # don't add it to the key for ground messages
        if ac_id:
            key = "{0}.{1}.{2}".format(msg_class, msg_name, ac_id)
        else:
            key = "{0}.{1}".format(msg_class, msg_name)
        if self.verbose:
            print("received message, key=%s, values=%s" % (key, ' '.join(values)))
            sys.stdout.flush()
        self.r.publish(key, values)
        self.r.set(key, values)

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
