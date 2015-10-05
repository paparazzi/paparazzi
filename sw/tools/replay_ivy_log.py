#!/usr/bin/env python

from __future__ import absolute_import, division, print_function

from ivy.std_api import *
import logging
import argparse
import os
from time import sleep

class IvyLogSender(object):
    def __init__(self, verbose=False):
        self.ivy_id = 0
        self.verbose = verbose
        IvyInit("LogSender %i" % os.getpid(), "READY", 0, lambda x,y: y, lambda x,y: y)
        if verbose:
            logging.getLogger('Ivy').setLevel(logging.INFO)
        else:
            logging.getLogger('Ivy').setLevel(logging.WARN)
        IvyStart("")

    def shutdown(self):
        IvyStop()

    def send(self, msg):
        IvySendMsg(msg)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="log file with one ivy msg per line")
    parser.add_argument("-d", "--delay", type=float, default=0.2, help="delay between messages (default: 0.2s)")
    parser.add_argument("-v", "--verbose", action="store_true", help="increase output verbosity")
    args = parser.parse_args()

    ils = IvyLogSender(args.verbose)
    if args.verbose:
        loglevel = logging.DEBUG
    else:
        loglevel = logging.INFO
    logging.basicConfig(format='%(asctime)s %(levelname)s: %(message)s', level=loglevel)
    logging.info("Reading log file %s" % args.file)
    # seems there is a sleep needed before you can send something
    sleep(0.2)
    try:
        with open(args.file) as log:
            for line in log:
                logging.debug("Sending msg: %s" % str.strip(line))
                ils.send(line)
                sleep(args.delay)
    finally:
        ils.shutdown()
    print("Done.")
