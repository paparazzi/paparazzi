#!/usr/bin/env python

from __future__ import print_function

import sys
from os import path, getenv
from time import sleep

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")

from ivy_msg_interface import IvyMessagesInterface
from pprzlink.message import PprzMessage


class BlockJumper(object):
    def __init__(self, verbose=False):
        self.verbose = verbose
        self._interface = IvyMessagesInterface(self.message_recv)

    def message_recv(self, ac_id, msg):
        if self.verbose:
            print("Got msg %s" % msg.name)

    def shutdown(self):
        print("Shutting down ivy interface...")
        self._interface.shutdown()

    def __del__(self):
        self.shutdown()

    def jump_to_block(self, ac_id, block_id):
        msg = PprzMessage("ground", "JUMP_TO_BLOCK")
        msg['ac_id'] = ac_id
        msg['block_id'] = block_id
        print("Sending message: %s" % msg)
        self._interface.send(msg)


if __name__ == '__main__':
    try:
        wm = BlockJumper()
        # sleep shortly in oder to make sure Ivy is up, then message sent before shutting down again
        sleep(0.1)
        wm.jump_to_block(ac_id=202, block_id=8)
        sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping on request")
    wm.shutdown()
