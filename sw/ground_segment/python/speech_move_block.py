#!/usr/bin/env python

from __future__ import print_function
import wit
import sys
import json
from Tkinter import *

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



master = Tk()
wit.init()
wm = BlockJumper()
def startListening():
  try:
   wit.voice_query_start("3SV3MXJDHPS5ACDMZZLKJKSJJQPZONK2")

  except KeyboardInterrupt:
    print("Stopping on request")

def stopListening():
  try:
    response = wit.voice_query_stop()
    ans = json.loads(response)
    print(dir(response))
    print(ans["outcomes"])
    if len(ans["outcomes"])>0:
      bestOutcome = ans["outcomes"][0]
      entities = bestOutcome["entities"]
      if entities["number"]:
        goTo = entities["number"][0]["value"]
       
        sleep(0.1)# sleep shortly in oder to make sure Ivy is up, then message sent before shutting down again
        wm.jump_to_block(ac_id=202, block_id=goTo)
        sleep(0.1)
  except KeyboardInterrupt:
    print("Stopping on request")
 
b = Button(master, text="Start", command=startListening)
b2 = Button(master, text="Stop", command=stopListening)
b.pack()
b2.pack()
mainloop()
