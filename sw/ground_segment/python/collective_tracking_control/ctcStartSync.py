#!/usr/bin/env python

# Script to start the collective tracking algorithm in each aircraft at the same time.

from __future__ import print_function

import time
import sys
import numpy as np
from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

list_ids = []
interface = IvyMessagesInterface("CTC_Start")

if len(sys.argv) != 3:
    print("Usage: dcfInitTables idNavBlock ids.txt")
    interface.shutdown()
    exit()

idNavBlock = int(sys.argv[1])
ids = np.loadtxt(sys.argv[2])
list_ids = np.ndarray.tolist(ids)

time.sleep(2)

for i in list_ids:
    msg = PprzMessage("ground", "JUMP_TO_BLOCK")
    msg['ac_id'] = int(i)
    msg['block_id'] = idNavBlock
    interface.send(msg)
    print(msg)

time.sleep(2)
interface.shutdown()
