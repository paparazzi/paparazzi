#!/usr/bin/env python
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
interface = IvyMessagesInterface("CTC")

if len(sys.argv) != 3:
    print("Usage: dcfInitTables target_id aircraft_ids.txt")
    interface.shutdown()
    exit()

target_id = int(sys.argv[1])
ids = np.loadtxt(sys.argv[2])
list_ids = np.ndarray.tolist(ids)

time.sleep(2)

for i in list_ids:
    msg_clean = PprzMessage("datalink", "CTC_CLEAN_TABLE")
    msg_clean['ac_id'] = int(i)

    print(msg_clean)
    interface.send(msg_clean)

for i in list_ids :
    msg = PprzMessage("datalink", "CTC_REG_TABLE")
    msg['ac_id'] = int(i)
    for ii in list_ids:
        if (i != ii):
            msg['nei_id'] = int(ii)
            interface.send(msg)
            print(msg)

time.sleep(2)
interface.shutdown()
