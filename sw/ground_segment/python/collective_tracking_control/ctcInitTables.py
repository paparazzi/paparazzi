#!/usr/bin/env python
from __future__ import print_function

import time
import sys
import wx
import numpy as np
import sys
from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage 
from settings_xml_parse import PaparazziACSettings

list_ids = []
interface = IvyMessagesInterface("CTC")

if len(sys.argv) != 2:
    print("Usage: dcfInitTables ids.txt")
    interface.shutdown()
    exit()

ids = np.loadtxt(sys.argv[1])
list_ids = np.ndarray.tolist(ids)

time.sleep(2)

for i in list_ids:
    msg_clean = PprzMessage("datalink", "CTC_CLEAN_TABLE")
    msg_clean['ac_id'] = int(i)

    print(msg_clean)
    interface.send(msg_clean)

for i in list_ids:
    msg = PprzMessage("datalink", "CTC_REG_TABLE")
    msg['ac_id'] = int(i)
    for ii in list_ids:
        if(i != ii):
            msg['nei_id'] = int(ii)
            interface.send(msg)
            print(msg)

time.sleep(2)
interface.shutdown()
