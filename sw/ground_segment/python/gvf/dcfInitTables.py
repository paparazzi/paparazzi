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
interface = IvyMessagesInterface("DCF")

if len(sys.argv) != 6:
    print("Usage: dcfInitTables topology.txt desired_sigma.txt ids.txt radius k")
    interface.shutdown()
    exit()

B = np.loadtxt(sys.argv[1])
desired_sigmas = np.loadtxt(sys.argv[2])*np.pi/180.0
ids = np.loadtxt(sys.argv[3])
radius = float(sys.argv[4])
k = float(sys.argv[5])

list_ids = np.ndarray.tolist(ids)

if np.size(ids) != np.size(B,0):
    print("The ammount of aircrafts in the topology and ids does not match")
    interface.shutdown()
    exit()

time.sleep(2)
for i in list_ids:
    msg = PprzMessage("datalink", "DCF_REG_TABLE")
    msg['ac_id'] = int(i)
    msg['desired_theta'] = 2
    print(msg)
    interface.send(msg)

time.sleep(2)
interface.shutdown()
