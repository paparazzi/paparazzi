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

if len(sys.argv) != 4:
    print("Usage: dcfInitTables topology.txt desired_sigma.txt ids.txt")
    interface.shutdown()
    exit()

B = np.loadtxt(sys.argv[1])
desired_sigmas = np.loadtxt(sys.argv[2])*np.pi/180.0
ids = np.loadtxt(sys.argv[3])

list_ids = np.ndarray.tolist(ids)

if np.size(ids) != np.size(B,0):
    print("The ammount of aircrafts in the topology and ids do not match")
    interface.shutdown()
    exit()
time.sleep(2)

if len(list_ids) == 2:
    B.shape = (2,1)

for count, column in enumerate(B.T):
    index = np.nonzero(column)
    i = index[0]

    msg_clean_a = PprzMessage("datalink", "DCF_REG_TABLE")
    msg_clean_a['ac_id'] = int(list_ids[i[0]])
    msg_clean_a['nei_id'] = 0
    msg_clean_b = PprzMessage("datalink", "DCF_REG_TABLE")
    msg_clean_b['ac_id'] = int(list_ids[i[1]])
    msg_clean_b['nei_id'] = 0

    interface.send(msg_clean_a)
    interface.send(msg_clean_b)

for count, column in enumerate(B.T):
    index = np.nonzero(column)
    i = index[0]

    msga = PprzMessage("datalink", "DCF_REG_TABLE")
    msga['ac_id'] = int(list_ids[i[0]])
    msga['nei_id'] = int(list_ids[i[1]])
    if len(list_ids) == 2:
        msga['desired_sigma'] = int(desired_sigmas)
    else:
        msga['desired_sigma'] = int(desired_sigmas[count])
    interface.send(msga)

    msgb = PprzMessage("datalink", "DCF_REG_TABLE")
    msgb['ac_id'] = int(list_ids[i[1]])
    msgb['nei_id'] = int(list_ids[i[0]])
    if len(list_ids) == 2:
        msgb['desired_sigma'] = int(desired_sigmas)
    else:
        msgb['desired_sigma'] = int(desired_sigmas[count])
    interface.send(msgb)

    print(msga)
    print(msgb)

time.sleep(2)
interface.shutdown()
