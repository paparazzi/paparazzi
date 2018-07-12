#!/usr/bin/env python
#!/usr/bin/env python
#
# Copyright (C) 2017 Hector Garcia de Marina <hgdemarina@gmail.com>
#                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.

'''
Init the aircraft's tables for the decentralized circular formations module
'''

import time
import sys
import wx
import numpy as np
import sys
import json
from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

class initTable:
    def __init__(self, config, verbose=False):
        self.verbose = verbose
        self.config = config
        self.ids = np.array(self.config['ids'])
        self.B = np.array(self.config['topology'])
        self.Zdesired = np.array(self.config['desired_intervehicle_angles'])
        self.list_ids = np.ndarray.tolist(self.ids)

        # Start IVY interface
        self._interface = IvyMessagesInterface("Init DCF tables")

    def __del__(self):
        self.stop()

    def stop(self):
        # Stop IVY interface
        if self._interface is not None:
            self._interface.shutdown()

    def init_dcftables(self):
        time.sleep(2)

        for count, column in enumerate(self.B.T):
            index = np.nonzero(column)
            i = index[0]

            # nei_id = 0, special msg to clean the table onboard
            msg_clean_a = PprzMessage("datalink", "DCF_REG_TABLE")
            msg_clean_a['ac_id'] = int(self.list_ids[i[0]])
            msg_clean_a['nei_id'] = 0
            msg_clean_b = PprzMessage("datalink", "DCF_REG_TABLE")
            msg_clean_b['ac_id'] = int(self.list_ids[i[1]])
            msg_clean_b['nei_id'] = 0

            self._interface.send(msg_clean_a)
            self._interface.send(msg_clean_b)

            if self.verbose:
                print(msg_clean_a)
                print(msg_clean_b)

        for count, column in enumerate(self.B.T):
            index = np.nonzero(column)
            i = index[0]

            msga = PprzMessage("datalink", "DCF_REG_TABLE")
            msga['ac_id'] = int(self.list_ids[i[0]])
            msga['nei_id'] = int(self.list_ids[i[1]])
            if len(self.list_ids) == 2:
                msga['desired_sigma'] = (column[index])[0]*int(self.Zdesired)
            else:
                msga['desired_sigma'] = (column[index])[0]*int(self.Zdesired[count])
            self._interface.send(msga)

            msgb = PprzMessage("datalink", "DCF_REG_TABLE")
            msgb['ac_id'] = int(self.list_ids[i[1]])
            msgb['nei_id'] = int(self.list_ids[i[0]])
            if len(self.list_ids) == 2:
                msgb['desired_sigma'] = (column[index])[1]*int(self.Zdesired)
            else:
                msgb['desired_sigma'] = (column[index])[1]*int(self.Zdesired[count])
            self._interface.send(msgb)

            if self.verbose:
                print(msga)
                print(msgb)

        time.sleep(2)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Init tables for DCF module")
    parser.add_argument('config_file', help="JSON configuration file")
    parser.add_argument('-v', '--verbose', dest='verbose', default=False, action='store_true', help="display debug messages")
    args = parser.parse_args()

    with open(args.config_file, 'r') as f:
        conf = json.load(f)
    if args.verbose:
        print(json.dumps(conf))
    try:
        initTableDCF = initTable(conf, verbose=args.verbose)
        initTableDCF.init_dcftables()
    except KeyboardInterrupt:
        initTableDCF.stop()
    initTableDCF.stop()
