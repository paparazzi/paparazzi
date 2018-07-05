#!/usr/bin/env python
#
# Copyright (C) 2018 TUDelft
#
# This file is part of paparazzi.
#
# paparazzi is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi.  If not, see <http://www.gnu.org/licenses/>.
#

import os
import numpy as np

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))


batmodel = np.loadtxt(PPRZ_SRC + '/sw/ground_segment/python/energy_mon/batterymodel.csv', delimiter=',')


capacity = 3300 #mAh
cells_in_series = 6 #cells
cells_in_parallel = 6 #cells



def index_from_volt(volt):
    v = round(volt*100.0,0)
    # 1/100 of volts
    if v>420:
        v=420
    elif v<251:
        v=251

    # index in a list from 420:-1:250
    index = 420 - v
    return index

def mah_from_volt_and_current(volt, current):
    global batmodel
    item = index_from_volt(volt)
    mah = batmodel[item,1:]

    # interpolate between point 1 and 2
    m0 = mah[1]
    a0 = 4.0
    dm = mah[1]-mah[0]
    da = 2.0

    return (current - a0) / da * dm + m0


#print(mah_from_volt_and_current(3.6, 2.5))

#for i in batmodel[:,[0,3]]:
#    print(i)

