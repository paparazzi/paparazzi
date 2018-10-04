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
from scipy import interpolate

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))


batmodel = np.loadtxt(PPRZ_SRC + '/sw/ground_segment/python/energy_mon/batterymodel.csv', delimiter=',')
batmodel_reversed = batmodel[::-1, :]
f_batmodel = interpolate.RectBivariateSpline(batmodel_reversed[:, 0], range(2, 11, 2), batmodel_reversed[:, 1:])

# batmodel_wh = scipy.integrate.cumtrapz(batmodel[:, 1:], )
# batmodel_power = batmodel.copy()
# batmodel_power[1:] *= np.array([2, 4, 6, 8, 10])

# Indices:
# 0: V
# 1: mAh 2A
# 2: mAh 4A
# 2: mAh 6A
# 2: mAh 8A
# 2: mAh 10A

capacity = 3300 #mAh
cells_in_series = 6 #cells
cells_in_parallel = 6 #cells
cells_in_battery = cells_in_parallel * cells_in_series

def interpolate_monotonic_x(x, y, x_test):
    """Interpolate a function defined on a monotonically increasing grid
    :param x -> x-data, must be monotonically increasing/decreasing (this is not checked
    :param y -> corresponding y-data
    :param x_test -> the x value for which you want the interpolated y
    """
    n = len(x)
    dx = x[1] - x[0]
    x0 = [0]

    i_float = (x_test - x0) / dx

    assert i_float >= 0 and i_float <= len(y)

    i_lower = int(i_float)

    return y[i_lower] + (y[i_lower + 1] - y[i_lower]) / (i_float - i_lower)


def index_from_volt(volt):
    v = round(volt*100.0,0)
    # 1/100 of volts
    if v>420:
        v=420
    elif v<251:
        v=251

    # index in a list from 420:-1:250
    index = 420 - v
    return int(index)

def mah_from_volt_and_current(volt, current):
    global f_batmodel
    return f_batmodel(volt, current)  # (current - a0) / da * dm + m0

def volt_amp_from_mAh_power(mAh, power):
    Vs = np.zeros(5)
    powers = np.zeros(5)
    for i in range(5):
        Vs[i] = np.interp(mAh, batmodel[:, i+1], batmodel[:, 0])
        I = i * 2 + 2
        powers[i] = Vs[i] * I

    volt = np.interp(power, powers, Vs)
    amp = np.interp(power, powers, range(2, 11, 2))
    return volt, amp


def time_mAh_from_volt_to_volt_power(v0, v1, power):
    mAh_accumulated = 0
    t_accumulated = 0
    mAh_prev = None

    for v in np.arange(v0, v1, -0.01):
        I = power / v  # current in A
        mAh = mah_from_volt_and_current(v, I)
        dmAh = mAh - mAh_prev if not mAh_prev is None else 0
        mAh_prev = mAh
        dt = (dmAh / 1000) / I * 3600

        mAh_accumulated += dmAh
        t_accumulated += dt

    return t_accumulated, mAh_accumulated

#print(mah_from_volt_and_current(3.6, 2.5))

#for i in batmodel[:,[0,3]]:
#    print(i)


if __name__ == '__main__':
    v, i = volt_amp_from_mAh_power(2500, 550/cells_in_battery)
    print(v, i, v*i*cells_in_battery)
    print(time_mAh_from_volt_to_volt_power(3.3, 3.0, 550/cells_in_battery))
