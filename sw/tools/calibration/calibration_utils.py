
#  $Id$
#  Copyright (C) 2010 Antoine Drouin
#
# This file is part of Paparazzi.
#
# Paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# Paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#

import re
import scipy
from scipy import linalg
from scipy import stats
from pylab import *
from mpl_toolkits.mplot3d import Axes3D

#
# returns available ac_id from a log
#
def get_ids_in_log(filename):
    f = open(filename, 'r')
    ids = []
    pattern = re.compile("\S+ (\S+)")
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
            id = m.group(1)
            if not id in ids:
              ids.append(id)
    return ids

#
# extracts raw sensor measurements from a log
#
def read_log(ac_id, filename, sensor):
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" IMU_"+sensor+"_RAW (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
            list_meas.append([float(m.group(2)), float(m.group(3)), float(m.group(4))])
    return scipy.array(list_meas)

#
# extracts raw magnetometer and current measurements from a log
#
def read_log_mag_current(ac_id, filename):
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" IMU_MAG_CURRENT_CALIBRATION (\S+) (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
            list_meas.append([float(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5))])
    return scipy.array(list_meas)

#
# select only non-noisy data
#
def filter_meas(meas, window_size, noise_threshold):
    filtered_meas = []
    filtered_idx = []
    for i in range(window_size, len(meas)-window_size):
        noise = meas[i-window_size:i+window_size,:].std(axis=0)
        if  linalg.norm(noise) < noise_threshold:
            filtered_meas.append(meas[i,:])
            filtered_idx.append(i)
    return scipy.array(filtered_meas), filtered_idx


#
# initial boundary based calibration
#
def get_min_max_guess(meas, scale):
    max_meas = meas[:,:].max(axis=0)
    min_meas = meas[:,:].min(axis=0)
    n = (max_meas + min_meas) / 2
    sf = 2*scale/(max_meas - min_meas)
    return scipy.array([n[0], n[1], n[2], sf[0], sf[1], sf[2]])


#
# scale the set of measurements
#
def scale_measurements(meas, p):
    l_comp = []
    l_norm = []
    for m in meas[:,]:
        sm = (m - p[0:3])*p[3:6]
        l_comp.append(sm)
        l_norm.append(linalg.norm(sm))
    return scipy.array(l_comp), scipy.array(l_norm)

#
# calculate linear coefficient of magnetometer-current relation
#
def estimate_mag_current_relation(meas):
    coefficient = []
    for i in range(0,3):
        gradient, intercept, r_value, p_value, std_err = stats.linregress(meas[:,3], meas[:,i])
        coefficient.append(gradient)
    return coefficient

#
# print xml for airframe file
#
def print_xml(p, sensor, res):
    print("")
    print("<define name=\""+sensor+"_X_NEUTRAL\" value=\""+str(int(round(p[0])))+"\"/>")
    print("<define name=\""+sensor+"_Y_NEUTRAL\" value=\""+str(int(round(p[1])))+"\"/>")
    print("<define name=\""+sensor+"_Z_NEUTRAL\" value=\""+str(int(round(p[2])))+"\"/>")
    print("<define name=\""+sensor+"_X_SENS\" value=\""+str(p[3]*2**res)+"\" integer=\"16\"/>")
    print("<define name=\""+sensor+"_Y_SENS\" value=\""+str(p[4]*2**res)+"\" integer=\"16\"/>")
    print("<define name=\""+sensor+"_Z_SENS\" value=\""+str(p[5]*2**res)+"\" integer=\"16\"/>")



#
# plot calibration results
#
def plot_results(block, measurements, flt_idx, flt_meas, cp0, np0, cp1, np1, sensor_ref):
    subplot(3, 1, 1)
    plot(measurements[:, 0])
    plot(measurements[:, 1])
    plot(measurements[:, 2])
    plot(flt_idx, flt_meas[:, 0], 'ro')
    plot(flt_idx, flt_meas[:, 1], 'ro')
    plot(flt_idx, flt_meas[:, 2], 'ro')
    xlabel('time (s)')
    ylabel('ADC')
    title('Raw sensors')

    subplot(3, 2, 3)
    plot(cp0[:, 0])
    plot(cp0[:, 1])
    plot(cp0[:, 2])
    plot(-sensor_ref*scipy.ones(len(flt_meas)))
    plot(sensor_ref*scipy.ones(len(flt_meas)))

    subplot(3, 2, 4)
    plot(np0)
    plot(sensor_ref*scipy.ones(len(flt_meas)))

    subplot(3, 2, 5)
    plot(cp1[:, 0])
    plot(cp1[:, 1])
    plot(cp1[:, 2])
    plot(-sensor_ref*scipy.ones(len(flt_meas)))
    plot(sensor_ref*scipy.ones(len(flt_meas)))

    subplot(3, 2, 6)
    plot(np1)
    plot(sensor_ref*scipy.ones(len(flt_meas)))

    # if we want to have another plot we only draw the figure (non-blocking)
    # also in matplotlib before 1.0.0 there is only one call to show possible
    if block:
        show()
    else:
        draw()

#
# plot mag measurements in 3D
#
def plot_mag_3d(measured, calibrated, p):
    # set up points for sphere and ellipsoid wireframes
    u = r_[0:2 * pi:20j]
    v = r_[0:pi:20j]
    wx = outer(cos(u), sin(v))
    wy = outer(sin(u), sin(v))
    wz = outer(ones(size(u)), cos(v))
    ex = p[0] * ones(size(u)) + outer(cos(u), sin(v)) / p[3]
    ey = p[1] * ones(size(u)) + outer(sin(u), sin(v)) / p[4]
    ez = p[2] * ones(size(u)) + outer(ones(size(u)), cos(v)) / p[5]

    # measurements
    mx = measured[:, 0]
    my = measured[:, 1]
    mz = measured[:, 2]

    # calibrated values
    cx = calibrated[:, 0]
    cy = calibrated[:, 1]
    cz = calibrated[:, 2]

    # axes size
    left = 0.02
    bottom = 0.05
    width = 0.46
    height = 0.9
    rect_l = [left, bottom, width, height]
    rect_r = [left/2+0.5, bottom, width, height]

    fig = figure(figsize=figaspect(0.5))
    if matplotlib.__version__.startswith('0'):
        ax = Axes3D(fig, rect=rect_l)
    else:
        ax = fig.add_subplot(1, 2, 1, position=rect_l, projection='3d')
    # plot measurements
    ax.scatter(mx, my, mz)
    hold(True)
    # plot line from center to ellipsoid center
    ax.plot([0.0, p[0]], [0.0, p[1]], [0.0, p[2]], color='black', marker='+', markersize=10)
    # plot ellipsoid
    ax.plot_wireframe(ex, ey, ez, color='grey', alpha=0.5)

    # Create cubic bounding box to simulate equal aspect ratio
    max_range = np.array([mx.max() - mx.min(), my.max() - my.min(), mz.max() - mz.min()]).max()
    Xb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][0].flatten() + 0.5 * (mx.max() + mx.min())
    Yb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][1].flatten() + 0.5 * (my.max() + my.min())
    Zb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][2].flatten() + 0.5 * (mz.max() + mz.min())
    # uncomment following both lines to test the fake bounding box:
    #for xb, yb, zb in zip(Xb, Yb, Zb):
    #    ax.plot([xb], [yb], [zb], 'r*')

    ax.set_title('MAG raw with fitted ellipsoid and center offset')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    if matplotlib.__version__.startswith('0'):
        ax = Axes3D(fig, rect=rect_r)
    else:
        ax = fig.add_subplot(1, 2, 2, position=rect_r, projection='3d')
    ax.plot_wireframe(wx, wy, wz, color='grey', alpha=0.5)
    hold(True)
    ax.scatter(cx, cy, cz)

    ax.set_title('MAG calibrated on unit sphere')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlim3d(-1, 1)
    ax.set_ylim3d(-1, 1)
    ax.set_zlim3d(-1, 1)
    show()

#
# read a turntable log
# return an array which first column is turnatble and next 3 are gyro
#
def read_turntable_log(ac_id, tt_id, filename, _min, _max):
    f = open(filename, 'r')
    pattern_g = re.compile("(\S+) "+str(ac_id)+" IMU_GYRO_RAW (\S+) (\S+) (\S+)")
    pattern_t = re.compile("(\S+) "+str(tt_id)+" IMU_TURNTABLE (\S+)")
    last_tt = None
    list_tt = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern_t, line)
        if m:
            last_tt = float(m.group(2))
        m = re.match(pattern_g, line)
        if m and last_tt and last_tt > _min and last_tt < _max:
            list_tt.append([last_tt, float(m.group(2)), float(m.group(3)), float(m.group(4))])
    return scipy.array(list_tt)

#
#
#
