
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

from __future__ import print_function, division

import re
from telnetlib import theNULL
import numpy as np
from numpy import sin, cos
from scipy import linalg, stats
from fractions import Fraction

import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def get_ids_in_log(filename):
    """Returns available ac_id from a log."""
    f = open(filename, 'r')
    ids = []
    pattern = re.compile("\S+ (\S+)")
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
            ac_id = m.group(1)
            if not ac_id in ids:
                ids.append(ac_id)
    return ids

def get_sensor_ids(ac_id, filename, sensor):
    f = open(filename, 'r')
    ids = []
    if sensor == 'MAG':
        pattern = re.compile("\S+ "+ac_id+" IMU_"+sensor+"_RAW (\S+) \S+ \S+ \S+")
    else:
        pattern = re.compile("\S+ "+ac_id+" IMU_"+sensor+"_RAW (\S+) \S+ \S+ \S+ \S+")
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
            sensor_id = m.group(1)
            if not sensor_id in ids:
                ids.append(sensor_id)
    return ids

def read_log(ac_id, filename, sensor, sensor_id):
    """Extracts raw sensor measurements from a log."""
    f = open(filename, 'r')
    if sensor == 'MAG':
        pattern = re.compile("(\S+) "+ac_id+" IMU_"+sensor+"_RAW "+sensor_id+" (\S+) (\S+) (\S+)")
    else:
        pattern = re.compile("(\S+) "+ac_id+" IMU_"+sensor+"_RAW "+sensor_id+" \S+ (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
            list_meas.append([float(m.group(2)), float(m.group(3)), float(m.group(4))])
    return np.array(list_meas)


def read_log_scaled(ac_id, filename, sensor, sensor_id, t_start, t_end):
    """Extracts scaled sensor measurements from a log."""
    f = open(filename, 'r')
    pattern = re.compile("(\S+) "+ac_id+" IMU_"+sensor+"_SCALED "+sensor_id+" (\S+) (\S+) (\S+)")
    list_meas = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m = re.match(pattern, line)
        if m:
            if (float(m.group(1)) >= float(t_start)) and (float(m.group(1)) < (float(t_end)+1.0)):
                list_meas.append([float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))])
    return np.array(list_meas)


def read_log_mag_current(ac_id, filename):
    """Extracts raw magnetometer and current measurements from a log."""
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
    return np.array(list_meas)


def filter_meas(meas, window_size, noise_threshold):
    """Select only non-noisy data."""
    filtered_meas = []
    filtered_idx = []
    for i in range(window_size, len(meas)-window_size):
        noise = meas[i-window_size:i+window_size, :].std(axis=0)
        if linalg.norm(noise) < noise_threshold:
            filtered_meas.append(meas[i, :])
            filtered_idx.append(i)
    return np.array(filtered_meas), filtered_idx


def get_min_max_guess(meas, scale):
    """Initial boundary based calibration."""
    max_meas = meas[:, :].max(axis=0)
    min_meas = meas[:, :].min(axis=0)
    range = max_meas - min_meas
    # check if we would get division by zero
    if range.all():
        n = (max_meas + min_meas) / 2
        sf = 2*scale/range
        return np.array([n[0], n[1], n[2], sf[0], sf[1], sf[2]])
    else:
        return np.array([0, 0, 0, 0])


def scale_measurements(meas, p):
    """Scale the set of measurements."""
    l_comp = []
    l_norm = []
    for m in meas[:, ]:
        sm = (m - p[0:3])*p[3:6]
        l_comp.append(sm)
        l_norm.append(linalg.norm(sm))
    return np.array(l_comp), np.array(l_norm)


def estimate_mag_current_relation(meas):
    """Calculate linear coefficient of magnetometer-current relation."""
    coefficient = []
    offset = []
    for i in range(0, 3):
        gradient, intercept, r_value, p_value, std_err = stats.linregress(meas[:, 3], meas[:, i])
        coefficient.append(gradient)
        offset.append(intercept)
    return coefficient, offset

def continious_frac(v):
    max_val = 2**16
    if v > 0:
        s = 1
    else:
        v = -v
        s = -1
    return _continious_frac(v, max_val, int(v), v, (1, int(v)), (0,1), s)

def _continious_frac(v, max_val, a, x, num, den, s):
    x1 = 1 / (x - a)
    a1 = int(x1)
    (num1, num2) = num
    num3 = a1 * num2 + num1
    (den1, den2) = den
    den3 = a1 * den2 + den1
    if num3 > max_val or den3 > max_val:
        return (num2, s*den2)
    elif (num3 / den3) == v:
        return (num3, s*den3)
    else:
        return _continious_frac(v, max_val, a1, x1, (num2, num3), (den2, den3), s)

def print_xml(p, sensor, sensor_id, res):
    """Print xml for airframe file."""
    x_sens = continious_frac(p[3]*2**res)
    y_sens = continious_frac(p[4]*2**res)
    z_sens = continious_frac(p[5]*2**res)

    struct = "{{.abi_id="+sensor_id+", .calibrated={.neutral=true, .scale=true},"
    struct += ".neutral={"+str(int(round(p[0])))+","+str(int(round(p[1])))+","+str(int(round(p[2])))+"}, "
    struct += ".scale={{"+str(x_sens[0])+","+str(y_sens[0])+","+str(z_sens[0])+"},"
    struct += "{"+str(x_sens[1])+","+str(y_sens[1])+","+str(z_sens[1])+"}}"
    struct += "}}"

    print("")
    print("<define name=\"IMU_"+sensor+"_CALIB\" value=\""+struct+"\"/>")
    print("")
    print("<define name=\""+sensor+"_X_NEUTRAL\" value=\""+str(int(round(p[0])))+"\"/>")
    print("<define name=\""+sensor+"_Y_NEUTRAL\" value=\""+str(int(round(p[1])))+"\"/>")
    print("<define name=\""+sensor+"_Z_NEUTRAL\" value=\""+str(int(round(p[2])))+"\"/>")
    print("<define name=\""+sensor+"_X_SENS\" value=\""+str(p[3]*2**res)+"\" integer=\"16\"/>")
    print("<define name=\""+sensor+"_Y_SENS\" value=\""+str(p[4]*2**res)+"\" integer=\"16\"/>")
    print("<define name=\""+sensor+"_Z_SENS\" value=\""+str(p[5]*2**res)+"\" integer=\"16\"/>")


def print_imu_scaled(sensor, measurements, attrs):
    print("")
    print(sensor+" : Time Range("+str(measurements[:,0].min(axis=0))+" : "+str(measurements[:,0].max(axis=0))+")")
    np.set_printoptions(formatter={'float': '{:-7.3f}'.format})
    print("         " + attrs[2] + "      " + attrs[3] + "      " + attrs[4])
    print("Min   " + str(measurements[:,1:].min(axis=0)*attrs[0])  + " " + attrs[1])
    print("Max   " + str(measurements[:,1:].max(axis=0)*attrs[0])  + " " + attrs[1])
    print("Mean  " + str(measurements[:,1:].mean(axis=0)*attrs[0]) + " " + attrs[1])
    print("StDev " + str(measurements[:,1:].std(axis=0)*attrs[0])  + " " + attrs[1])


def plot_measurements(sensor, measurements):
    plt.plot(measurements[:, 0])
    plt.plot(measurements[:, 1])
    plt.plot(measurements[:, 2])
    plt.ylabel('ADC')
    plt.title("Raw %s measurements" % sensor)
    plt.show()

def plot_results(sensor, measurements, flt_idx, flt_meas, cp0, np0, cp1, np1, sensor_ref, blocking=True):
    """Plot calibration results."""
    # plot raw measurements with filtered ones marked as red circles
    plt.subplot(3, 1, 1)
    plt.plot(measurements[:, 0])
    plt.plot(measurements[:, 1])
    plt.plot(measurements[:, 2])
    plt.plot(flt_idx, flt_meas[:, 0], 'ro')
    plt.plot(flt_idx, flt_meas[:, 1], 'ro')
    plt.plot(flt_idx, flt_meas[:, 2], 'ro')
    plt.ylabel('ADC')
    plt.title('Raw '+sensor+', red dots are actually used measurements')

    plt.tight_layout()
    # show scaled measurements with initial guess
    plt.subplot(3, 2, 3)
    plt.plot(cp0[:, 0])
    plt.plot(cp0[:, 1])
    plt.plot(cp0[:, 2])
    plt.plot(-sensor_ref*np.ones(len(flt_meas)))
    plt.plot(sensor_ref*np.ones(len(flt_meas)))
    plt.title('scaled '+sensor+' (initial guess)')
    plt.xticks([])

    plt.subplot(3, 2, 4)
    plt.plot(np0)
    plt.plot(sensor_ref*np.ones(len(flt_meas)))
    plt.title('norm of '+sensor+' (initial guess)')
    plt.xticks([])

    # show scaled measurements after optimization
    plt.subplot(3, 2, 5)
    plt.plot(cp1[:, 0])
    plt.plot(cp1[:, 1])
    plt.plot(cp1[:, 2])
    plt.plot(-sensor_ref*np.ones(len(flt_meas)))
    plt.plot(sensor_ref*np.ones(len(flt_meas)))
    plt.title('scaled '+sensor+' (optimized)')
    plt.xticks([])

    plt.subplot(3, 2, 6)
    plt.plot(np1)
    plt.plot(sensor_ref*np.ones(len(flt_meas)))
    plt.title('norm of '+sensor+' (optimized)')
    plt.xticks([])

    # if we want to have another plot we only draw the figure (non-blocking)
    # also in matplotlib before 1.0.0 there is only one call to show possible
    if blocking:
        plt.show()
    else:
        plt.draw()


def plot_imu_scaled(sensor, measurements, attrs):
    """Plot imu scaled results."""
    plt.figure("Sensor Scaled")

    plt.subplot(4, 1, 1)
    plt.plot(measurements[:, 0], measurements[:, 1]*attrs[0])
    plt.plot(measurements[:, 0], measurements[:, 2]*attrs[0])
    plt.plot(measurements[:, 0], measurements[:, 3]*attrs[0])
    #plt.xlabel('Time (s)')
    plt.ylabel(attrs[1])
    plt.title(sensor)

    plt.subplot(4, 1, 2)
    plt.plot(measurements[:, 0], measurements[:, 1]*attrs[0], 'b')
    #plt.xlabel('Time (s)')
    plt.ylabel(attrs[2])

    plt.subplot(4, 1, 3)
    plt.plot(measurements[:, 0], measurements[:, 2]*attrs[0], 'g')
    #plt.xlabel('Time (s)')
    plt.ylabel(attrs[3])

    plt.subplot(4, 1, 4)
    plt.plot(measurements[:, 0], measurements[:, 3]*attrs[0], 'r')
    plt.xlabel('Time (s)')
    plt.ylabel(attrs[4])

    plt.show()


def plot_imu_scaled_fft(sensor, measurements, attrs):
    """Plot imu scaled fft results."""
    #dt = 0.0769
    #Fs = 1/dt
    Fs = 26.0

    plt.figure("Sensor Scaled - FFT")

    plt.subplot(3, 1, 1)
    plt.magnitude_spectrum(measurements[:, 1]*attrs[0], Fs=Fs, scale='linear')
    plt.ylabel(attrs[2])
    plt.title(sensor)

    plt.subplot(3, 1, 2)
    plt.magnitude_spectrum(measurements[:, 2]*attrs[0], Fs=Fs, scale='linear')
    plt.ylabel(attrs[3])

    plt.subplot(3, 1, 3)
    plt.magnitude_spectrum(measurements[:, 3]*attrs[0], Fs=Fs, scale='linear')
    plt.xlabel('Frequency')
    plt.ylabel(attrs[4])

    plt.show()



def plot_mag_3d(measured, calibrated, p):
    """Plot magnetometer measurements on 3D sphere."""
    # set up points for sphere and ellipsoid wireframes
    u = np.r_[0:2 * np.pi:20j]
    v = np.r_[0:np.pi:20j]
    wx = np.outer(cos(u), sin(v))
    wy = np.outer(sin(u), sin(v))
    wz = np.outer(np.ones(np.size(u)), cos(v))
    ex = p[0] * np.ones(np.size(u)) + np.outer(cos(u), sin(v)) / p[3]
    ey = p[1] * np.ones(np.size(u)) + np.outer(sin(u), sin(v)) / p[4]
    ez = p[2] * np.ones(np.size(u)) + np.outer(np.ones(np.size(u)), cos(v)) / p[5]

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

    fig = plt.figure(figsize=plt.figaspect(0.5))
    if matplotlib.__version__.startswith('0'):
        ax = Axes3D(fig, rect=rect_l)
    else:
        ax = fig.add_subplot(1, 2, 1, position=rect_l, projection='3d')
    # plot measurements
    ax.scatter(mx, my, mz)
    # plot line from center to ellipsoid center
    ax.plot([0.0, p[0]], [0.0, p[1]], [0.0, p[2]], color='black', marker='+', markersize=10)
    # plot ellipsoid
    ax.plot_wireframe(ex, ey, ez, color='grey', alpha=0.5)

    # Create cubic bounding box to simulate equal aspect ratio
    max_range = np.array([mx.max() - mx.min(), my.max() - my.min(), mz.max() - mz.min()]).max()
    Xb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][0].flatten() + 0.5 * (mx.max() + mx.min())
    Yb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][1].flatten() + 0.5 * (my.max() + my.min())
    Zb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][2].flatten() + 0.5 * (mz.max() + mz.min())
    # add the fake bounding box:
    for xb, yb, zb in zip(Xb, Yb, Zb):
        ax.plot([xb], [yb], [zb], 'w')

    ax.set_title('MAG raw with fitted ellipsoid and center offset')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    if matplotlib.__version__.startswith('0'):
        ax = Axes3D(fig, rect=rect_r)
    else:
        ax = fig.add_subplot(1, 2, 2, position=rect_r, projection='3d')
    ax.plot_wireframe(wx, wy, wz, color='grey', alpha=0.5)
    ax.scatter(cx, cy, cz)

    ax.set_title('MAG calibrated on unit sphere')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlim3d(-1, 1)
    ax.set_ylim3d(-1, 1)
    ax.set_zlim3d(-1, 1)
    plt.show()


def read_turntable_log(ac_id, tt_id, filename, _min, _max):
    """ Read a turntable log.
    return an array which first column is turnatble and next 3 are gyro
    """
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
        if m and last_tt and _min < last_tt < _max:
            list_tt.append([last_tt, float(m.group(2)), float(m.group(3)), float(m.group(4))])
    return np.array(list_tt)


def plot_mag_current_fit(measurements, coefficient, offset):
    plt.subplot(2, 1, 1)
    plt.plot(measurements[:, 0])
    plt.plot(measurements[:, 1])
    plt.plot(measurements[:, 2])
    plt.plot(measurements[:, 3]*coefficient[0]+offset[0])
    plt.plot(measurements[:, 3]*coefficient[1]+offset[1])
    plt.plot(measurements[:, 3]*coefficient[2]+offset[2])
    plt.ylabel('ADC')
    plt.title("Raw measurements")
    plt.subplot(2, 1, 2)
    plt.plot(measurements[:, 3])
    plt.show()
