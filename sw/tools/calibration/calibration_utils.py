
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
from pylab import *


#
# returns available ac_id from a log
#
def get_ids_in_log(filename):
    f = open(filename, 'r')
    ids = []
    pattern = re.compile("\S+ (\S+)")
    while 1:
        line = f.readline().strip()
        if line == '':
            break
        m=re.match(pattern, line)
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
    while 1:
        line = f.readline().strip()
        if line == '':
            break
        m=re.match(pattern, line)
        if m:
            list_meas.append([float(m.group(2)), float(m.group(3)), float(m.group(4))])
    return scipy.array(list_meas)



#
# select only non-noisy data
#
def filter_meas(meas, window_size, noise_threshold):
    filtered_meas = []
    filtered_idx = []
    for i in range(window_size,len(meas)-window_size):
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
    l_comp = [];
    l_norm = [];
    for m in meas[:,]:
        sm = (m - p[0:3])*p[3:6]
        l_comp.append(sm)
        l_norm.append(linalg.norm(sm))
    return scipy.array(l_comp), scipy.array(l_norm)


#
# print xml for airframe file
#
def print_xml(p, sensor, res):
    print ""
    print "<define name=\""+sensor+"_X_NEUTRAL\" value=\""+str(int(round(p[0])))+"\"/>"
    print "<define name=\""+sensor+"_Y_NEUTRAL\" value=\""+str(int(round(p[1])))+"\"/>"
    print "<define name=\""+sensor+"_Z_NEUTRAL\" value=\""+str(int(round(p[2])))+"\"/>"
    print "<define name=\""+sensor+"_X_SENS\" value=\""+str(p[3]*2**res)+"\" integer=\"16\"/>"
    print "<define name=\""+sensor+"_Y_SENS\" value=\""+str(p[4]*2**res)+"\" integer=\"16\"/>"
    print "<define name=\""+sensor+"_Z_SENS\" value=\""+str(p[5]*2**res)+"\" integer=\"16\"/>"



#
# plot calibration results
#
def plot_results(measurements, flt_idx, flt_meas, cp0, np0, cp1, np1, sensor_ref):
    subplot(3,1,1)
    plot(measurements[:,0])
    plot(measurements[:,1])
    plot(measurements[:,2])
    plot(flt_idx, flt_meas[:,0], 'ro')
    plot(flt_idx, flt_meas[:,1], 'ro')
    plot(flt_idx, flt_meas[:,2], 'ro')
    xlabel('time (s)')
    ylabel('ADC')
    title('Raw sensors')
  
    subplot(3,2,3)
    plot(cp0[:,0]);
    plot(cp0[:,1]);
    plot(cp0[:,2]);
    plot(-sensor_ref*scipy.ones(len(flt_meas)));
    plot(sensor_ref*scipy.ones(len(flt_meas)));

    subplot(3,2,4)
    plot(np0);
    plot(sensor_ref*scipy.ones(len(flt_meas)));

    subplot(3,2,5)
    plot(cp1[:,0]);
    plot(cp1[:,1]);
    plot(cp1[:,2]);
    plot(-sensor_ref*scipy.ones(len(flt_meas)));
    plot(sensor_ref*scipy.ones(len(flt_meas)));

    subplot(3,2,6)
    plot(np1);
    plot(sensor_ref*scipy.ones(len(flt_meas)));

    show();


#
# read a turntable log
# return an array which first column is turnatble and next 3 are gyro
#
def read_turntable_log(ac_id, tt_id, filename, _min, _max):
    f = open(filename, 'r')
    pattern_g = re.compile("(\S+) "+ac_id+" IMU_GYRO_RAW (\S+) (\S+) (\S+)")
    pattern_t = re.compile("(\S+) "+tt_id+" IMU_TURNTABLE (\S+)")
    last_tt = None
    list_tt = []
    while 1:
        line = f.readline().strip()
        if line == '':
            break
        m=re.match(pattern_t, line)
        if m:
            last_tt = float(m.group(2))
        m=re.match(pattern_g, line)
        if m and last_tt and last_tt > _min and last_tt < _max:
            list_tt.append([last_tt, float(m.group(2)), float(m.group(3)), float(m.group(4))])
    return scipy.array(list_tt)           

#
#
#
