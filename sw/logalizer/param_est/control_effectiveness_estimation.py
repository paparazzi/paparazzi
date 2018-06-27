#!/usr/bin/env python3

# Copyright (C) 2018 Ewoud Smeur

import scipy as sp
from scipy import signal
import csv
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure, show

# Do a linear least squares fit of the data
def fit_axis(x,y, axis, start, end):
    c = np.linalg.lstsq(x[start:end], y[start:end])[0]
    print('Fit of axis ' + axis + ': ' + str(c*1000))
    plt.figure()
    plt.plot(t,y)
    plt.plot(t,np.dot(x,c))
    plt.ylabel(axis + ' dotdot[rad/s^3]')
    plt.xlabel('t [s]')
    return c

# Effort to make a function that accurately models the actuator, but this is still not sufficient for the bebop2
def actuator_dyn_2nd(x, zeta, omega, dt, maxrate, maxaccel):
    y = np.zeros(x.shape)
    y[0] = x[0]
    yd = 0
    ydd = 0
    for i in range(x.shape[0]-1):
        ydd = -2*zeta*omega*yd + (x[i]-y[i])*omega*omega
        if(abs(ydd) > maxaccel):
            ydd = maxaccel*np.sign(ydd)
        yd = yd + ydd*dt
        if(abs(yd) > maxrate):
            yd = maxrate*np.sign(yd)
        y[i+1] = y[i] + yd*dt
    return y


# Read data from log file
data = genfromtxt('00008.csv', delimiter=',', skip_header=1)

# Sample frequency
sf = 512;
#First order actuator dynamics constant (discrete, depending on sf)
fo_c = 0.08

N = data.shape[0]

# Data structure
t = np.arange(N)/sf
gyro = data[:,1:4]
accel = data[:,4:7]/pow(2,10)
cmd = data[:,7:11]
# If actuator feedback is available:
obs = data[:,11:15]
# If testing the actuator response:
servotest = data[:,15]

# Actuator dynamics
cmd_a = sp.signal.lfilter([fo_c], [1, -(1-fo_c)], cmd, axis=0)
servotest_a = sp.signal.lfilter([fo_c], [1, -(1-fo_c)], servotest, axis=0)

# b, a = signal.butter(2, 7/(sf/2), 'low', analog=False)
# servotest_a = sp.signal.lfilter(b, a, servotest, axis=0)
# servotest_a = actuator_dyn_2nd(servotest, 0.8, 70, 1/sf, 60000, 10000000)

# Filtering
b, a = signal.butter(2, 3.2/(sf/2), 'low', analog=False)
gyro_f = sp.signal.lfilter(b, a, gyro, axis=0)
cmd_af = sp.signal.lfilter(b, a, cmd_a, axis=0)
accel_f = sp.signal.lfilter(b, a, accel, axis=0)

# derivatives
dgyro_f = np.vstack((np.zeros((1,3)), np.diff(gyro_f,1,axis=0)))*sf
ddgyro_f = np.vstack((np.zeros((1,3)), np.diff(dgyro_f,1,axis=0)))*sf
daccel_f = np.vstack((np.zeros((1,3)), np.diff(accel_f,1,axis=0)))*sf
dcmd_af = np.vstack((np.zeros((1,4)), np.diff(cmd_af,1,axis=0)))*sf
ddcmd_af = np.vstack((np.zeros((1,4)), np.diff(dcmd_af,1,axis=0)))*sf

# Selective data (for instance remove the takeoff and landing)
start = 20*sf
end = 40*sf
# Estimation of the control effectiveness
c = fit_axis(dcmd_af[:,0:4], ddgyro_f[:,[0]], 'p', 0, N)
c = fit_axis(dcmd_af[:,0:4], ddgyro_f[:,[1]], 'q', 0, N)
c = fit_axis(dcmd_af[:,0:4], daccel_f[:,[2]], 'accel', start, end)
# Use all commands, to see if there is crosstalk
c = fit_axis(np.hstack((dcmd_af[:,0:4], ddcmd_af[:,0:4]/sf)), ddgyro_f[:,[2]], 'r', 0, end)

plt.figure()
plt.plot(t,cmd)
plt.xlabel('t [s]')
plt.ylabel('command [PPRZ]')

# Show all plots
plt.show()
