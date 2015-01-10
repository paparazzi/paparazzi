#!/usr/bin/env python

from __future__ import division, print_function, absolute_import
import sys
import os

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", "../../../..")
sys.path.append(PPRZ_SRC + "/sw/lib/python")

from pprz_math.geodetic import *
from pprz_math.algebra import DoubleRMat
import matplotlib.pyplot as plt
import numpy as np

# Origin at ENAC
UTM_EAST0 = 377349  # in m
UTM_NORTH0 = 4824583  # in m
UTM_ZONE0 = 31
ALT0 = 147.000  # in m

utm_origin = UtmCoor_d(north=UTM_NORTH0, east=UTM_EAST0, alt=ALT0, zone=UTM_ZONE0)
print("origin %s" % utm_origin)

lla_origin = utm_origin.to_lla()
ecef_origin = lla_origin.to_ecef()
ltp_origin = ecef_origin.to_ltp_def()
print(ltp_origin)

# calculate ENU coordinates for 100 points in 100m distance
nb_points = 100
dist_points = 100
enu_res = np.zeros((nb_points, 2))
utm_res = np.zeros((nb_points, 2))
for i in range(0, nb_points):
    utm = UtmCoor_d()
    utm.north = i * dist_points + utm_origin.north
    utm.east = i * dist_points+ utm_origin.east
    utm.alt = utm_origin.alt
    utm.zone = utm_origin.zone
    #print(utm)
    utm_res[i, 0] = utm.east - utm_origin.east
    utm_res[i, 1] = utm.north - utm_origin.north
    lla = utm.to_lla()
    #print(lla)
    ecef = lla.to_ecef()
    enu = ecef.to_enu(ltp_origin)
    enu_res[i, 0] = enu.x
    enu_res[i, 1] = enu.y
    #print(enu)


dist = np.linalg.norm(utm_res, axis=1)
error = np.linalg.norm(utm_res - enu_res, axis=1)

plt.figure(1)
plt.subplot(211)
plt.title("utm vs. enu")
plt.plot(enu_res[:, 0], enu_res[:, 1], 'g', label="ENU")
plt.plot(utm_res[:, 0], utm_res[:, 1], 'r', label="UTM")
plt.ylabel("y/north [m]")
plt.xlabel("x/east [m]")
plt.legend(loc='upper left')
plt.subplot(212)
plt.plot(dist, error, 'r')
plt.xlabel("dist from origin [m]")
plt.ylabel("error [m]")

plt.show()
