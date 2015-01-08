#!/usr/bin/env python

from __future__ import division, print_function, absolute_import
import sys
import os

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", "../../../..")
sys.path.append(PPRZ_SRC + "/sw/lib/python/math")

from pprz_geodetic import *
from math import degrees
import matplotlib.pyplot as plt
import numpy as np

# Origin at ENAC
UTM_EAST0 = 377349  # in m
UTM_NORTH0 = 4824583  # in m
UTM_ZONE0 = 31
ALT0 = 147.000  # in m

utm_origin = UtmCoor_d(north=UTM_NORTH0, east=UTM_EAST0, alt=ALT0, zone=UTM_ZONE0)
print("origin %s" % utm_origin)

lla_origin = LlaCoor_d()
lla_of_utm_d(lla_origin, utm_origin)
ecef_origin = EcefCoor_d()
ecef_of_lla_d(ecef_origin, lla_origin)
ltp_origin = LtpDef_d()
ltp_def_from_ecef_d(ltp_origin, ecef_origin)
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
    lla = LlaCoor_d()
    lla_of_utm_d(lla, utm)
    #print(lla)
    ecef = EcefCoor_d()
    ecef_of_lla_d(ecef, lla)
    enu = EnuCoor_d()
    enu_of_ecef_point_d(enu, ltp_origin, ecef)
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
