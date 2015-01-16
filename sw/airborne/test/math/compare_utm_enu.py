#!/usr/bin/env python

from __future__ import division, print_function, absolute_import
import sys
import os

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", "../../../..")
sys.path.append(PPRZ_SRC + "/sw/lib/python")

from pprz_math.geodetic import *
from pprz_math.algebra import DoubleRMat, DoubleEulers, DoubleVect3
from math import radians, degrees, tan
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

# convergence angle to "true north" is approx 1 deg here
earth_radius = 6378137.0
n = 0.9996 * earth_radius
UTM_DELTA_EAST = 500000.
dist_to_meridian = utm_origin.east - UTM_DELTA_EAST
conv = dist_to_meridian / n * tan(lla_origin.lat)
# or (middle meridian of UTM zone 31 is at 3deg)
#conv = atan(tan(lla_origin.lon - radians(3))*sin(lla_origin.lat))
print("approx. convergence angle (north error compared to meridian): %f deg" % degrees(conv))
# Rotation matrix to correct for "true north"
R = DoubleEulers(psi=-conv).to_rmat()

# calculate ENU coordinates for 100 points in 100m distance
nb_points = 100
dist_points = 100
enu_res = np.zeros((nb_points, 2))
enu_res_c = np.zeros((nb_points, 2))
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
    enu_c = R * DoubleVect3(enu.x, enu.y, enu.z)
    enu_res_c[i, 0] = enu_c.x
    enu_res_c[i, 1] = enu_c.y
    #print(enu)


dist = np.linalg.norm(utm_res, axis=1)
error = np.linalg.norm(utm_res - enu_res, axis=1)
error_c = np.linalg.norm(utm_res - enu_res_c, axis=1)

plt.figure(1)
plt.subplot(311)
plt.title("utm vs. enu")
plt.plot(enu_res[:, 0], enu_res[:, 1], 'g', label="ENU")
plt.plot(utm_res[:, 0], utm_res[:, 1], 'r', label="UTM")
plt.ylabel("y/north [m]")
plt.xlabel("x/east [m]")
plt.legend(loc='upper left')
plt.subplot(312)
plt.plot(dist, error, 'r')
plt.xlabel("dist from origin [m]")
plt.ylabel("error [m]")
plt.subplot(313)
plt.plot(dist, error_c, 'r')
plt.xlabel("dist from origin [m]")
plt.ylabel("error with north fix [m]")

plt.show()
