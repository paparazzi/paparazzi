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

utm_origin = UtmCoor_d()
utm_origin.north = UTM_NORTH0
utm_origin.east = UTM_EAST0
utm_origin.zone = UTM_ZONE0
utm_origin.alt = ALT0
print("utm origin xyz [%.3f, %.3f, %.3f]" % (utm_origin.east, utm_origin.north, utm_origin.alt))

lla_origin = LlaCoor_d()
lla_of_utm_d(lla_origin, utm_origin)
print("lla origin lat/lon/alt [%.7f, %.7f, %.3f]" % (degrees(lla_origin.lat), degrees(lla_origin.lon), lla_origin.alt))
ecef_origin = EcefCoor_d()
ecef_of_lla_d(ecef_origin, lla_origin)
print("ecef origin xyz [%.3f, %.3f, %.3f]" % (ecef_origin.x, ecef_origin.y, ecef_origin.z))
ltp_origin = LtpDef_d()
ltp_def_from_ecef_d(ltp_origin, ecef_origin)

# calculate ENU coordinates for 100 points in 100m distance
nb_points = 100
dist_points = 100
enu_res = np.zeros((nb_points, 2))
utm_res = np.zeros((nb_points, 2))
for i in range(0, nb_points):
    utm = UtmCoor_d()
    utm.north = i * dist_points + UTM_NORTH0
    utm.east = i * dist_points + UTM_EAST0
    utm.alt = ALT0
    utm.zone = UTM_ZONE0
    #print("utm x=%.3f, y=%.3f" % (utm.east, utm.north))
    utm_res[i, 0] = utm.east - UTM_EAST0
    utm_res[i, 1] = utm.north - UTM_NORTH0
    lla = LlaCoor_d()
    lla_of_utm_d(lla, utm)
    #print("lla lat/lon/alt [%.7f, %.7f, %.3f]" % (degrees(lla.lat), degrees(lla.lon), lla.alt))
    ecef = EcefCoor_d()
    ecef_of_lla_d(ecef, lla)
    enu = EnuCoor_d()
    enu_of_ecef_point_d(enu, ltp_origin, ecef)
    enu_res[i, 0] = enu.x
    enu_res[i, 1] = enu.y
    #print("enu x=%.3f, y=%.3f" % (enu.x, enu.y))


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