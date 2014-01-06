#! /usr/bin/env python

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

#
# calibrate gyrometers using turntable measurements
#

from __future__ import print_function, division

from optparse import OptionParser
import os
import sys

from scipy import linspace, polyval, stats

import matplotlib.pyplot as plt

import calibration_utils

#
# lisa 3
# p : a=-4511.16 b=31948.34, std error= 0.603
# q : a=-4598.46 b=31834.48, std error= 0.734
# r : a=-4525.63 b=32687.95, std error= 0.624
#
# lisa 4
# p : a=-4492.05 b=32684.94, std error= 0.600
# q : a=-4369.63 b=33260.96, std error= 0.710
# r : a=-4577.13 b=32707.72, std error= 0.730
#
# crista
# p : a= 3864.82 b=31288.09, std error= 0.866
# q : a= 3793.71 b=32593.89, std error= 3.070
# r : a= 3817.11 b=32709.70, std error= 3.296
#



def main():
    usage = "usage: %prog --id <ac_id> --tt_id <tt_id> --axis <axis> [options] log_filename.data" + "\n" + "Run %prog --help to list the options."
    parser = OptionParser(usage)
    parser.add_option("-i", "--id", dest="ac_id",
                      action="store", type=int, default=-1,
                      help="aircraft id to use")
    parser.add_option("-t", "--tt_id", dest="tt_id",
                      action="store", type=int, default=-1,
                      help="turntable id to use")
    parser.add_option("-a", "--axis", dest="axis",
                      type="choice", choices=['p', 'q', 'r'],
                      help="axis to calibrate (p, q, r)",
                      action="store")
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose")
    (options, args) = parser.parse_args()
    if len(args) != 1:
        parser.error("incorrect number of arguments")
    else:
        if os.path.isfile(args[0]):
            filename = args[0]
        else:
            print(args[0] + " not found")
            sys.exit(1)
    if not filename.endswith(".data"):
        parser.error("Please specify a *.data log file")

    if options.ac_id < 0 or options.ac_id > 255:
        parser.error("Specify a valid aircraft id number!")
    if options.tt_id < 0 or options.tt_id > 255:
        parser.error("Specify a valid turntable id number!")
    if options.verbose:
        print("reading file "+filename+" for aircraft "+str(options.ac_id)+" and turntable "+str(options.tt_id))

    samples = calibration_utils.read_turntable_log(options.ac_id, options.tt_id, filename, 1, 7)

    if len(samples) == 0:
        print("Error: found zero matching messages in log file!")
        print("Was looking for IMU_TURNTABLE from id: "+str(options.tt_id)+" and IMU_GYRO_RAW from id: "+str(options.ac_id)+" in file "+filename)
        sys.exit(1)
    if options.verbose:
        print("found "+str(len(samples))+" records")

    if options.axis == 'p':
        axis_idx = 1
    elif options.axis == 'q':
        axis_idx = 2
    elif options.axis == 'r':
        axis_idx = 3
    else:
        parser.error("Specify a valid axis!")

    #Linear regression using stats.linregress
    t = samples[:, 0]
    xn = samples[:, axis_idx]
    (a_s, b_s, r, tt, stderr) = stats.linregress(t, xn)
    print('Linear regression using stats.linregress')
    print(('regression: a=%.2f b=%.2f, std error= %.3f' % (a_s, b_s, stderr)))
    print(('<define name="GYRO_X_NEUTRAL" value="%d"/>' % (b_s)))
    print(('<define name="GYRO_X_SENS" value="%f" integer="16"/>' % (pow(2, 12)/a_s)))

    #
    # overlay fited value
    #
    ovl_omega = linspace(1, 7.5, 10)
    ovl_adc = polyval([a_s, b_s], ovl_omega)

    plt.title('Linear Regression Example')
    plt.subplot(3, 1, 1)
    plt.plot(samples[:, 1])
    plt.plot(samples[:, 2])
    plt.plot(samples[:, 3])
    plt.legend(['p', 'q', 'r'])

    plt.subplot(3, 1, 2)
    plt.plot(samples[:, 0])

    plt.subplot(3, 1, 3)
    plt.plot(samples[:, 0], samples[:, axis_idx], 'b.')
    plt.plot(ovl_omega, ovl_adc, 'r')

    plt.show()


if __name__ == "__main__":
    main()
