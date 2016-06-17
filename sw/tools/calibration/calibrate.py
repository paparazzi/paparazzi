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

from __future__ import print_function

import sys
import os
from optparse import OptionParser
import scipy
from scipy import optimize

import calibration_utils


def main():
    usage = "usage: %prog [options] log_filename.data" + "\n" + "Run %prog --help to list the options."
    parser = OptionParser(usage)
    parser.add_option("-i", "--id", dest="ac_id",
                      action="store",
                      help="aircraft id to use")
    parser.add_option("-s", "--sensor", dest="sensor",
                      type="choice", choices=["ACCEL", "MAG"],
                      help="sensor to calibrate (ACCEL, MAG)",
                      action="store", default="ACCEL")
    parser.add_option("-p", "--plot",
                      help="Show resulting plots",
                      action="store_true", dest="plot")
    parser.add_option("--noise_threshold",
                      help="specify noise threshold instead of automatically determining it",
                      action="store", dest="noise_threshold", default=0)
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

    if os.path.getsize(filename) == 0:
        print("File specified has no data.")
        sys.exit(1)

    ac_ids = calibration_utils.get_ids_in_log(filename)
    if options.ac_id is None:
        if len(ac_ids) == 1:
            options.ac_id = ac_ids[0]
        else:
            parser.error("More than one aircraft id found in log file. Specify the id to use.")
    if options.verbose:
        print("Using aircraft id "+options.ac_id)

    if options.sensor == "ACCEL":
        sensor_ref = 9.81
        sensor_res = 10
        noise_window = 20
        noise_threshold = options.noise_threshold
    elif options.sensor == "MAG":
        sensor_ref = 1.
        sensor_res = 11
        noise_window = 10
        noise_threshold = 1000

    if options.verbose:
        print("reading file "+filename+" for aircraft "+options.ac_id+" and sensor "+options.sensor)

    # read raw measurements from log file
    measurements = calibration_utils.read_log(options.ac_id, filename, options.sensor)
    if len(measurements) == 0:
        print("Error: found zero IMU_"+options.sensor+"_RAW measurements for aircraft with id "+options.ac_id+" in log file!")
        sys.exit(1)
    if options.verbose:
        print("found "+str(len(measurements))+" records")

    # check that values are not all zero
    if not measurements.any():
        print("Error: all IMU_"+options.sensor+"_RAW measurements are zero!")
        sys.exit(1)

    # estimate the noise threshold if not explicitly given
    if noise_threshold <= 0:
        # mean over all measurements (flattended array) as approx neutral value
        neutral = scipy.mean(measurements)
        # find the median of measurement vector length after subtracting approximate neutral
        meas_median = scipy.median(scipy.array([scipy.linalg.norm(v - neutral) for v in measurements]))
        # set noise threshold to be below 10% of that
        noise_threshold = meas_median * 0.1
    if options.verbose:
        print("Using noise threshold of", noise_threshold, "for filtering.")

    # filter out noisy measurements
    flt_meas, flt_idx = calibration_utils.filter_meas(measurements, noise_window, noise_threshold)
    if options.verbose:
        print("remaining "+str(len(flt_meas))+" after filtering")
    if len(flt_meas) == 0:
        print("Error: found zero IMU_" + options.sensor + "_RAW measurements for aircraft with id " + options.ac_id +
              " in log file after filtering with noise threshold of " + noise_threshold +
              "!\nMaybe try specifying manually with the --noise_threshold option.")
        if options.plot:
            calibration_utils.plot_measurements(options.sensor, measurements)
        sys.exit(1)

    # get an initial min/max guess
    p0 = calibration_utils.get_min_max_guess(flt_meas, sensor_ref)
    cp0, np0 = calibration_utils.scale_measurements(flt_meas, p0)
    print("initial guess : avg "+str(np0.mean())+" std "+str(np0.std()))
#    print p0

    def err_func(p, meas, y):
        cp, np = calibration_utils.scale_measurements(meas, p)
        err = y*scipy.ones(len(meas)) - np
        return err

    p1, cov, info, msg, success = optimize.leastsq(err_func, p0[:], args=(flt_meas, sensor_ref), full_output=1)
    optimze_failed = success not in [1, 2, 3, 4]
    if optimze_failed:
        print("Optimization error: ", msg)
        print("Please try to provide a clean logfile with proper distribution of measurements.")
        #sys.exit(1)

    cp1, np1 = calibration_utils.scale_measurements(flt_meas, p1)

    if optimze_failed:
        print("last iteration of failed optimized guess : avg "+str(np1.mean())+" std "+str(np1.std()))
    else:
        print("optimized guess : avg "+str(np1.mean())+" std "+str(np1.std()))

    if not optimze_failed:
        calibration_utils.print_xml(p1, options.sensor, sensor_res)

    if options.plot:
        # if we are calibrating a mag, just draw first plot (non-blocking), then show the second
        if options.sensor == "MAG":
            calibration_utils.plot_results(options.sensor, measurements, flt_idx, flt_meas, cp0, np0, cp1, np1, sensor_ref, blocking=False)
            calibration_utils.plot_mag_3d(flt_meas, cp1, p1)
        # otherwise show the first plot (blocking)
        else:
            calibration_utils.plot_results(options.sensor, measurements, flt_idx, flt_meas, cp0, np0, cp1, np1, sensor_ref)

if __name__ == "__main__":
    main()
