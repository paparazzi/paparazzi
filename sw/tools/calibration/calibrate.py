#! /usr/bin/env python

import os
from optparse import OptionParser
import scipy
from scipy import optimize

import sensor_calibration


def main():
    usage = "usage: %prog [options] log_filename"
    parser = OptionParser(usage)
    parser.add_option("-i", "--id", dest="ac_id",
                      action="store",
                      help="aircraft id to use")
    parser.add_option("-s", "--sensor", dest="sensor",
                      help="sensor to calibrate ( ACCEL or MAG)",
                      action="store", default="ACCEL")
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose")
    (options, args) = parser.parse_args()
    if len(args) != 1:
        parser.error("incorrect number of arguments")
    else:
        if os.path.isfile(args[0]):
            filename = args[0]
        else:
            print args[0] + " not found"
            sys.exit(1)
    if options.verbose:
        print "reading file "+filename+" for aircraft "+options.ac_id+" and sensor "+options.sensor
    measurements = sensor_calibration.read_log(options.ac_id, filename, options.sensor)
    if options.verbose:
       print "found "+str(len(measurements))+" records"
    if options.sensor == "ACCEL":
        sensor_ref = 9.81
        sensor_res = 10
        noise_window = 20;
        noise_threshold = 40;
    else: # MAG
        sensor_ref = 1.
        sensor_res = 11
        noise_window = 10;
        noise_threshold = 1000;
    flt_meas, flt_idx = sensor_calibration.filter_meas(measurements, noise_window, noise_threshold)
    if options.verbose:
        print "remaining "+str(len(flt_meas))+" after low pass"
    p0 = sensor_calibration.get_min_max_guess(flt_meas, sensor_ref)
    cp0, np0 = sensor_calibration.scale_measurements(flt_meas, p0)
    print "initial guess : avg "+str(np0.mean())+" std "+str(np0.std())
#    print p0

    def err_func(p,meas,y):
        cp, np = sensor_calibration.scale_measurements(meas, p)
        err = y*scipy.ones(len(meas)) - np
        return err

    p1, success = optimize.leastsq(err_func, p0[:], args=(flt_meas, sensor_ref))
    cp1, np1 = sensor_calibration.scale_measurements(flt_meas, p1)

    print "optimized guess : avg "+str(np1.mean())+" std "+str(np1.std())
#    print p1

    sensor_calibration.print_xml(p1, options.sensor, sensor_res)
    print ""

    sensor_calibration.plot_results(measurements, flt_idx, flt_meas, cp0, np0, cp1, np1, sensor_ref)

if __name__ == "__main__":
    main()
