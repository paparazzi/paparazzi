#! /usr/bin/env python

from __future__ import print_function, division

import time
import logging
import sys
from os import path, getenv

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.ivy import IvyMessagesInterface

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

import scipy
from scipy import optimize

import calibration_utils


class MagPlot(object):
    def __init__(self):
        # Setup the figure and axes...
        self.fig = plt.figure()
        #self.ax = self.fig.add_subplot(1, 1, 1, projection='3d')
        self.ax = p3.Axes3D(self.fig)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.set_ax_lim(10)
        self.ax.set_title("Raw Mag data")

        self.data = np.zeros((1, 3))
        self.max_lim = 1

        # Then setup FuncAnimation.
        self.ani = animation.FuncAnimation(self.fig, self.update, interval=100, init_func=self.setup_plot, blit=False)

    def set_ax_lim(self, lim):
        lim = [-lim, lim]
        self.ax.set_xlim3d(lim)
        self.ax.set_ylim3d(lim)
        self.ax.set_zlim3d(lim)

    def setup_plot(self):
        x = self.data[:, 0]
        y = self.data[:, 1]
        z = self.data[:, 2]
        self.scat = self.ax.scatter(x, y, z, alpha=1)

        # For FuncAnimation's sake, we need to return the artist we'll be using
        # Note that it expects a sequence of artists, thus the trailing comma.
        return self.scat,

    def show(self, block=True):
        plt.show(block=block)

    def update(self, i):
        logging.debug("updating scatter: %d with %s" % (i, len(self.data)))
        self.scat.set_offsets(self.data[:, 0:2])
        self.scat.set_3d_properties(self.data[:, 2], 'z')
        # We need to return the updated artist for FuncAnimation to draw..
        # Note that it expects a sequence of artists, thus the trailing comma.
        return self.scat,

    def add_data(self, data):
        logging.debug("adding data %s" % data)
        if len(self.data) == 1 and not np.any(self.data):
            self.data[0] = np.array(data)
        else:
            self.data = np.vstack((self.data, np.array(data)))
        max_lim = np.max(np.abs(data))
        if max_lim > self.max_lim:
            self.max_lim = max_lim
            self.set_ax_lim(max_lim)



class MagCalibrator(object):
    def __init__(self, plot_results=True, verbose=False):
        self._interface = IvyMessagesInterface("calib_mag")
        self.plotter = MagPlot()
        self.data = []
        self.flt_meas = []
        self.p0 = np.array([0, 0, 0, 0, 0, 0])
        self.optimization_done = False
        self.plot_results = plot_results

    def start_collect(self):
        self._interface.subscribe(self.message_recv, "(.*IMU_MAG_RAW.*)")

    def stop_collect(self):
        self._interface.unsubscribe_all()

    def message_recv(self, ac_id, msg):
        self.data.append(np.array([int(v) for v in msg.fieldvalues]))
        if self.plot_results:
            self.plotter.add_data((map(int, msg.fieldvalues)))

    def shutdown(self):
        if self._interface is not None:
            print("Shutting down ivy interface...")
            self._interface.shutdown()
            self._interface = None

    def __del__(self):
        self.shutdown()

    def calc_min_max_guess(self):
        if len(self.data) > 3:
            # filter out noisy measurements?
            self.flt_meas = np.array(self.data)
            self.p0 = calibration_utils.get_min_max_guess(self.flt_meas, 1.0)

    def print_min_max_guess(self):
        self.calc_min_max_guess()
        if self.data:
            print("Current guess from %d measurements: neutral [%d, %d, %d], scale [%.3f, %.3f, %.3f]" % (len(self.flt_meas),
                  int(round(self.p0[0])), int(round(self.p0[1])), int(round(self.p0[2])),
                  self.p0[3]*2**11, self.p0[4]*2**11, self.p0[5]*2**11))

    def calibrate(self):
        self.calc_min_max_guess()

        if len(self.flt_meas) < 10:
            logging.warning("Not enough measurements")
            return

        cp0, np0 = calibration_utils.scale_measurements(self.flt_meas, self.p0)
        logging.info("initial guess : avg "+str(np0.mean())+" std "+str(np0.std()))
        calibration_utils.print_xml(self.p0, "MAG", 11)

        def err_func(p, meas, y):
            cp, np = calibration_utils.scale_measurements(meas, p)
            err = y * scipy.ones(len(meas)) - np
            return err

        p1, cov, info, msg, success = optimize.leastsq(err_func, self.p0[:], args=(self.flt_meas, 1.0), full_output=1)
        self.optimization_done = success in [1, 2, 3, 4]
        if not self.optimization_done:
            logging.warning("Optimization error: ", msg)

        cp1, np1 = calibration_utils.scale_measurements(self.flt_meas, p1)

        if self.optimization_done:
            logging.info("optimized guess : avg " + str(np1.mean()) + " std " + str(np1.std()))
            calibration_utils.print_xml(p1, "MAG", 11)
        else:
            logging.info("last iteration of failed optimized guess : avg " + str(np1.mean()) + " std " + str(np1.std()))

        if self.plot_results:
            calibration_utils.plot_results("MAG", np.array(self.data), range(len(self.data)),
                                           self.flt_meas, cp0, np0, cp1, np1, 1.0, blocking=False)
            calibration_utils.plot_mag_3d(self.flt_meas, cp1, p1)



if __name__ == '__main__':
    import argparse
    logging.basicConfig(level=logging.INFO)
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--plot', action='store_true', help='Interactive plot')
    args = parser.parse_args()
    if args.plot:
        print("Close the interactive plot window to run the final calibration.")
    else:
        print("Press CTRL-C to stop data collection and run the final calibration.")
    try:
        mc = MagCalibrator(plot_results=args.plot)
        mc.start_collect()
        if args.plot:
            mc.plotter.show()
        else:
            while True:
                time.sleep(2)
                mc.print_min_max_guess()
    except KeyboardInterrupt:
        print("Stopping on request")

    mc.stop_collect()
    mc.calibrate()
    mc.shutdown()
