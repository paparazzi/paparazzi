#!/usr/bin/env python3
#
# Copyright (C) 2018 Ewoud Smeur
# Copyright (C) 2020 Gautier Hattenberger <gautier.hattenberger@enac.fr>
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.


import os
import sys
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt

import control_effectiveness_utils as ut

def process_data(conf, f_name, start, end, freq=None, variables=None, verbose=False, use_ranges=False, plot=False):

    # Read data from log file
    data = genfromtxt(f_name, delimiter=',', skip_header=1)
    N = data.shape[0]

    # extract variables
    var = {}
    if 'variables' in conf:
        var = conf['variables']
    if variables is not None:
        # overwrite default value if needed
        for var_name, value in variables:
            if var_name not in var:
                print(f"Variable name '{var_name}' not in list '{var.keys()}'")
                break
            try:
                var[var_name] = float(value)
            except:
                print(f"Variable value '{value}' not a float or int")

    # extract ranges
    ranges = None
    if 'ranges' in conf and use_ranges:
        ranges = conf['ranges']

    # Get number of inputs and outputs
    mixing = np.array(conf['mixing'])
    (nb_in, nb_out) = np.shape(mixing)
    if verbose:
        print("Nb of inputs:", nb_in)
        print("Nb of commands:", nb_out)
        print("Mixing matrix:")
        print(mixing)

    # Search for time vector
    time = None
    for el in conf['data']:
        if el['type'] == 'timestamp':
            # convert time limits to index in timestamp array
            (start, end) = ut.get_index_from_time(data[:,el['column']], start, end)
            # get time vector
            time = ut.apply_format(el, data[start:end, el['column']])
            # Auto freq if needed
            if freq is None:
                period = np.mean(np.diff(time))
                if period > 0.:
                    freq = np.round(1. / period)
                    print("Using auto freq:", freq)
                else:
                    print("Invalid freq")
                    sys.exit(1)
            break
    if time is None:
        start = int(start * freq)
        end = int(end * freq)
        time = np.arange(end-start) / freq # default time vector if not in data
    var['freq'] = freq

    output = np.zeros((nb_in, nb_out))

    if ranges is None:
        # Search and filter inputs and outputs
        inputs, raw_inputs, commands, raw_commands = ut.extract_filtered_data(conf, var, data, nb_in, nb_out, start, end)

        for i in range(nb_in):
            name = ut.get_name_by_index(conf, 'input', i)
            cmd = np.multiply(commands, mixing[[i],:])
            axis_fit = ut.fit_axis(cmd, inputs[:,[i]], name, verbose)
            output[[i],:] = axis_fit.T
            cmd_fit = np.dot(cmd, axis_fit)
            lin_fit, res = ut.fit_lin(cmd_fit[:,0], inputs[:,[i]][:,0], name, True)
            ut.plot_results(cmd_fit, inputs[:,[i]], raw_inputs[:,[i]], lin_fit, time, freq, name)

    else:
        for e in ranges:
            r = ranges[e]
            lin_var = np.arange(r[0], r[1]+r[2], r[2])
            lin_res = np.zeros(lin_var.shape)
            best = np.inf
            best_result = None
            tmp_output = np.zeros((nb_in, nb_out))
            for j, v in enumerate(lin_var):
                var[e] = v
                inputs, raw_inputs, commands, raw_commands = ut.extract_filtered_data(
                        conf, var, data, nb_in, nb_out, start, end)
                res_total = 0.
                for i in range(nb_in):
                    name = ut.get_name_by_index(conf, 'input', i)
                    cmd = np.multiply(commands, mixing[[i],:])
                    axis_fit = ut.fit_axis(cmd, inputs[:,[i]], name, verbose)
                    tmp_output[[i],:] = axis_fit.T
                    cmd_fit = np.dot(cmd, axis_fit)
                    lin_fit, residual = ut.fit_lin(cmd_fit[:,0], inputs[:,[i]][:,0], name, verbose)
                    res_total += float(residual[0])

                if res_total < best:
                    best = res_total
                    best_result = v
                    output = tmp_output.copy()
                lin_res[j] = res_total

            # show results for this range
            print(f'Best result for {e} with value {best_result:.2f} (res = {best:.5E})')
            ut.plot_residuals(lin_var, lin_res, e)
            var[e] = best_result # set best value in variables

    ut.print_results(conf, var, output)

    if plot:
        plt.show()

def main():
    from argparse import ArgumentParser
    import json

    parser = ArgumentParser(description="Control effectiveness estimation tool")
    parser.add_argument("config", help="JSON configuration file")
    parser.add_argument("data", help="Log file for parameter estimation")
    parser.add_argument("-f", "--sample_freq", dest="freq",
                      help="Sampling frequency, trying auto freq if not set")
    parser.add_argument("-var", "--variable", dest="vars", action='append', nargs=2,
                      metavar=('var_name','value'),
                      help="Set variables by name, 'None' for config file default")
    parser.add_argument("-s", "--start",
                      help="Start time",
                      action="store", dest="start", default="0")
    parser.add_argument("-e", "--end",
                      help="End time (-1 for unlimited time)",
                      action="store", dest="end", default=-1)
    parser.add_argument("-p", "--plot",
                      help="Show resulting plots",
                      action="store_true", dest="plot")
    parser.add_argument("-r", "--use_ranges",
                      action="store_true", dest="use_ranges")
    parser.add_argument("-v", "--verbose",
                      action="store_true", dest="verbose")
    args = parser.parse_args()

    if not os.path.isfile(args.config) and not os.path.isfile(args.data):
        print("Config or data files are not valid")
        sys.exit(1)

    start = int(args.start)
    end = int(args.end)
    freq = args.freq
    if freq is not None:
        freq = float(freq)

    with open(args.config, 'r') as f:
        conf = json.load(f)
        process_data(conf, args.data, start, end, freq, args.vars, args.verbose, args.use_ranges, args.plot)


if __name__ == "__main__":
    main()

