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
from scipy import optimize
import matplotlib.pyplot as plt

import control_effectiveness_utils as ut
from control_effectiveness_utils import Configuration

from typing import Optional


########## Helper functions ##########    
    
def get_time_from_conf(conf:dict, start:float, end:float, freq:Optional[float], data):
    """ Extract time data and frequency from configuration hint, arguments and data

    Args:
        conf (dict):    Configuration dictionnary (parsed JSON)
        start (float):  Start time of the considered data 
        end (float):    End time of the considered data
        freq (Optional[float]): Data Frequency hint
        data (_type_): Data source

    Raises:
        ValueError: Measured a negative period in the signal

    Returns:
        (float,float,float,float): start,end,freq,time
    """
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
                    freq = float(np.round(1. / period))
                    print("Using auto freq:", freq)
                else:
                    raise ValueError("Invalid freq")
            break
    return start,end,freq,time


def fit_eff_matrix(conf:Configuration, inputs:np.ndarray, commands:np.ndarray, v_inv:np.ndarray, verbose:bool=False, weighting:bool=False):
    """ Fit the command efficiency using a linear model, yielding a matrix

    Args:
        conf (Configuration):   Additional configuration data
        inputs (np.ndarray):    Acceleration and rotation rates
        commands (np.ndarray):  Commands given
        v_inv (np.ndarray):     Inversion matrix from virtual to standard commands
        verbose (bool, optional): Display fitting messages. Defaults to False.
        weighting (bool, optional): Use a weighted estimation based on command magnitue (low command should imply low effects). Defaults to False.

    Returns:
        (np.ndarray,np.ndarray): Efficiency matrix and estimation residuals
    """
    nb_in = conf.nb_in
    nb_out = conf.nb_out
    
    output = np.zeros((nb_in, nb_out))
    
    residuals = []
    if weighting:
        for i in range(nb_in):
            weights = np.linalg.norm(commands,axis=1)            
            axis_fit,sol = ut.fit_weighted_lin_lstsq(commands,inputs[:,i],weights)
            output[[i],:] = axis_fit.T
            # cmd_fit = np.dot(commands, axis_fit)
            residuals.append(sol.cost)
    else:
        for i in range(nb_in):
            cmd = np.multiply(commands, conf.mixing[[i],:])
            axis_fit,res = ut.fit_axis(cmd, inputs[:,[i]], str(i), verbose)
            output[[i],:] = np.matmul(v_inv, axis_fit).T
            residuals.append(res)
    
    return output,np.array(residuals)

def make_virtual_cmd(virtual_cmd:np.ndarray, commands:np.ndarray):
    """Given the virtual command transformation matrix and somes commands,
    transform them.

    Args:
        virtual_cmd (np.ndarray): Conversion matrix from standard command to virtual ones
        commands (np.ndarray): Array of commands

    Returns:
        np.ndarray: Converted commands
    """
    v_nb = virtual_cmd.shape[0]
    nb_data = commands.shape[0]
    v_commands = np.zeros((nb_data, v_nb))
    for i in range(nb_data):
        v_commands[i,:] = np.matmul(virtual_cmd, commands[i,:])
    return v_commands

def find_eff_matrix(conf:Configuration, start:int, end:int, data:np.ndarray, verbose:bool=False) -> tuple[np.ndarray,np.ndarray]:
    """ Extract input and command data, then use them for estimating efficiency coefficients.
    If given, convert to virtual commands first.

    Args:
        conf (Configuration):   Configuration details
        start (int):            Start time for data extraction
        end (int):              End time for data extraction
        data (np.ndarray):      Source data array
        verbose (bool, optional): Display detailed estimation info. Defaults to False.

    Returns:
        (np.ndarray,np.ndarray): Efficiency matrix and estimation residuals
    """
    inputs, raw_inputs, commands, raw_commands = ut.extract_filtered_data(conf, data, start, end)
    print(f"Handling {commands.shape} input datapoints...")
    if conf.virtual_cmd is not None:
        commands = make_virtual_cmd(conf.virtual_cmd, commands)
        v_inv = np.linalg.pinv(conf.virtual_cmd)
    else:
        v_inv = np.identity(conf.nb_out)
            
    return fit_eff_matrix(conf, inputs, commands, v_inv, verbose)

def find_eff_matrix_with_meta_opt(conf:Configuration, start:int, end:int, data:np.ndarray, verbose:bool=False):
    """Extract input and command data, then use them for estimating efficiency coefficients.
    If given, convert to virtual commands first. Also perform a brute-force sweep of some
    estimation parameters to try improving the final accuracy. This relies on the "ranges" parameter in the configuration file.

    Args:
        conf (Configuration):   Configuration details
        start (int):            Start time for data extraction
        end (int):              End time for data extraction
        data (np.ndarray):      Source data array
        verbose (bool, optional): Display detailed estimation info. Defaults to False.

    Returns:
        (np.ndarray,np.ndarray): Efficiency matrix and estimation residuals
    """
    param_names = [k for k in conf.ranges.keys()]
    rranges = [slice(conf.ranges[k][0],conf.ranges[k][1]+conf.ranges[k][2],conf.ranges[k][2]) for k in param_names]
    
    ## Parse for printing problem size
    _, _, commands_print, _ = ut.extract_filtered_data(
                    conf, data, start, end)
    print(f"Handling {commands_print.shape} input datapoints with optimized parameters: {param_names}...")
    ## End parse
    
    def obj_fun(xs):
        if len(param_names) > 1:
            for i,x in enumerate(xs):
                conf.variables[param_names[i]] = x
        else:
            conf.variables[param_names[0]] = xs
                
        inputs, raw_inputs, commands, raw_commands = ut.extract_filtered_data(
                    conf, data, start, end)
        if conf.virtual_cmd is not None:
            commands = make_virtual_cmd(conf.virtual_cmd, commands)
            v_inv = np.linalg.pinv(conf.virtual_cmd)
        else:
            v_inv = np.identity(conf.nb_out)
                
        _,residuals = fit_eff_matrix(conf, inputs, commands, v_inv, False)
        return sum(residuals)
    
    if len(rranges) == 1:
        pname = param_names[0]
        res = optimize.minimize_scalar(obj_fun, (conf.ranges[pname][0],conf.ranges[pname][1]),tol=conf.ranges[pname][2]/2,
                                       options={'disp':3 if verbose else 0})
        
        x = [res.x]
    else:
        x,_,_,_ = optimize.brute(obj_fun, rranges, full_output=True,
                          finish=optimize.fmin)
    
    
    for i,x in enumerate(x):
        conf.variables[param_names[i]] = x
            
    inputs, raw_inputs, commands, raw_commands = ut.extract_filtered_data(
                conf, data, start, end)
    if conf.virtual_cmd is not None:
        commands = make_virtual_cmd(conf.virtual_cmd, commands)
        v_inv = np.linalg.pinv(conf.virtual_cmd)
    else:
        v_inv = np.identity(conf.nb_out)
            
    return fit_eff_matrix(conf, inputs, commands, v_inv, verbose)
        

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
    parser.add_argument("--no-virtual", help="Ignore use virtual commands",
                        action='store_true',dest="no_virtual")
    parser.add_argument("-s", "--start",
                      help="Start time",
                      action="store", dest="start", default="0")
    parser.add_argument("-e", "--end",
                      help="End time (-1 for unlimited time)",
                      action="store", dest="end", default=-1)
    parser.add_argument("-p", "--plot",
                      help="Show resulting plots",
                      action="store_true", dest="plot")
    parser.add_argument("-spec", "--spectrogram",
                      help="Show spectrogram instead of FFT in resutls plots",
                      action="store_true", dest="spec")
    parser.add_argument("-r", "--use_ranges",
                      action="store_true", dest="use_ranges")
    parser.add_argument("-v", "--verbose",
                      action="store_true", dest="verbose")
   
    args = parser.parse_args()
 
 
    # Display parameters
    verbose = args.verbose
    plot = args.plot
    
    # Set up time parameters
    start = int(args.start)
    end = int(args.end)
    freq = args.freq
    if freq is not None:
        freq = float(freq)
        

    # Read data
    if not os.path.isfile(args.data):
        raise FileNotFoundError(args.data)
    data = genfromtxt(args.data, delimiter=',', skip_header=1)
    
    # Read configuration
    if not os.path.isfile(args.config):
        raise FileNotFoundError(args.config)
    with open(args.config, 'r') as f:
        conf:dict = json.load(f)
        
    configuration = Configuration.from_dict(conf,args.vars,args.no_virtual)
    
    # Search for time vector in data from presets
    start, end, freq, time = get_time_from_conf(conf, start, end, freq, data)
    if time is None:
        start = int(start * freq)
        end = int(end * freq)
        time = np.arange(end-start) / freq # default time vector if not in data
    configuration.variables['freq'] = freq

    if len(configuration.ranges) > 0 and args.use_ranges:
        output,residuals = find_eff_matrix_with_meta_opt(configuration, start, end, data, verbose)
    else:
        output,residuals = find_eff_matrix(configuration, start, end, data, verbose)

    if plot:
        inputs, raw_inputs, commands, raw_commands = ut.extract_filtered_data(configuration, data, start, end)
        for i in range(configuration.nb_in):
            name = ut.get_name_by_index(conf, 'input', i)
            estimated_accel = commands @ output[i]
            lin_fit = ut.fit_lin(estimated_accel, inputs[:,[i]][:,0], name, verbose)
            fig = ut.plot_results(estimated_accel, inputs[:,[i]], raw_inputs[:,[i]], lin_fit, time, freq, name, spec=args.spec)
            fig.suptitle(f"Residual: {residuals[i]}")
        plt.show()
        
    ut.print_results(conf, configuration.variables, output)


if __name__ == "__main__":
    main()

