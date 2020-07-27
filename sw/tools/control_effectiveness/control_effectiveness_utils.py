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

'''
Utility functions for control effectiveness estimation
'''

import sys
import numpy as np
import scipy as sp
from scipy import signal
import matplotlib.pyplot as plt
from matplotlib.pyplot import show

#
# functions for actuators model
#

def first_order_model(signal, tau):
    '''
    Apply a first order filter with (discrete) time constant tau
    '''
    return sp.signal.lfilter([tau], [1, tau-1], signal, axis=0)

def rate_limit_model(signal, max_rate):
    '''
    Apply rate limiter of signal
    '''
    return signal # TODO

#
# Utility functions
#

def diff_signal(signal, freq, order=1, filt=None):
    '''
    compute the nth-order derivative of a signal of fixed freq
    and by applying a filter if necessary
    '''
    diff = np.diff(signal, order)
    res = np.hstack((np.zeros((1,order)), diff.reshape(1,len(diff)))) * pow(freq, order)
    return res

def get_param(param, var):
    '''
    get param value as a number or from var if string
    '''
    if isinstance(param, str):
        if param in var:
            return var[param]
        else:
            print('Unknown variable', param)
            sys.exit(1)
    else:
        return param

def apply_filter(filt_name, params, signal, var):
    '''
    apply a filter to an input signal based on the config (name + params)
    '''
    if filt_name == '1st_order':
        # params = [tau]
        return first_order_model(signal, get_param(params[0], var))

    elif filt_name == 'rate_limit':
        # params = [max_rate]
        return rate_limit_model(signal, get_param(params[0], var))

    elif filt_name == 'diff_signal':
        # params = [order]
        return diff_signal(signal, var['freq'], get_param(params[0], var))

    elif filt_name == 'butter':
        # params = [order, Wn]
        b, a = sp.signal.butter(get_param(params[0], var), get_param(params[1], var)/(var['freq']/2))
        return sp.signal.lfilter(b, a, signal, axis=0)

    elif filt_name == 'mult':
        # params = [factor]
        return signal * get_param(params[0], var)

    elif filt_name == 'div':
        # params = [factor]
        return signal / get_param(params[0], var)

    else:
        print("Unknown filter type", filt_name)


def get_name_by_index(conf, type_, index):
    for el in conf['data']:
        if el['index'] == index and el['type'] == type_:
            return el['name']
    return None

def apply_format(conf, data):
    try:
        format_ = conf['format']
        if format_ == "float":
            return data
        if format_ == "bfp":
            res = conf['resolution']
            return data / pow(2, res)
        if format_ == "pprz":
            return data
        if format_ == "int":
            scale = conf['scale']
            return data * scale
        else:
            print("Unknown format:", format_)
            return data
    except:
        print("Format error:", conf)
        return data

def get_index_from_time(time, start, end):
    N = len(time)
    start_idx = None
    end_idx = None
    # if end is negative, select till the last value
    if end < 0:
        end_idx = N
    for i, t in enumerate(time):
        if start_idx is None and start < t:
            start_idx = i
        if end_idx is None and end < t:
            end_idx = i
    # if not found, select all vector
    if start_idx is None:
        start_idx = 0
    if end_idx is None:
        end_idx = N
    return (start_idx, end_idx)

#
# Display functions
#

def plot_results(x, y, t, start, end, label, show=False):
    '''
    plot two curves for comparison
    '''
    #print(np.shape(x), np.shape(y), np.shape(t))
    plt.figure()
    plt.plot(t, y)
    plt.plot(t, x)
    plt.xlabel('t [s]')
    plt.ylabel(label)
    plt.figure()
    plt.plot(x[start:end], y[start:end])
    plt.xlabel('command [pprz]')
    plt.ylabel(label)
    if show:
        plt.show()

def print_results():
    pass


#
# Optimization functions
#

def fit_axis(x, y, axis, start, end, verbose=False):
    c = np.linalg.lstsq(x[start:end], y[start:end])#, rcond=None)
    if verbose:
        print("Fit axis", axis)
        print(c[0]*1000)
    return c[0]

