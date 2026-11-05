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
from dataclasses import dataclass
from typing import Optional

import numpy as np
import scipy as sp
from scipy import signal
from scipy.fftpack import fft
from scipy.optimize import least_squares,OptimizeResult
import matplotlib.pyplot as plt
from matplotlib.pyplot import show


@dataclass
class Configuration:
    """ Helper structure to store data parsed from the JSON configuration
    """
    variables:dict[str,float]
    ranges:dict[str,tuple[float,float,float]]
    nb_in:int
    nb_out:int
    mixing:np.ndarray
    data_descr:dict
    virtual_cmd:Optional[np.ndarray] = None
        
    @staticmethod
    def from_dict(conf_dict:dict,variables_redef:Optional[dict]=None, no_virtual:bool=False):
        var = conf_dict.get('variables',{})
        if variables_redef is not None:
            for var_name, value in variables_redef:
                if var_name not in var:
                    print(f"Variable name '{var_name}' not in list '{var.keys()}'")
                    break
                try:
                    var[var_name] = float(value)
                except:
                    print(f"Variable value '{value}' not a float or int")
        
        ranges = conf_dict.get('ranges',[])
        
        mixing = np.array(conf_dict['mixing'])
        (nb_in, nb_out) = np.shape(mixing)
        
        data_descr = conf_dict['data']
        
        # check virtual command
        virtual_cmd = None
        if 'virtual_cmd' in conf_dict:
            virtual_cmd = extract_virtual_cmd(conf_dict)
            nb_out = virtual_cmd.shape[1]
        
        return Configuration(var,ranges,nb_in,nb_out,mixing,data_descr,virtual_cmd)
    



########## Baseline functions ##########

#
# functions for actuators model
#

def first_order_model(signal, freq, cutoff_freq):
    '''
    Apply a first order filter with cutoff freq
    '''
    tau = 1. - np.exp(-cutoff_freq / freq)
    zi = sp.signal.lfilter_zi([tau], [1, tau-1])
    out, _ = sp.signal.lfilter([tau], [1, tau-1], signal, zi=signal[0]*zi, axis=0)
    return out

def rate_limit_model(signal, max_rate):
    '''
    Apply rate limiter of signal
    '''
    out = np.full(signal.size, signal[0])
    for i in range(len(signal)-1):
        delta = signal[i] - out[i]
        out[i+1] = out[i] + np.sign(delta) * min(max_rate, np.abs(delta))
    return out

#
# Utility functions
#

def diff_signal(signal, freq, order=1, filt=None):
    '''
    compute the nth-order derivative of a signal of fixed freq
    and by applying a filter if necessary
    '''
    diff = np.diff(signal, order, prepend=np.full((order,),signal[0]))
    return diff * pow(freq, order)

def get_param(param, var):
    '''
    get param value as a number or from var if string
    '''
    if isinstance(param, str):
        # Use exception-based programming instead of crashing with code 1
        # if param in var:
        return var[param]
        # else:
            # print('Unknown variable', param)
            # sys.exit(1)
    else:
        return param

def apply_filter(filt_name, params, signal, var):
    '''
    apply a filter to an input signal based on the config (name + params)
    '''
    if filt_name == '1st_order':
        # params = [freq]
        return first_order_model(signal, var['freq'], get_param(params[0], var))

    elif filt_name == 'rate_limit':
        # params = [max_rate]
        return rate_limit_model(signal, get_param(params[0], var))

    elif filt_name == 'diff_signal':
        # params = [order]
        return diff_signal(signal, var['freq'], get_param(params[0], var))

    elif filt_name == 'butter':
        # params = [order, Wn]
        b, a = sp.signal.butter(get_param(params[0], var), get_param(params[1], var)/(var['freq']/2))
        zi = sp.signal.lfilter_zi(b, a)
        out, _ = sp.signal.lfilter(b, a, signal, zi=zi*signal[0], axis=0)
        return out

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

def extract_filtered_data(conf:Configuration, data:np.ndarray, start:int, end:int):
    
    # Recover values from struct instead of putting them in function argument
    nb_in = conf.nb_in
    nb_out = conf.nb_out
    
    inputs = np.zeros((end-start, nb_in))
    raw_inputs = np.zeros((end-start, nb_in))
    commands = np.zeros((end-start, nb_out))
    raw_commands = np.zeros((end-start, nb_out))
    for el in conf.data_descr:
        t = el['type']
        idx = el['index']
        col = el['column']
        if t == 'input' and idx >= 0:
            if idx >= nb_in:
                raise IndexError("Invalid input index {} for {} (max is {})".format(idx,el['name'],nb_in)) # Again, exception rather than exit code 1
            raw_inputs[:, idx] = apply_format(el, data[start:end, col])
            inputs[:, idx] = apply_format(el, data[start:end, col])
            for (filt_name, params) in el['filters']:
                # Useful (?) prints for understanding wrong initial behavior in differentiated signal
                # print(filt_name)
                # print(f"Input0 ({idx}) before filtering: ",inputs[0,idx])
                inputs[:,idx] = apply_filter(filt_name, params, inputs[:, idx].copy(), conf.variables)
                # print(f"Input0 ({idx}) after filtering: ",inputs[0,idx])
        if t == 'command' and idx >= 0:
            if idx >= nb_out:
                raise IndexError("Invalid command index {} for {} (max is {})".format(idx,el['name'],nb_out)) # Again, exception rather than exit code 1
            raw_commands[:, idx] = apply_format(el, data[start:end, col])
            commands[:, idx] = apply_format(el, data[start:end, col])
            for (filt_name, params) in el['filters']:
                commands[:,idx] = apply_filter(filt_name, params, commands[:, idx].copy(), conf.variables)    
    return inputs, raw_inputs, commands, raw_commands

def extract_virtual_cmd(conf_dict:dict) -> np.ndarray:
    """Get virtual commands from the configuration

    Args:
        conf_dict (dict): Configuration dictionnary (parsed JSON)

    Returns:
        np.ndarray: Transformation matrix from standard to virtual commands
    """
    virtual_cmd = np.array(conf_dict['virtual_cmd'])
    v_nb = virtual_cmd.shape[0]
    _, S, _ = np.linalg.svd(virtual_cmd)
    virtual_cmd = virtual_cmd / S.reshape(v_nb, 1)
    # nb_out = virtual_cmd.shape[1]
    
    return virtual_cmd


#
# Display functions
#

def plot_results(x, y, y_raw, z, t, freq, label, show=False, spec=False):
    '''
    Plot two curves for comparison
    
    Use scatter instead of line plot for real data (as data between the points does not exist!)
    '''
    fig = plt.figure(layout='constrained')
    ax_time = plt.subplot2grid((2,3), (0,0), colspan=2, rowspan=2)
    ax_fit = plt.subplot2grid((2,3), (0,2))
    ax_fft = plt.subplot2grid((2,3), (1,2))

    # time plot
    ax_time.plot(t, y, linestyle='-',linewidth=0.2,marker='.',markersize=3, label='force')
    ax_time.plot(t, x, label='cmd')
    ax_time.set_title(label)
    ax_time.set_xlabel('t [s]')
    ax_time.legend()

    # Fit line
    ax_fit.plot(x, y, linestyle='', marker='.',markersize=3,label='Force from cmd')
    p = np.poly1d(z[0])
    xp = np.linspace(np.min(x), np.max(x), 2)
    ax_fit.plot(xp,p(xp),label='lin fit')
    ax_fit.set_title(f'RMSE: {z[1][0]:3E}')
    
    # FFT
    N = len(y)
    if spec is False:
        # show FFT
        T = 1./freq
        yf = fft(y_raw)
        xf = np.linspace(0.0, 1.0/(2.0*T), int(N/2))
        ax_fft.plot(xf, 2.0/N * np.abs(yf[0:int(N/2)]))
        ax_fft.grid()
        ax_fft.set_xlabel('Freq (Hz)')
        ax_fft.set_yticks([])
        ax_fft.set_ylabel('FFT Amplitude')
    else:
        # show spectrogram
        ax_fft.specgram(y_raw.reshape(N), Fs=freq)
        ax_fft.set_xlabel('Time (s)')
        ax_fft.set_ylabel('Freq (Hz)')

    if show:
        plt.show()
        
    return fig

def plot_residuals(values, residuals, label, show=False):
    plt.figure()
    plt.plot(values, residuals)
    plt.xlabel(label)
    plt.ylabel('residual')
    plt.grid()
    plt.title(label)
    if show:
        plt.show()

def value_or_default(key, dic, default):
    if key in dic:
        return dic[key]
    else:
        return default

def print_results(conf, var, output):
    if 'display' not in conf:
        print("No display section")
        return
    
    # get display info
    disp = conf['display']

    print("\nAdd the following lines to your airframe file:\n")
    for d in disp:
        name = d['name']
        coef = value_or_default('coef', d, None)
        matrix = value_or_default('matrix', d, None)
        scaling = value_or_default('scaling', d, 1.)
        if matrix is not None:
            print(f'<define name="{name}" type="matrix">')
            for i in range(len(matrix)):
                l = []
                for e in matrix[i]:
                    if isinstance(e, (int, float, str)):
                        l.append("{:.2f}".format(scaling*get_param(e, var)))
                    else:
                        l.append("{:.2f}".format(scaling*output[e[0], e[1]]))
                print(f'  <field value="{", ".join(l)}" type="float[]"/>')
            print('</define>')
        elif coef is not None:
            if isinstance(coef, (int, float, str)):
                print('<define name="{}" value="{}"/>'.format(name, scaling*get_param(coef, var)))
            elif len(coef) == 2 and isinstance(coef[0], int) and isinstance(coef[1], int):
                print('<define name="{}" value="{:.2f}"/>'.format(name, scaling*output[coef[0], coef[1]]))
            else:
                l = []
                for e in coef:
                    if isinstance(e, (float, str)):
                        l.append("{:.2f}".format(scaling*get_param(e, var)))
                    elif isinstance(e, int):
                        l.append("{}".format(int(scaling*get_param(e, var))))
                    else:
                        l.append("{:.2f}".format(scaling*output[e[0], e[1]]))
                print('<define name="{}" value="{{{}}}"/>'.format(name, ', '.join(l)))



#
# Optimization functions
#

def fit_axis(x, y, name, verbose=True):
    c = np.linalg.lstsq(x, y, rcond=None)
    if verbose:
        print("Fit axis", name)
        print(c[0]*1000)
    res = np.sum(np.square(x @ c[0] - y)) # Recomputed residual manually (not computed when matrix is rank-deficient)
    return c[0],np.sqrt(res)/len(y)

def fit_lin(x, y, name, verbose=False):
    z = np.polyfit(x, y, 1, full=True)
    if verbose:
        print(f'Fit residual {name}: {float(z[1][0]):.5E} ; RMSE: {np.sqrt(z[1][0])/len(x)}')
    return z[0], np.sqrt(z[1])/len(x)

def fit_weighted_lin_lstsq(X, Y, w) -> tuple[np.ndarray,OptimizeResult]:
    """ Compute a weighted linear least-squares using scipy.optimize
    Optimize for A : min_A ||diag(w) * (X @ A - Y)||_2^2 

    Args:
        X (np.ndarray): Input data, dimensions (N,M)
        Y (np.ndarray): Target data, dimensions (N)
        w (np.ndarray): Weights, dimensions (N)
        verbose (bool): Verbose output
    Returns:
        np.ndarray: A vector of dimension (M), solution of the least square problem
    """
    
    def obj(a:np.ndarray):
        return (X@a - Y) * w
    
    def obj_jac(a:np.ndarray):
        return np.diag(w) @ X
    
    # Use the un-weighted solution as startup
    x0 = np.linalg.lstsq(X,Y)[0]
    
    sol:OptimizeResult = least_squares(obj,x0,obj_jac,method='lm')
    return sol.x,sol
    
