#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2014  Antoine Drouin
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
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#

import math
import numpy as np
import scipy.signal
import matplotlib.pyplot as plt

import pat.utils as pu
import pat.algebra as pa

import control as ctl


def random_setpoint(time, dt_step=2):
    tf = time[0]
    sp = np.zeros((len(time), 3))
    sp_i = [0, 0, 0]
    for i in range(0, len(time)):
        if time[i] >= tf:
            ui = np.random.rand(3) - [0.5, 0.5, 0.5];
            ai = np.random.rand(1)
            n = np.linalg.norm(ui)
            if n > 0:
                ui /= n
            sp_i = pa.euler_of_quat(pa.quat_of_axis_angle(ui, ai))
            tf += dt_step
        sp[i] = sp_i
    return sp


def test_ref(r, time, setpoint):
    ref = np.zeros((len(time), 9))
    for i in range(1, time.size):
        sp_quat = pa.quat_of_euler(setpoint[i])
        r.update_quat(sp_quat, time[i] - time[i - 1])
        euler = pa.euler_of_quat(r.quat)
        ref[i] = np.concatenate((euler, r.vel, r.accel))
    return ref


def plot_ref(time, xref=None, sp=None, figure=None):
    margins = (0.05, 0.05, 0.98, 0.96, 0.20, 0.34)
    figure = pu.prepare_fig(figure, window_title='Reference', figsize=(20.48, 10.24), margins=margins)
    plots = [("$\phi$", "deg"), ("$\\theta$", "deg"), ("$\\psi$", "deg"),
             ("$p$", "deg/s"), ("$q$", "deg/s"), ("$r$", "deg/s"),
             ("$\dot{p}$", "deg/s2"), ("$\dot{q}$", "deg/s2"), ("$\dot{r}$", "deg/s2")]
    for i, (title, ylab) in enumerate(plots):
        ax = plt.subplot(3, 3, i + 1)
        if xref is not None: plt.plot(time, pu.deg_of_rad(xref[:, i]))
        pu.decorate(ax, title=title, ylab=ylab)
        if sp is not None and i < 3:
            plt.plot(time, pu.deg_of_rad(sp[:, i]))
    return figure


dt = 1. / 512.
time = np.arange(0., 4, dt)
sp = np.zeros((len(time), 3))
sp[:, 0] = pu.rad_of_deg(45.) * scipy.signal.square(math.pi / 2 * time + math.pi)
# sp[:, 1] = pu.rad_of_deg(5.)*scipy.signal.square(math.pi/2*time)
# sp[:, 2] = pu.rad_of_deg(45.)
# sp = random_setpoint(time)

# rs = [ctl.att_ref_analytic_disc(axis=0), ctl.att_ref_analytic_cont(axis=0), ctl.att_ref_default()]

args = {'omega': 10., 'xi': 0.7, 'sat_vel': pu.rad_of_deg(150.), 'sat_accel': pu.rad_of_deg(1800),
        'sat_jerk': pu.rad_of_deg(27000)}
rs = [ctl.att_ref_sat_naive(**args), ctl.att_ref_sat_nested(**args), ctl.att_ref_sat_nested2(**args)]
# rs.append(ctl.AttRefIntNative(**args))
rs.append(ctl.AttRefFloatNative(**args))

xrs = [test_ref(r, time, sp) for r in rs]
figure = None
for xr in xrs:
    figure = plot_ref(time, xr, None, figure)
figure = plot_ref(time, None, sp, figure)
legends = [r.name for r in rs] + ['Setpoint']
plt.subplot(3, 3, 3)
plt.legend(legends)
plt.show()
