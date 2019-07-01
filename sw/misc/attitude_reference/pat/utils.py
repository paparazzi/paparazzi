#
# Copyright 2013-2014 Antoine Drouin (poinix@gmail.com)
#
# This file is part of PAT.
#
#    PAT is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    PAT is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with PAT.  If not, see <http://www.gnu.org/licenses/>.
#

"""
Utility functions
"""
from __future__ import print_function
import math
import numpy as np
import numpy.linalg as linalg
import pdb

"""
Unit convertions
"""


def rad_of_deg(d):
    return d / 180. * math.pi


def sqrad_of_sqdeg(d):
    return d / (180. * math.pi) ** 2


def deg_of_rad(r):
    return r * 180. / math.pi


def sqdeg_of_sqrad(r):
    return r * (180. / math.pi) ** 2


def rps_of_rpm(r):
    return r * 2. * math.pi / 60.


def rpm_of_rps(r):
    return r / 2. / math.pi * 60.


# http://en.wikipedia.org/wiki/Nautical_mile
def m_of_NM(nm):
    return nm * 1852.


def NM_of_m(m):
    return m / 1852.


# http://en.wikipedia.org/wiki/Knot_(speed)
def mps_of_kt(kt):
    return kt * 0.514444


def kt_of_mps(mps):
    return mps / 0.514444


# http://en.wikipedia.org/wiki/Foot_(unit)
def m_of_ft(ft):
    return ft * 0.3048


def ft_of_m(m):
    return m / 0.3048


# feet per minute to/from meters per second
def ftpm_of_mps(mps):
    return mps * 60. * 3.28084


def mps_of_ftpm(ftpm):
    return ftpm / 60. / 3.28084


"""
Cliping
"""


def norm_angle_0_2pi(a):
    while a > 2. * math.pi:
        a -= 2. * math.pi
    while a <= 0:
        a += 2. * math.pi
    return a


def norm_angle_mpi_pi(a):
    while a > math.pi:
        a -= 2. * math.pi
    while a <= -math.pi:
        a += 2. * math.pi
    return a


#
def saturate(_v, _min, _max):
    if _v < _min:
        return _min
    if _v > _max:
        return _max
    return _v


"""
Plotting
"""
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

my_title_spec = {'color': 'k', 'fontsize': 20}


def save_if(filename):
    if filename: matplotlib.pyplot.savefig(filename, dpi=80)


def prepare_fig(fig=None, window_title=None, figsize=(20.48, 10.24), margins=None):
    if fig is None:
        fig = plt.figure(figsize=figsize)
    #    else:
    #        plt.figure(fig.number)
    if margins:
        left, bottom, right, top, wspace, hspace = margins
        fig.subplots_adjust(left=left, right=right, bottom=bottom, top=top,
                            hspace=hspace, wspace=wspace)
    if window_title:
        fig.canvas.set_window_title(window_title)
    return fig


def decorate(ax, title=None, xlab=None, ylab=None, legend=None, xlim=None, ylim=None):
    ax.xaxis.grid(color='k', linestyle='-', linewidth=0.2)
    ax.yaxis.grid(color='k', linestyle='-', linewidth=0.2)
    if xlab:
        ax.xaxis.set_label_text(xlab)
    if ylab:
        ax.yaxis.set_label_text(ylab)
    if title:
        ax.set_title(title, my_title_spec)
    if legend is not None:
        ax.legend(legend, loc='best')
    if xlim is not None:
        ax.set_xlim(xlim[0], xlim[1])
    if ylim is not None:
        ax.set_ylim(ylim[0], ylim[1])


def ensure_ylim(ax, yspan):
    ymin, ymax = ax.get_ylim()
    if ymax - ymin < yspan:
        ym = (ymin + ymax) / 2
        ax.set_ylim(ym - yspan / 2, ym + yspan / 2)


def write_text(nrows, ncols, plot_number, text, colspan=1, loc=[[0.5, 9.7]], filename=None):
    #    ax = plt.subplot(nrows, ncols, plot_number)
    gs = gridspec.GridSpec(nrows, ncols)
    row, col = divmod(plot_number - 1, ncols)
    ax = plt.subplot(gs[row, col:col + colspan])
    plt.axis([0, 10, 0, 10])
    ax.get_xaxis().set_visible(False)
    ax.get_yaxis().set_visible(False)
    for i in range(0, len(text)):
        plt.text(loc[i][0], loc[i][1], text[i], ha='left', va='top')
    save_if(filename)


def plot_in_grid(time, plots, ncol, figure=None, window_title="None", legend=None, filename=None,
                 margins=(0.04, 0.08, 0.93, 0.96, 0.20, 0.34)):
    nrow = math.ceil(len(plots) / float(ncol))
    figsize = (10.24 * ncol, 2.56 * nrow)
    figure = prepare_fig(figure, window_title, figsize=figsize, margins=margins)
    #    pdb.set_trace()
    for i, (title, ylab, data) in enumerate(plots):
        ax = figure.add_subplot(nrow, ncol, i + 1)
        ax.plot(time, data)
        decorate(ax, title=title, ylab=ylab)
    if legend is not None:
        ax.legend(legend, loc='best')
    save_if(filename)
    return figure


"""
Misc
"""


def num_jacobian(X, U, P, dyn):
    s_size = len(X)
    i_size = len(U)
    epsilonX = (0.1 * np.ones(s_size)).tolist()
    dX = np.diag(epsilonX)
    A = np.zeros((s_size, s_size))
    for i in range(0, s_size):
        dx = dX[i, :]
        delta_f = dyn(X + dx / 2, 0, U, P) - dyn(X - dx / 2, 0, U, P)
        delta_f = delta_f / dx[i]
        #        print(delta_f)
        A[:, i] = delta_f

    epsilonU = (0.1 * np.ones(i_size)).tolist()
    dU = np.diag(epsilonU)
    B = np.zeros((s_size, i_size))
    for i in range(0, i_size):
        du = dU[i, :]
        delta_f = dyn(X, 0, U + du / 2, P) - dyn(X, 0, U - du / 2, P)
        delta_f = delta_f / du[i]
        B[:, i] = delta_f

    return A, B


def saturate(V, Sats):
    Vsat = np.array(V)
    for i in range(0, len(V)):
        if Vsat[i] < Sats[i, 0]:
            Vsat[i] = Sats[i, 0]
        elif Vsat[i] > Sats[i, 1]:
            Vsat[i] = Sats[i, 1]
    return Vsat


def print_lti_dynamics(A, B, txt=None, print_original_form=False, print_modal_form=False):
    if txt:
        print(txt)
    if print_original_form:
        print("A\n", A)
        print("B\n", B)
    w, M = np.linalg.eig(A)
    print("modes \n", w)
    if print_modal_form:
        #        print "eigen vectors\n", M
        #        invM = np.linalg.inv(M)
        #        print "invM\n", invM
        #        Amod = np.dot(np.dot(invM, A), M)
        #        print "Amod\n", Amod
        for i in range(len(w)):
            print(w[i], "->", M[:, i])
