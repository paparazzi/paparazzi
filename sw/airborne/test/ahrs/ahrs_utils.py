#! /usr/bin/env python

#  Copyright (C) 2011 Antoine Drouin
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

import subprocess
import numpy as np
import matplotlib.pyplot as plt


def run_simulation(ahrs_type, build_opt, traj_nb):
    print("\nBuilding ahrs")
    args = ["make", "clean", "run_ahrs_on_synth", "AHRS_TYPE=AHRS_TYPE_" + ahrs_type] + build_opt
    #print(args)
    p = subprocess.Popen(args=args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=False)
    outputlines = p.stdout.readlines()
    p.wait()
    for i in outputlines:
        print("   # " + i, end=' ')
    print()
    print("Running simulation")
    print("   using traj " + str(traj_nb))
    p = subprocess.Popen(args=["./run_ahrs_on_synth", str(traj_nb)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         shell=False)
    outputlines = p.stdout.readlines()
    p.wait()
    #    for i in outputlines:
    #        print("    "+i, end=' ')
    #    print("\n")

    ahrs_data_type = [('time', 'float32'),
                      ('phi_true', 'float32'), ('theta_true', 'float32'), ('psi_true', 'float32'),
                      ('p_true', 'float32'), ('q_true', 'float32'), ('r_true', 'float32'),
                      ('bp_true', 'float32'), ('bq_true', 'float32'), ('br_true', 'float32'),
                      ('phi_ahrs', 'float32'), ('theta_ahrs', 'float32'), ('psi_ahrs', 'float32'),
                      ('p_ahrs', 'float32'), ('q_ahrs', 'float32'), ('r_ahrs', 'float32'),
                      ('bp_ahrs', 'float32'), ('bq_ahrs', 'float32'), ('br_ahrs', 'float32')]

    mydescr = np.dtype(ahrs_data_type)
    data = [[] for dummy in xrange(len(mydescr))]
    #    import code; code.interact(local=locals())
    for line in outputlines:
        if line.startswith("#"):
            print("   " + line, end=' ')
        else:
            fields = line.strip().split(' ')
            #print(fields)
            for i, number in enumerate(fields):
                data[i].append(number)
    print()
    for i in xrange(len(mydescr)):
        data[i] = np.cast[mydescr[i]](data[i])

    return np.rec.array(data, dtype=mydescr)


def plot_simulation_results(plot_true_state, lsty, label, sim_res):
    print("Plotting Results")

    #    f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True, sharey=True)

    plt.subplot(3, 3, 1)
    plt.plot(sim_res.time, sim_res.phi_ahrs, lsty, label=label)
    plt.ylabel('degres')
    plt.title('phi')
    plt.legend()

    plt.subplot(3, 3, 2)
    plt.plot(sim_res.time, sim_res.theta_ahrs, lsty)
    plt.title('theta')

    plt.subplot(3, 3, 3)
    plt.plot(sim_res.time, sim_res.psi_ahrs, lsty)
    plt.title('psi')

    plt.subplot(3, 3, 4)
    plt.plot(sim_res.time, sim_res.p_ahrs, lsty)
    plt.ylabel('degres/s')
    plt.title('p')

    plt.subplot(3, 3, 5)
    plt.plot(sim_res.time, sim_res.q_ahrs, lsty)
    plt.title('q')

    plt.subplot(3, 3, 6)
    plt.plot(sim_res.time, sim_res.r_ahrs, lsty)
    plt.title('r')

    plt.subplot(3, 3, 7)
    plt.plot(sim_res.time, sim_res.bp_ahrs, lsty)
    plt.ylabel('degres/s')
    plt.xlabel('time in s')
    plt.title('bp')

    plt.subplot(3, 3, 8)
    plt.plot(sim_res.time, sim_res.bq_ahrs, lsty)
    plt.xlabel('time in s')
    plt.title('bq')

    plt.subplot(3, 3, 9)
    plt.plot(sim_res.time, sim_res.br_ahrs, lsty)
    plt.xlabel('time in s')
    plt.title('br')

    if plot_true_state:
        plt.subplot(3, 3, 1)
        plt.plot(sim_res.time, sim_res.phi_true, 'r--')
        plt.subplot(3, 3, 2)
        plt.plot(sim_res.time, sim_res.theta_true, 'r--')
        plt.subplot(3, 3, 3)
        plt.plot(sim_res.time, sim_res.psi_true, 'r--')
        plt.subplot(3, 3, 4)
        plt.plot(sim_res.time, sim_res.p_true, 'r--')
        plt.subplot(3, 3, 5)
        plt.plot(sim_res.time, sim_res.q_true, 'r--')
        plt.subplot(3, 3, 6)
        plt.plot(sim_res.time, sim_res.r_true, 'r--')
        plt.subplot(3, 3, 7)
        plt.plot(sim_res.time, sim_res.bp_true, 'r--')
        plt.subplot(3, 3, 8)
        plt.plot(sim_res.time, sim_res.bq_true, 'r--')
        plt.subplot(3, 3, 9)
        plt.plot(sim_res.time, sim_res.br_true, 'r--')


def show_plot():
    plt.show()
