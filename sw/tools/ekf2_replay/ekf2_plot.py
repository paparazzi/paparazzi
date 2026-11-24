#!/usr/bin/env python3
#
# Copyright (C) 2026 Freek van Tienen <freek.v.tienen@gmail.com>
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

"""
Plot one or more estimates of ekf2_replay on top of each other, e.g. the replay
with the logged parameters and one with changed parameters.

  ekf2_plot.py orig.csv tuned.csv [-o plot.png]
"""

import argparse
import csv
import os

import matplotlib.pyplot as plt
import numpy as np

PANELS = [
    ("Position (m)", ["pos_n", "pos_e", "pos_d"]),
    ("Velocity (m/s)", ["vel_n", "vel_e", "vel_d"]),
    ("Attitude (deg)", ["roll", "pitch", "yaw"]),
    ("Accuracy", ["pos_acc_h", "pos_acc_v", "vel_acc_h", "vel_acc_v"]),
    ("Innovation test ratio", ["ratio_heading", "ratio_vel", "ratio_pos", "ratio_hgt", "ratio_tas", "ratio_beta"]),
    ("Wind (m/s)", ["wind_n", "wind_e"]),
    ("Gyro bias (rad/s)", ["gyro_bias_x", "gyro_bias_y", "gyro_bias_z"]),
    ("Accel bias (m/s2)", ["accel_bias_x", "accel_bias_y", "accel_bias_z"]),
    ("Mag bias (gauss)", ["mag_bias_x", "mag_bias_y", "mag_bias_z"]),
]


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    data = {}
    for key in rows[0].keys():
        if key in ("control_status", "fault_status"):
            data[key] = np.array([int(r[key], 16) for r in rows])
        else:
            data[key] = np.array([float(r[key]) if r[key] not in ("", "nan") else np.nan for r in rows])
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", nargs="+", help="estimates written by ekf2_replay")
    parser.add_argument("-o", "--output", help="save the plot to this file instead of showing it")
    args = parser.parse_args()

    runs = [(os.path.splitext(os.path.basename(p))[0], load(p)) for p in args.csv]
    fig, axes = plt.subplots(len(PANELS), 1, sharex=True, figsize=(12, 2.4 * len(PANELS)))
    styles = ["-", "--", ":", "-."]
    for ax, (title, keys) in zip(axes, PANELS):
        for k, key in enumerate(keys):
            for r, (name, d) in enumerate(runs):
                label = key if len(runs) == 1 else "%s %s" % (key, name)
                ax.plot(d["time"], d[key], styles[r % len(styles)], color="C%d" % k, label=label, linewidth=1)
        ax.set_ylabel(title)
        ax.grid(True)
        ax.legend(loc="upper right", fontsize="x-small", ncol=len(runs))
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()

    # summary of the innovation test ratios (a well tuned EKF has most of them well below 1)
    for name, d in runs:
        ratios = ", ".join("%s %.2f" % (k[6:], np.nanmean(d[k])) for k in PANELS[4][1] if np.nanmax(d[k]) > 0)
        print("%s: mean test ratio %s" % (name, ratios))

    if args.output:
        fig.savefig(args.output, dpi=100)
    else:
        plt.show()


if __name__ == "__main__":
    main()
