#!/usr/bin/env python3
#
# Copyright (C) 2023 TUDelft
#
# This file is part of paparazzi.
#
# paparazzi is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi.  If not, see <http://www.gnu.org/licenses/>.
#

"""
Rotating wing status monitor (Paparazzi Center tool "RotWingStatus").

Listens on the Ivy bus to the telemetry of a rotating wing drone (ESC, STAB_ATTITUDE,
ROTATING_WING_STATE, AIR_DATA, INFO_MSG, IMU_HEATER, FUELCELL, POWER_DEVICE) and draws
the drone with the status of every motor, the wing state, the power and the fuel cell.
The window itself is in rotwing_viewer.py.

Optional speech warnings (-s 1) need pyttsx3 (see requirements.txt).
"""

import sys
import signal
import argparse

from PyQt5 import QtWidgets

import rotwing_viewer


def main():
    """Parse the arguments, open the monitor window and run the Qt main loop."""
    parser = argparse.ArgumentParser(description="Rotwing Status Monitor")
    parser.add_argument('-s', '--speech', dest='speech', default=False, type=bool, help="Rotwing status viewer speech")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)  # Ctrl-C in the terminal closes the program
    window = rotwing_viewer.RotWingWindow(args)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
