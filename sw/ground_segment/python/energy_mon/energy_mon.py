#!/usr/bin/env python3
#
# Copyright (C) 2018 TUDelft
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
Energy monitor (Center tool EnergyManager): battery cell state and flight time
prediction from the ENERGY, TEMP_ADC and AIR_DATA telemetry on the Ivy bus.
The window is in energy_mon_viewer.py, the battery model in battery_model.py.
"""

import sys
import argparse

from PyQt5 import QtWidgets

import energy_mon_viewer


def main():
    """Parse the arguments and run the window"""
    parser = argparse.ArgumentParser(description="Energy monitor: battery state and flight time left")
    parser.add_argument("-b", "--ivy_bus", dest="ivy_bus", default=None,
                        help="Ivy bus address:port (default: IVY_BUS environment variable or the Ivy default)")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    window = energy_mon_viewer.EnergyMonWindow(args.ivy_bus)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
