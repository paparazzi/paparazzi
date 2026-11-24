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
Nederdrone status window: fuel cell, battery and CAN ESC status from the Ivy bus.
The window itself is in fuel_cell_viewer.py.
"""

import argparse
import signal
import sys

from PyQt5 import QtWidgets

import fuel_cell_viewer


def main():
    """ Parse the arguments, open the window and run the Qt event loop """
    parser = argparse.ArgumentParser(description="Nederdrone fuel cell, battery and ESC status")
    parser.add_argument("-b", "--ivy_bus", dest="ivy_bus", default=None,
                        help="Ivy bus address:port (default: $IVY_BUS or the system default)")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    window = fuel_cell_viewer.FuelCellWidget(args.ivy_bus)
    window.show()
    # Ctrl-C in the terminal closes the window (and stops Ivy)
    signal.signal(signal.SIGINT, lambda *args: window.close())
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
