#!/usr/bin/env python3
#
# Copyright (C) 2016 TUDelft
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
Distance counter: shows on screen how far an aircraft has flown and for how long.

It listens on the Ivy bus to:
 - INS: local position (x, y); the horizontal distance between two messages is added up
 - ROTORCRAFT_STATUS: cpu_time; the time between two messages is added up
Works with both NPS (simulation) and a real aircraft (e.g. in the Optitrack arena).
The counter can be paused and reset with the buttons.
"""

import sys
import math
import signal
import argparse
from os import path, getenv
from PyQt5 import QtCore, QtGui, QtWidgets

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../..')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../..')))

sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python")  # pprzlink

from pprzlink.ivy import IvyMessagesInterface

# INS positions are sent in fixed point with 8 fractional bits
INS_POS_FRAC = 256.0


class DistanceCounter(QtWidgets.QWidget):
    """ Window with the INS data, the distance flown and the elapsed time """

    # Ivy callbacks run in the Ivy thread: data goes to the GUI through these signals
    ins_sig = QtCore.pyqtSignal(float, float, float)
    time_sig = QtCore.pyqtSignal(float)

    def __init__(self, ivy_bus=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Distance Counter")
        self.resize(300, 120)

        self.msg_count = 0      # number of INS messages received
        self.ins_x = 0.         # last position (m)
        self.ins_y = 0.
        self.ins_z = 0.
        self.init = False       # True once a first position was received after a reset
        self.distance = 0.      # distance flown (m)
        self.time_old = 0.      # last cpu_time (s)
        self.time_elapsed = 0.  # time counted (s)
        self.running = True     # counter running or paused

        self.setup_ui()
        self.update_labels()

        self.ins_sig.connect(self.on_ins)
        self.time_sig.connect(self.on_time)

        # Ivy: only the two messages used here
        if ivy_bus is None:
            self.interface = IvyMessagesInterface("DistanceCounter")
        else:
            self.interface = IvyMessagesInterface("DistanceCounter", ivy_bus=ivy_bus)
        self.interface.subscribe(self.message_recv, '^([^ ]* +(INS|ROTORCRAFT_STATUS)( .*|$))')

    def setup_ui(self):
        """ Text lines on the left, state and buttons on the right """
        lay = QtWidgets.QGridLayout(self)
        self.packets_label = QtWidgets.QLabel()
        self.data_label = QtWidgets.QLabel()
        self.distance_label = QtWidgets.QLabel()
        self.time_label = QtWidgets.QLabel()
        self.state_label = QtWidgets.QLabel()
        lay.addWidget(self.packets_label, 0, 0, 1, 3)
        lay.addWidget(self.data_label, 1, 0, 1, 3)
        lay.addWidget(self.distance_label, 2, 0)
        lay.addWidget(self.state_label, 2, 1, 1, 2)
        lay.addWidget(self.time_label, 3, 0)

        start_stop_button = QtWidgets.QPushButton("Start/Pause")
        start_stop_button.clicked.connect(self.on_start_stop)
        lay.addWidget(start_stop_button, 3, 1)
        reset_button = QtWidgets.QPushButton("Reset")
        reset_button.clicked.connect(self.on_reset)
        lay.addWidget(reset_button, 3, 2)
        lay.setRowStretch(4, 1)

    def message_recv(self, ac_id, msg):
        """ Ivy thread: extract the values and send them to the GUI thread """
        if msg.name == "INS":
            self.ins_sig.emit(float(msg['ins_x']) / INS_POS_FRAC,
                              float(msg['ins_y']) / INS_POS_FRAC,
                              float(msg['ins_z']) / INS_POS_FRAC)
        elif msg.name == "ROTORCRAFT_STATUS":
            self.time_sig.emit(float(msg['cpu_time']))

    def on_ins(self, x, y, z):
        """ New position: add the horizontal distance from the previous one """
        self.msg_count += 1
        if not self.init:
            self.init = True
        elif self.running:
            self.distance += math.sqrt((x - self.ins_x) ** 2 + (y - self.ins_y) ** 2)
        self.ins_x, self.ins_y, self.ins_z = x, y, z
        self.update_labels()

    def on_time(self, time_new):
        """ New cpu_time: add the time since the previous one """
        if time_new > self.time_old and self.time_old != 0 and self.running:
            self.time_elapsed += time_new - self.time_old
        self.time_old = time_new
        self.update_labels()

    def update_labels(self):
        """ Show the current values """
        self.packets_label.setText("INS Packets: {}".format(self.msg_count))
        self.data_label.setText("Data: {:.2f}, {:.2f}, {:.2f}".format(self.ins_x, self.ins_y, self.ins_z))
        self.distance_label.setText("Distance: {:.2f} m".format(self.distance))
        self.time_label.setText("Time elapsed: {:g} s".format(self.time_elapsed))
        self.state_label.setText("Counter running" if self.running else "Counter paused")

    def on_start_stop(self):
        """ Pause or restart counting """
        self.running = not self.running
        self.update_labels()

    def on_reset(self):
        """ Distance and time back to zero """
        self.time_old = 0.
        self.time_elapsed = 0.
        self.distance = 0.
        self.init = False
        self.update_labels()

    def closeEvent(self, event):
        """ Stop Ivy when the window closes """
        self.interface.shutdown()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description="Distance counter: distance flown (INS messages) and elapsed time "
                                                 "(ROTORCRAFT_STATUS) shown from the Ivy bus.",
                                     epilog="dist.py is part of the paparazzi-uav project.")
    parser.add_argument('-b', '--ivy_bus', dest='ivy_bus', default=None,
                        help="Ivy bus address and port [default: system Ivy bus]")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal.SIG_DFL)  # Ctrl-C closes the program
    app = QtWidgets.QApplication(sys.argv)
    window = DistanceCounter(args.ivy_bus)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
