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
SV Info: shows the GPS satellites of the SVINFO messages on the Ivy bus.

The top part is a sky plot (north up, azimuth around the circle, elevation as
distance from the centre: 0 deg at the centre, 90 deg on the outer circle)
with one dot per satellite, coloured by its quality indicator (QI); the dot is
larger when the satellite is used in the fix. The bottom part shows one bar per
receiver channel with the signal strength (C/N0 in dBHz).

The window size and position are remembered between runs.
"""

import sys
import os
import math
import argparse

from PyQt5 import QtCore, QtGui, QtWidgets

PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))
PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                   '../../../..')))

sys.path.append(PPRZ_HOME + "/var/lib/python")  # pprzlink

from pprzlink.ivy import IvyMessagesInterface, IVY_BUS

WIDTH = 300     # default width of the window (and of the sky plot)
BARH = 140      # default height of the signal bars below the sky plot
CHANNEL = 16    # number of receiver channels shown as bars

# Colour of a satellite for each quality indicator (QI) value
QI_COLOURS = {
    0: QtGui.QColor(64, 64, 64),     # This channel is idle
    1: QtGui.QColor(128, 128, 128),  # Searching
    2: QtGui.QColor(0, 128, 128),    # Signal acquired
    3: QtGui.QColor(255, 0, 0),      # Signal detected but unusable
    4: QtGui.QColor(0, 0, 255),      # Code lock on signal
    5: QtGui.QColor(0, 255, 0),      # Code and carrier locked
    6: QtGui.QColor(0, 255, 0),      # Code and carrier locked
    7: QtGui.QColor(0, 255, 0),      # Code and carrier locked
}


class SvChannel(object):
    """Last SVINFO values of one receiver channel."""
    def __init__(self, chn, msg):
        self.chn = chn
        self.SVID = int(msg['SVID'])
        self.Flags = int(msg['Flags'])
        self.QI = int(msg['QI'])
        self.CNO = int(msg['CNO'])
        self.Elev = int(msg['Elev'])
        self.Azim = int(msg['Azim'])


class SVInfoWindow(QtWidgets.QWidget):
    """Window drawing the sky plot and the signal bars."""

    # Emitted from the Ivy thread with (channel, SvChannel), handled in the GUI thread
    sv_received = QtCore.pyqtSignal(int, object)

    def __init__(self, ivy_bus):
        super().__init__()
        self.setWindowTitle("SV Info")
        self.setWindowIcon(QtGui.QIcon(os.path.join(PPRZ_SRC, "sw/ground_segment/python/svinfo/svinfo.ico")))

        # Restore the window size and position of the last run
        self.settings = QtCore.QSettings("paparazzi", "svinfo")
        self.resize(int(self.settings.value("width", WIDTH)), int(self.settings.value("height", WIDTH + BARH)))
        if self.settings.contains("left"):
            self.move(int(self.settings.value("left")), int(self.settings.value("top")))

        self.sv = {}  # channel number -> SvChannel
        self.sv_received.connect(self.update_sv)

        self.interface = IvyMessagesInterface("svinfoframe", ivy_bus=ivy_bus)
        self.interface.subscribe(self.message_recv)

    def message_recv(self, ac_id, msg):
        """Ivy callback (Ivy thread): pass SVINFO messages to the GUI thread."""
        if msg.name == "SVINFO":
            chn = int(msg['chn'])
            self.sv_received.emit(chn, SvChannel(chn, msg))

    def update_sv(self, chn, sv):
        """Store the new channel values and redraw (GUI thread)."""
        self.sv[chn] = sv
        self.update()

    def paintEvent(self, event):
        """Draw the sky plot with the satellites and the signal bar of each channel."""
        tdx = -5
        tdy = -7
        th = 15

        w = self.width()
        hgt = self.height()
        bar = hgt - w - th - th

        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtCore.Qt.white)
        p.setPen(QtCore.Qt.black)

        # Background: 90 and 45 degrees elevation circles, centre, and the cardinal points
        p.setBrush(QtCore.Qt.NoBrush)
        centre = QtCore.QPointF(w / 2, w / 2)
        for r in (w / 2 - 1, w / 4 - 1, 1):
            p.drawEllipse(centre, r, r)
        p.setFont(QtGui.QFont("Serif", 11, QtGui.QFont.Bold))
        fm = p.fontMetrics()
        p.drawText(QtCore.QPointF(w / 2 + tdx, 2 + fm.ascent()), "N")
        p.drawText(QtCore.QPointF(w / 2 + tdx, w - 17 + fm.ascent()), "S")
        p.drawText(QtCore.QPointF(w - 15, w / 2 + tdy + fm.ascent()), "E")
        p.drawText(QtCore.QPointF(2, w / 2 + tdy + fm.ascent()), "W")

        # Satellites and their signal bars
        p.setFont(QtGui.QFont("Serif", 8))
        fm = p.fontMetrics()
        cw = w / CHANNEL  # width of one channel bar
        for chn, sv in self.sv.items():
            used = sv.Flags & 1
            s = 7 + used * 5  # dot radius, larger when used in the fix

            el = sv.Elev / 90.0 * w / 2.0  # distance from the centre
            az = math.radians(sv.Azim)
            x = w / 2.0 + math.sin(az) * el
            y = w / 2.0 - math.cos(az) * el

            p.setBrush(QI_COLOURS.get(sv.QI, QI_COLOURS[0]))
            p.drawEllipse(QtCore.QPointF(x, y), s, s)
            p.drawText(QtCore.QPointF(x + tdx, y + tdy + fm.ascent()), str(sv.SVID))

            bh = (bar - th - th) * sv.CNO / 55.0
            p.drawRect(QtCore.QRectF(cw * chn + 5 * (1 - used), hgt - th - bh, cw - 2 - 10 * (1 - used), bh))
            p.drawText(QtCore.QPointF(cw * chn, hgt - th + fm.ascent()), str(chn))
            p.drawText(QtCore.QPointF(cw * chn, hgt - bar + fm.ascent()), str(sv.CNO))
        p.end()

    def closeEvent(self, event):
        """Save the window geometry and stop the Ivy interface."""
        self.settings.setValue("width", self.width())
        self.settings.setValue("height", self.height())
        self.settings.setValue("left", self.x())
        self.settings.setValue("top", self.y())
        self.interface.shutdown()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description="Sky plot and signal strength of the GPS satellites (SVINFO messages)")
    parser.add_argument("-b", "--ivy_bus", default=IVY_BUS, help="Ivy bus address:port (default: $IVY_BUS or the Ivy default)")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    window = SVInfoWindow(args.ivy_bus)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
