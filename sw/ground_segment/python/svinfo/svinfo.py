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

Elevation and azimuth follow the u-blox NAV-SVINFO / NAV-SAT definitions (NMEA
GSV uses the same): elevation in degrees above the horizon (0 horizon, 90
zenith, unknown when out of -90..90), azimuth in degrees from true north,
clockwise.

The top part is a sky plot (north up, east right as seen from above, the zenith
at the centre, 45 deg elevation on the inner circle and the horizon on the
outer circle) with one dot per satellite, coloured by its quality indicator
(QI); the dot is larger when the satellite is used in the fix. Satellites with
an unknown position (elevation out of range, or 0/0 while not used, as sent by
a receiver without orbit data) and satellites below the horizon get no dot. The bottom part shows one bar per
receiver channel with the signal strength (C/N0 in dBHz).

Only one aircraft is shown: the one given with -i, or else the first one that
sends SVINFO. Channels without news for STALE_TIME seconds are removed.

The window size and position are remembered between runs.
"""

import sys
import os
import math
import argparse
import time

from PyQt5 import QtCore, QtGui, QtWidgets

PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))
PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                   '../../../..')))

sys.path.append(PPRZ_HOME + "/var/lib/python")  # pprzlink
os.environ.setdefault("PAPARAZZI_HOME", PPRZ_HOME)  # pprzlink finds messages.xml with it

from pprzlink.ivy import IvyMessagesInterface, IVY_BUS

WIDTH = 300     # default width of the window (and of the sky plot)
BARH = 140      # default height of the signal bars below the sky plot
CHANNEL = 16    # number of receiver channels shown as bars
STALE_TIME = 5  # seconds without SVINFO after which a channel is removed

# Colour of a satellite for each quality indicator (QI) value
QI_COLOURS = {
    0: QtGui.QColor(64, 64, 64),     # No signal (channel idle)
    1: QtGui.QColor(128, 128, 128),  # Searching signal
    2: QtGui.QColor(0, 128, 128),    # Signal acquired
    3: QtGui.QColor(255, 0, 0),      # Signal detected but unusable
    4: QtGui.QColor(0, 0, 255),      # Code locked and time synchronized
    5: QtGui.QColor(0, 255, 0),      # Code and carrier locked and time synchronized
    6: QtGui.QColor(0, 255, 0),      # Code and carrier locked and time synchronized
    7: QtGui.QColor(0, 255, 0),      # Code and carrier locked and time synchronized
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
        self.time = time.monotonic()


def sky_position_known(sv):
    """True when the satellite has a known position above the horizon."""
    if not 0 <= sv.Elev <= 90:
        return False  # below the horizon, or unknown (u-blox: out of -90..90)
    return bool(sv.Flags & 1) or sv.Elev != 0 or sv.Azim != 0  # 0/0: no orbit data


class SVInfoWindow(QtWidgets.QWidget):
    """Window drawing the sky plot and the signal bars."""

    # Emitted from the Ivy thread with (channel, SvChannel), handled in the GUI thread
    sv_received = QtCore.pyqtSignal(str, int, object)

    def __init__(self, ivy_bus, ac_id=None):
        super().__init__()
        self.setWindowIcon(QtGui.QIcon(os.path.join(PPRZ_SRC, "sw/ground_segment/python/svinfo/svinfo.ico")))

        # Restore the window size and position of the last run
        self.settings = QtCore.QSettings("paparazzi", "svinfo")
        self.resize(int(self.settings.value("width", WIDTH)), int(self.settings.value("height", WIDTH + BARH)))
        if self.settings.contains("left"):
            self.move(int(self.settings.value("left")), int(self.settings.value("top")))

        self.ac_id = ac_id  # aircraft shown; None: the first one that sends SVINFO
        self.sv = {}  # channel number -> SvChannel
        self.sv_received.connect(self.update_sv)
        self.update_title()

        # Remove stale channels (e.g. after a GPS restart) and redraw
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.remove_stale)
        self.timer.start(1000)

        self.interface = IvyMessagesInterface("svinfoframe", ivy_bus=ivy_bus)
        self.interface.subscribe(self.message_recv)

    def message_recv(self, ac_id, msg):
        """Ivy callback (Ivy thread): pass SVINFO messages to the GUI thread."""
        if msg.name == "SVINFO":
            chn = int(msg['chn'])
            self.sv_received.emit(str(ac_id), chn, SvChannel(chn, msg))

    def update_title(self):
        self.setWindowTitle("SV Info" if self.ac_id is None else "SV Info (aircraft %s)" % self.ac_id)

    def update_sv(self, ac_id, chn, sv):
        """Store the new channel values of the shown aircraft and redraw (GUI thread)."""
        if self.ac_id is None:
            self.ac_id = ac_id
            self.update_title()
        if ac_id != self.ac_id:
            return
        self.sv[chn] = sv
        self.update()

    def remove_stale(self):
        now = time.monotonic()
        stale = [chn for chn, sv in self.sv.items() if now - sv.time > STALE_TIME]
        for chn in stale:
            del self.sv[chn]
        if stale:
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

        if not self.sv:
            p.drawText(QtCore.QRectF(0, w, w, hgt - w), QtCore.Qt.AlignCenter, "Waiting for SVINFO")

        # Satellites and their signal bars
        p.setFont(QtGui.QFont("Serif", 8))
        fm = p.fontMetrics()
        cw = w / CHANNEL  # width of one channel bar
        for chn, sv in self.sv.items():
            used = sv.Flags & 1
            s = 7 + used * 5  # dot radius, larger when used in the fix

            p.setBrush(QI_COLOURS.get(sv.QI, QI_COLOURS[0]))
            if sky_position_known(sv):
                el = (90 - sv.Elev) / 90.0 * w / 2.0  # distance from the centre: 0 at the zenith
                az = math.radians(sv.Azim)
                x = w / 2.0 + math.sin(az) * el
                y = w / 2.0 - math.cos(az) * el
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
        self.timer.stop()
        self.interface.shutdown()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description="Sky plot and signal strength of the GPS satellites (SVINFO messages)")
    parser.add_argument("-b", "--ivy_bus", default=IVY_BUS, help="Ivy bus address:port (default: $IVY_BUS or the Ivy default)")
    parser.add_argument("-i", "--ac_id", help="aircraft id to show (default: the first one that sends SVINFO)")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    window = SVInfoWindow(args.ivy_bus, args.ac_id)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
