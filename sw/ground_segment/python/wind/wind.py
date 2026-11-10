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
Wind Tool: estimate the wind from the ground speed and the airspeed of a rotorcraft.

The window shows, north up, the last 1000 ground speed vectors (blue dots, from ROTORCRAFT_FP)
and the last 1000 airspeed vectors (red dots, AIR_DATA airspeed in the direction of the heading
psi of ROTORCRAFT_FP). Drag the blue cloud with the mouse until it lies on the red one: the
distance you dragged is the wind, shown at the bottom ("Wind = ... m/s from ..." in degrees).
Circles every 5 m/s, up to 35 m/s.
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

sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface, IVY_BUS

WIDTH = 300.0           # reference size of the speed circle area (pixels)
BARH = 40.0             # reference height of the text bar below it (pixels)
MAX_AIRSPEED = 35.0     # speed at the edge of the circle area (m/s)
MSG_BUFFER_SIZE = 1000  # number of samples kept and drawn


class WindWidget(QtWidgets.QWidget):
    """Draws the speed circles and the two clouds; mouse drag moves the ground speed cloud."""

    # Ivy thread -> GUI thread: (message name, values)
    msg_sig = QtCore.pyqtSignal(str, tuple)

    def __init__(self, parent=None):
        super().__init__(parent)
        # ground speed (east, north) in m/s, ring buffer
        self.count_gs = 0
        self.n_gs = 0  # number of samples received, up to MSG_BUFFER_SIZE
        self.ground_gs_x = [0.0] * MSG_BUFFER_SIZE
        self.ground_gs_y = [0.0] * MSG_BUFFER_SIZE
        # airspeed (m/s) and heading (rad) at that moment, ring buffer
        self.count_as = 0
        self.n_as = 0
        self.last_heading = 0.0
        self.airspeed = [0.0] * MSG_BUFFER_SIZE
        self.heading = [0.0] * MSG_BUFFER_SIZE
        # offset of the ground speed cloud set with the mouse (pixels)
        self.click_x = 0
        self.click_y = 0
        self.click_on = False
        self.mid = int(WIDTH / 2)

        self.msg_sig.connect(self.add_sample)

    def message_recv(self, ac_id, msg):
        """Ivy callback (Ivy thread): convert the message and pass it to the GUI thread."""
        if msg.name == "ROTORCRAFT_FP":
            veast = float(msg['veast']) * 0.0000019
            vnorth = float(msg['vnorth']) * 0.0000019
            psi = float(msg['psi']) * 0.0139882
            self.msg_sig.emit(msg.name, (veast, vnorth, psi))
        elif msg.name == "AIR_DATA":
            self.msg_sig.emit(msg.name, (float(msg['airspeed']),))

    def add_sample(self, name, values):
        """Store one sample in the ring buffers (GUI thread) and redraw."""
        if name == "ROTORCRAFT_FP":
            veast, vnorth, psi = values
            self.ground_gs_x[self.count_gs] = veast
            self.ground_gs_y[self.count_gs] = vnorth
            self.last_heading = psi
            self.count_gs = (self.count_gs + 1) % MSG_BUFFER_SIZE
            self.n_gs = min(self.n_gs + 1, MSG_BUFFER_SIZE)
        elif name == "AIR_DATA":
            self.airspeed[self.count_as] = values[0]
            self.heading[self.count_as] = math.radians(self.last_heading)
            self.count_as = (self.count_as + 1) % MSG_BUFFER_SIZE
            self.n_as = min(self.n_as + 1, MSG_BUFFER_SIZE)
        self.update()

    def mousePressEvent(self, event):
        """Start dragging the ground speed cloud."""
        if event.button() == QtCore.Qt.LeftButton:
            self.click_on = True
            self.mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        """Stop dragging."""
        if event.button() == QtCore.Qt.LeftButton:
            self.click_on = False

    def mouseMoveEvent(self, event):
        """While dragging, the ground speed cloud is centred on the mouse."""
        if self.click_on:
            self.click_x = event.pos().x() - self.mid
            self.click_y = event.pos().y() - self.mid
            self.update()

    def wind(self, diameter):
        """Wind speed (m/s) and direction it comes from (deg, 0..360) from the mouse offset."""
        speed = math.hypot(self.click_x, self.click_y) / diameter * MAX_AIRSPEED
        direction = math.degrees(math.atan2(self.click_x, -self.click_y)) % 360.0
        return speed, direction

    def paintEvent(self, event):
        """Draw circles, compass letters, both clouds and the wind text."""
        w = float(self.width())
        h = float(self.height())
        # keep the aspect ratio of circle area + text bar
        if w / h > (WIDTH / (WIDTH + BARH)):
            w = (WIDTH / (WIDTH + BARH)) * h
        else:
            h = ((WIDTH + BARH) / WIDTH) * w

        self.mid = int(w / 2)
        diameter = w / 2  # radius of the MAX_AIRSPEED circle (pixels)
        scale = diameter / MAX_AIRSPEED

        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtCore.Qt.white)

        # speed circles every 5 m/s
        p.setPen(QtCore.Qt.black)
        p.setBrush(QtCore.Qt.NoBrush)
        for v in range(0, 40, 5):
            r = int(diameter * v / MAX_AIRSPEED)
            p.drawEllipse(QtCore.QPoint(self.mid, self.mid), r, r)

        # compass letters
        font = QtGui.QFont("Serif", 11, QtGui.QFont.Bold)
        p.setFont(font)
        fm = QtGui.QFontMetrics(font)
        p.drawText(self.mid - fm.width("N") // 2, fm.ascent() + 2, "N")
        p.drawText(self.mid - fm.width("S") // 2, int(w) - fm.descent() - 2, "S")
        p.drawText(int(w) - fm.width("E") - 2, self.mid + fm.ascent() // 2, "E")
        p.drawText(2, self.mid + fm.ascent() // 2, "W")

        # ground speed, north up, moved by the mouse offset
        p.setPen(QtCore.Qt.black)
        p.setBrush(QtGui.QColor(0, 0, 255))
        for gx, gy in zip(self.ground_gs_x[:self.n_gs], self.ground_gs_y[:self.n_gs]):
            p.drawEllipse(QtCore.QPoint(int(gx * scale + self.mid + self.click_x),
                                        int(-gy * scale + self.mid + self.click_y)), 2, 2)

        # airspeed in the direction of the heading, north up
        p.setBrush(QtGui.QColor(255, 0, 0))
        for a, hd in zip(self.airspeed[:self.n_as], self.heading[:self.n_as]):
            p.drawEllipse(QtCore.QPoint(int(a * math.sin(hd) * scale + self.mid),
                                        int(-a * math.cos(hd) * scale + self.mid)), 2, 2)

        # sample counters and mouse offset
        p.setFont(QtGui.QFont("Serif", 8))
        p.drawText(0, int(h) - 4, "#{}, {} | {}-{}".format(self.count_gs, self.count_as,
                                                           self.click_x, self.click_y))

        # result
        speed, direction = self.wind(diameter)
        font = QtGui.QFont("Serif")
        font.setPixelSize(max(1, int(16.0 / WIDTH * w)))
        p.setFont(font)
        p.drawText(0, int(w) + QtGui.QFontMetrics(font).ascent(),
                   "Wind = {:.1f} m/s from {:.0f}".format(speed, direction))
        p.end()


class WindWindow(QtWidgets.QMainWindow):
    """Main window: the wind widget, Ivy connection, saved size and position."""

    def __init__(self, ivy_bus):
        super().__init__()
        self.setWindowTitle("Wind Tool")
        self.setWindowIcon(QtGui.QIcon(os.path.join(PPRZ_SRC, "sw/ground_segment/python/wind/wind.png")))
        self.wind = WindWidget(self)
        self.setCentralWidget(self.wind)

        # restore the last size and position
        self.settings = QtCore.QSettings("paparazzi", "wind_conf")
        self.resize(int(self.settings.value("width", int(WIDTH))),
                    int(self.settings.value("height", int(WIDTH + BARH))))
        if self.settings.contains("left"):
            self.move(int(self.settings.value("left")), int(self.settings.value("top")))

        self.interface = IvyMessagesInterface("windframe", ivy_bus=ivy_bus)
        self.interface.subscribe(self.wind.message_recv)

    def closeEvent(self, event):
        """Save size and position, stop Ivy."""
        self.settings.setValue("width", self.width())
        self.settings.setValue("height", self.height())
        self.settings.setValue("left", self.x())
        self.settings.setValue("top", self.y())
        self.interface.shutdown()
        event.accept()


def main():
    """Parse the arguments and run the window."""
    parser = argparse.ArgumentParser(description="Wind Tool: estimate the wind from ground speed and airspeed")
    parser.add_argument("-b", "--ivy_bus", dest="ivy_bus", default=IVY_BUS,
                        help="Ivy bus address and port (default: $IVY_BUS or the pprzlink default)")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    window = WindWindow(args.ivy_bus)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
