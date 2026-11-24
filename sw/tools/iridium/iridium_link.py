#!/usr/bin/env python3
#
# Copyright (C) 2016 TU Delft
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
Iridium Link Controller (Iridium dialer).

Configures the Iridium ground modem on a serial port, registers it in the
network and calls the air modem. During the call the data is bridged to
UDP like the Paparazzi UDP link: data from the aircraft is broadcast on
127.255.255.255 port 4242, data sent to port 4243 goes to the aircraft.
"""

import argparse
import socket
import sys
from os import path

from PyQt5 import QtCore, QtGui, QtWidgets

from iridium_protocol import IridiumModem

DEFAULT_DEVICE = "/dev/ttyUSB0"
DEFAULT_BAUD = 19200
DEFAULT_NUMBER = "00881234567890"
UDP_IN = 4243
UDP_OUT = 4242
UDP_BROADCAST = '127.255.255.255'


class PprzUdp(QtCore.QObject):
    """Receive datagrams on in_port, broadcast datagrams to out_port."""

    received = QtCore.pyqtSignal(bytes)

    def __init__(self, in_port, out_port, parent=None):
        super().__init__(parent)
        self.out_port = out_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(('', in_port))
        self.sock.setblocking(False)
        self.notifier = QtCore.QSocketNotifier(self.sock.fileno(), QtCore.QSocketNotifier.Read, self)
        self.notifier.activated.connect(self.read)

    def send(self, data):
        """Broadcast data on the local host."""
        self.sock.sendto(data, (UDP_BROADCAST, self.out_port))

    def read(self):
        """Read one datagram and pass it on."""
        try:
            data = self.sock.recv(65535)
        except OSError:
            return
        self.received.emit(data)


class IridiumLinkWindow(QtWidgets.QWidget):
    """Window with the settings, the modem status, a console and buttons."""

    def __init__(self, args, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Iridium Link Controller")
        self.setWindowIcon(QtGui.QIcon(path.join(path.dirname(path.abspath(__file__)), "sat.ico")))
        self.resize(800, 400)

        self.modem = IridiumModem(cpin1=args.pin1, cpin2=args.pin2, parent=self)
        self.udp = PprzUdp(args.udp_in, args.udp_out, parent=self)

        # Settings and status on the left, console on the right, buttons below
        vlay = QtWidgets.QVBoxLayout(self)
        hlay = QtWidgets.QHBoxLayout()
        vlay.addLayout(hlay)
        status_lay = QtWidgets.QVBoxLayout()
        hlay.addLayout(status_lay)

        status_lay.addWidget(QtWidgets.QLabel("Device"))
        self.entry_device = QtWidgets.QLineEdit(args.device)
        status_lay.addWidget(self.entry_device)
        status_lay.addWidget(QtWidgets.QLabel("Baud rate"))
        self.entry_baud = QtWidgets.QLineEdit(str(args.baud))
        status_lay.addWidget(self.entry_baud)
        status_lay.addWidget(QtWidgets.QLabel("Phone number"))
        self.entry_number = QtWidgets.QLineEdit(args.number)
        status_lay.addWidget(self.entry_number)
        self.chk_redial = QtWidgets.QCheckBox("Auto redial")
        status_lay.addWidget(self.chk_redial)

        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.HLine)
        status_lay.addWidget(line)
        self.chk_connected = self.status_box("Connected", status_lay)
        self.chk_configured = self.status_box("Configured", status_lay)
        self.chk_registered = self.status_box("Registered", status_lay)
        self.chk_in_call = self.status_box("In call", status_lay)
        status_lay.addStretch()

        self.console = QtWidgets.QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        hlay.addWidget(self.console, stretch=1)

        btn_lay = QtWidgets.QHBoxLayout()
        vlay.addLayout(btn_lay)
        self.btn_open = self.button("Open", self.open, btn_lay, True)
        self.btn_register = self.button("Register", self.modem.register, btn_lay, False)
        self.btn_call = self.button("Call", self.call, btn_lay, False)
        self.btn_hangup = self.button("Hangup", self.modem.hangup, btn_lay, False)
        self.btn_csq = self.button("CSQ", self.modem.get_csq, btn_lay, False)

        # Connect the modem and the UDP link
        self.modem.connected.connect(self.on_connected)
        self.modem.configured.connect(self.chk_configured.setChecked)
        self.modem.registered.connect(self.chk_registered.setChecked)
        self.modem.calling.connect(self.on_calling)
        self.modem.call_dropped.connect(self.on_call_dropped)
        self.modem.console.connect(self.console_write)
        self.modem.call_data.connect(self.udp.send)
        self.udp.received.connect(self.on_udp_data)

    def status_box(self, text, layout):
        """Read-only check box showing a modem state."""
        box = QtWidgets.QCheckBox(text)
        box.setEnabled(False)
        layout.addWidget(box)
        return box

    def button(self, text, slot, layout, enabled):
        """Push button calling slot when clicked."""
        btn = QtWidgets.QPushButton(text)
        btn.clicked.connect(lambda: slot())
        btn.setEnabled(enabled)
        layout.addWidget(btn)
        return btn

    def open(self):
        """Open the serial port with the device and baud rate entered."""
        try:
            baud = int(self.entry_baud.text())
        except ValueError:
            self.console_write("[ERROR] Wrong baud rate: %s\n" % self.entry_baud.text())
            return
        self.modem.open(self.entry_device.text(), baud)

    def call(self):
        """Call the phone number entered."""
        self.modem.call(self.entry_number.text())

    def on_connected(self, val):
        """Serial port opened or closed: update the buttons."""
        self.chk_connected.setChecked(val)
        self.btn_open.setEnabled(not val)
        self.btn_register.setEnabled(val)
        self.btn_call.setEnabled(val)
        self.btn_csq.setEnabled(val)

    def on_calling(self, val):
        """Call started or ended: update the buttons."""
        self.chk_in_call.setChecked(val)
        connected = self.modem.is_open()
        self.btn_call.setEnabled(connected and not val)
        self.btn_hangup.setEnabled(connected and val)
        self.btn_register.setEnabled(connected and not val)
        self.btn_csq.setEnabled(connected and not val)

    def on_call_dropped(self):
        """The modem ended the call: call again when auto redial is on."""
        if self.chk_redial.isChecked() and self.modem.is_open():
            self.call()

    def on_udp_data(self, data):
        """Send data from the UDP link to the aircraft, only during a call."""
        if self.modem.in_call:
            self.modem.transmit(data)

    def console_write(self, text):
        """Append text to the console and scroll down."""
        self.console.moveCursor(QtGui.QTextCursor.End)
        self.console.insertPlainText(text)
        self.console.moveCursor(QtGui.QTextCursor.End)

    def closeEvent(self, event):
        """Close the serial port when the window is closed."""
        self.modem.close()
        super().closeEvent(event)


def main():
    """Parse the arguments and run the window."""
    parser = argparse.ArgumentParser(description="Iridium Link Controller: call the air modem "
                                     "and bridge the call data to the Paparazzi UDP link.")
    parser.add_argument("-d", "--device", default=DEFAULT_DEVICE, help="serial device of the ground modem (default %(default)s)")
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD, help="baud rate (default %(default)s)")
    parser.add_argument("-n", "--number", default=DEFAULT_NUMBER, help="phone number of the air modem (default %(default)s)")
    parser.add_argument("--pin1", default="1111", help="SIM PIN1 code (default %(default)s)")
    parser.add_argument("--pin2", default="2222", help="SIM PIN2 code (default %(default)s)")
    parser.add_argument("--udp_in", type=int, default=UDP_IN, help="UDP port receiving data for the aircraft (default %(default)s)")
    parser.add_argument("--udp_out", type=int, default=UDP_OUT, help="UDP port the aircraft data is broadcast to (default %(default)s)")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    window = IridiumLinkWindow(args)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
