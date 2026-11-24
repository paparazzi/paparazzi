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
Iridium ground modem over a serial port (AT commands).

The modem is in one of two modes:
 - terminal mode: AT commands are sent one by one, each waits for OK or
   ERROR (or a timeout) before the next one is sent;
 - call mode (after ATD): all bytes are data for the aircraft, except
   NO CARRIER / BUSY, which end the call.

Everything runs in the Qt event loop: the serial port is read when a
QSocketNotifier says data is waiting, the command timeout is a QTimer.
"""

import re

import serial
from PyQt5.QtCore import QObject, QSocketNotifier, QTimer, pyqtSignal

DELIMITER = b'\r\n'     # Line end used by the Iridium modem
NO_CARRIER = 'NO CARRIER'
BUSY = 'BUSY'


class IridiumModem(QObject):
    """Configure the modem, register, call and hang up; pass call data on."""

    connected = pyqtSignal(bool)    # Serial port opened / closed
    configured = pyqtSignal(bool)   # Modem configuration succeeded / failed
    calling = pyqtSignal(bool)      # Call started / ended
    call_dropped = pyqtSignal()     # Call ended by the modem (NO CARRIER, BUSY)
    registered = pyqtSignal(bool)   # Registered in the Iridium network
    call_data = pyqtSignal(bytes)   # Data received from the aircraft
    console = pyqtSignal(str)       # Text for the console

    serial_ctrl = True      # Do we use CTS and RTS?
    timeout = 3             # Timeout for receiving an OK or ERROR in seconds
    show_data = True        # Show data in hex while in call

    def __init__(self, cpin1='1111', cpin2='2222', parent=None):
        super().__init__(parent)
        self.cpin1 = cpin1          # PIN code 1 for the SIM card
        self.cpin2 = cpin2          # PIN code 2 for the SIM card
        self.serial = None
        self.notifier = None
        self.last_cmd = ''          # Last command transmitted
        self.echo_enabled = True    # Whether echo is enabled
        self.in_call = False        # Are we currently in call
        self.recv_data = b''        # The received data until now
        self.queue = []             # Commands waiting to be sent: (cmd, wait_ok, on_done)
        self.waiting = None         # Callback of the command waiting for OK/ERROR
        self.cpin_state = ''        # Last answer to AT+CPIN?
        self.pin_sent = False       # PIN already sent since the last Register

        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.cmd_timeout)

    def open(self, device, baudrate):
        """Open the serial port and configure the modem."""
        try:
            self.serial = serial.Serial(device, baudrate, timeout=0)
        except (serial.SerialException, ValueError) as e:
            self.serial = None
            self.console.emit("[ERROR] %s\n" % e)
            return
        self.notifier = QSocketNotifier(self.serial.fileno(), QSocketNotifier.Read, self)
        self.notifier.activated.connect(self.serial_readable)
        self.echo_enabled = True
        self.in_call = False
        self.recv_data = b''
        self.connected.emit(True)
        self.configure()

    def close(self):
        """Close the serial port and forget the pending commands."""
        if self.serial is None:
            return
        self.notifier.setEnabled(False)
        self.notifier = None
        self.serial.close()
        self.serial = None
        self.timer.stop()
        self.queue = []
        self.waiting = None
        if self.in_call:
            self.in_call = False
            self.calling.emit(False)
        self.connected.emit(False)
        self.configured.emit(False)

    def is_open(self):
        """True when the serial port is open."""
        return self.serial is not None

    def configure(self):
        """Send the serial, echo and data connection settings."""
        if self.serial_ctrl:
            cfg_cmd = "ATE0 V1 X3"
        else:
            cfg_cmd = "ATE0 V1 X3 &D0 &K0"
        self.send_cmd(cfg_cmd, on_done=self.configure_echo_done)

    def configure_echo_done(self, ok):
        """After the echo settings: set the data connection settings."""
        if not ok:
            self.configure_failed()
            return
        self.echo_enabled = False
        self.send_cmd("AT+CBST=7,0,1", on_done=self.configure_cbst_done)

    def configure_cbst_done(self, ok):
        """After the data connection settings: report the result."""
        if not ok:
            self.configure_failed()
            return
        self.configured.emit(True)

    def configure_failed(self):
        """Close the port: it needs to be opened (and configured) again."""
        self.close()

    def register(self):
        """Ask the SIM state; the answer leads to PIN entry and registration."""
        self.pin_sent = False
        self.send_cmd("AT+CPIN?", on_done=self.cpin_done)

    def cpin_done(self, ok):
        """After AT+CPIN?: enter the PIN or register in the network."""
        if not ok:
            return
        if self.cpin_state in ('SIM PIN1', 'SIM PIN2'):
            if self.pin_sent:
                # Never send a wrong PIN twice: three wrong PINs block the SIM
                self.console.emit("[ERROR] SIM still asks for a PIN, check the PIN code\n")
                return
            self.pin_sent = True
            pin = self.cpin1 if self.cpin_state == 'SIM PIN1' else self.cpin2
            self.send_cmd("AT+CPIN=\"%s\"" % pin)
            self.send_cmd("AT+CPIN?", on_done=self.cpin_done)
        elif self.cpin_state == 'READY':
            self.send_cmd("AT+CREG=1")
            self.send_cmd("AT+CREG?")
        else:
            self.send_cmd("AT+CPIN?", on_done=self.cpin_done)

    def get_csq(self):
        """Ask the signal quality (answer shown in the console)."""
        self.send_cmd("AT+CSQ?")

    def call(self, number):
        """Dial the air modem; from now on received bytes are call data."""
        self.in_call = True
        self.send_cmd("ATD" + number, wait_ok=False)
        self.calling.emit(True)

    def hangup(self):
        """Leave the call mode with +++ and hang up with ATH."""
        if not self.in_call:
            return
        self.in_call = False
        self.calling.emit(False)
        self.send_cmd("+++", on_done=self.hangup_escape_done, delimiter=False)

    def hangup_escape_done(self, ok):
        """After +++ was answered with OK: hang up."""
        if ok:
            self.send_cmd("ATH")

    def transmit(self, data):
        """Write raw bytes to the modem (e.g. data for the aircraft)."""
        if self.serial is None:
            return
        try:
            self.serial.write(data)
        except serial.SerialException as e:
            self.console.emit("[ERROR] %s\n" % e)
            self.close()

    def send_cmd(self, cmd, wait_ok=True, on_done=None, delimiter=True):
        """Queue a command; it is sent once the previous one is answered."""
        self.queue.append((cmd, wait_ok, on_done, delimiter))
        self.send_next()

    def send_next(self):
        """Send queued commands until one has to wait for OK/ERROR."""
        while self.waiting is None and self.queue and self.serial is not None:
            cmd, wait_ok, on_done, delimiter = self.queue.pop(0)
            self.last_cmd = cmd
            if delimiter:
                self.console.emit(">> " + cmd + "\n")
                self.transmit(cmd.encode() + DELIMITER)
            else:
                self.console.emit(cmd + "\n")
                self.transmit(cmd.encode())
            if wait_ok:
                self.waiting = on_done if on_done is not None else (lambda ok: None)
                self.timer.start(self.timeout * 1000)

    def cmd_done(self, ok):
        """The command waiting for an answer got OK, ERROR or a timeout."""
        if self.waiting is None:
            return
        self.timer.stop()
        on_done = self.waiting
        self.waiting = None
        on_done(ok)
        self.send_next()

    def cmd_timeout(self):
        """No OK or ERROR within the timeout."""
        self.console.emit("TIMEOUT (did not receive OK)\n")
        self.cmd_done(False)

    def handle_ok(self, data):
        """Handle the 'OK' message."""
        self.cmd_done(True)

    def handle_error(self, data):
        """Handle the 'ERROR' message."""
        self.cmd_done(False)

    def handle_creg(self, data):
        """Handle the 'CREG' message (network registration)."""
        self.registered.emit(data == '001,001')

    def handle_cpin(self, data):
        """Handle the 'CPIN' message (SIM state); acted on after the OK."""
        self.cpin_state = data

    def serial_readable(self):
        """Read what the serial port has and handle it."""
        try:
            data = self.serial.read(self.serial.in_waiting or 1)
        except (serial.SerialException, OSError) as e:
            self.console.emit("[ERROR] %s\n" % e)
            self.close()
            return
        self.recv_data += data
        if self.in_call:
            self.handle_in_call()
        else:
            self.handle_in_terminal()

    def handle_in_call(self):
        """Handle data during a call (bytes, or the end of the call)."""
        text = self.recv_data.decode('latin-1').strip()
        # Check if we could have gotten an abort sequence
        if text != '' and len(self.recv_data) < 50:
            # Check for NO CARRIER or BUSY (wait for the full line)
            if NO_CARRIER.startswith(text) or BUSY.startswith(text):
                if self.recv_data.endswith(DELIMITER):
                    self.in_call = False
                    self.recv_data = b''
                    self.console.emit(text + "\n")
                    self.calling.emit(False)
                    self.call_dropped.emit()
                return

            # Check if we got a +CR: or CONNECT and ignore it
            if text.startswith("+CR:") or text.startswith("CONNECT"):
                if self.recv_data.endswith(DELIMITER):
                    self.console.emit(text + "\n")
                    self.recv_data = b''
                return

        # Handle the in call data
        if self.show_data:
            self.console.emit("< %s\n" % self.recv_data.hex())
        self.call_data.emit(self.recv_data)
        self.recv_data = b''

    def handle_in_terminal(self):
        """Handle every full line received in terminal mode."""
        while not self.in_call:
            new_line = self.recv_data.find(DELIMITER)
            if new_line < 0:
                return
            recv_line = self.recv_data[:new_line].decode('latin-1').strip()
            self.recv_data = self.recv_data[new_line + len(DELIMITER):]

            # Skip empty lines and the echo of our own command
            if recv_line == '':
                continue
            if self.echo_enabled and recv_line == self.last_cmd:
                continue

            # Split the information from the received line
            self.console.emit(recv_line + "\n")
            reg = re.match(r'([+]?)([A-Za-z0-9 ]+)[:]?[ ]?([A-Za-z0-9, ]*)', recv_line)
            if reg is None:
                continue
            command = reg.group(2).lower()
            data = reg.group(3).strip()

            # Dispatch the handler
            handler = getattr(self, "handle_%s" % command, None)
            if handler:
                handler(data)
