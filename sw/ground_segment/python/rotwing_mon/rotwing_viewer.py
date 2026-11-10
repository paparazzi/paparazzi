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
Window of the rotating wing status monitor (started by rotwing_mon.py).

The Ivy messages are decoded into small message classes in the Ivy thread, handed to the
GUI thread with a Qt signal and drawn with QPainter: the drone outline, a box per motor
(voltage, current, rpm, temperatures, INDI command), the rotating wing state (top left)
and the fuel cell (top right). The window size and position are remembered.
"""

import sys
import os
import datetime
import queue
import threading

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

sys.path.append(PPRZ_HOME + "/var/lib/python")  # pprzlink

from pprzlink.ivy import IvyMessagesInterface

WIDTH = 600
BARH = 140

BLACK = QtGui.QColor(0, 0, 0)
WHITE = QtGui.QColor(255, 255, 255)
RED = QtGui.QColor(255, 0, 0)
BROWN = QtGui.QColor(139, 64, 0)


class Speech(object):
    """Optional spoken warnings with pyttsx3, in a thread of their own so the window never blocks."""

    def __init__(self):
        import pyttsx3  # raises ImportError when not installed, handled by the caller
        self.pyttsx3 = pyttsx3
        self.last_speech_time = datetime.datetime.now()
        self.texts = queue.Queue()
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        """Speak the queued texts one by one (the engine lives in this thread)."""
        engine = self.pyttsx3.init()
        while True:
            engine.say(self.texts.get())
            engine.runAndWait()

    def speak_every_interval(self, text, interval):
        """Say text, but not more often than once every interval seconds."""
        if (datetime.datetime.now() - self.last_speech_time).total_seconds() > interval:
            self.last_speech_time = datetime.datetime.now()
            self.texts.put(text)


class EscMessage(object):
    """ESC message of one motor or servo, with the texts, bar fractions and colours to draw."""

    def __init__(self, msg):
        self.id = int(msg['motor_id'])
        self.node_id = int(msg['node_id'])
        self.amp = float(msg['amps'])
        self.rpm = float(msg['rpm'])
        self.volt_b = float(msg['bat_volts'])
        self.volt_m = float(msg['motor_volts'])
        self.temperature = float(msg['temperature'])
        self.temperature_dev = float(msg['temperature_dev'])
        self.energy = float(msg['energy'])

    def get_current(self):
        return str(round(self.amp, 1)) + "A"

    def get_current_perc(self):
        return self.amp / 30

    def get_rpm(self):
        return str(round(self.rpm, 0)) + " rpm"

    def get_rpm_perc(self):
        return self.rpm / 5150

    def get_rpm_color(self):
        if self.rpm < 4000:
            return 1
        return 0.5

    def get_volt(self):
        if self.id in [6, 7, 8, 9, 16, 17, 18, 19]:
            return "Servo " + str(self.id) + " " + str(round(self.volt_m, 1)) + "V"
        else:
            return "Mot " + str(self.id) + " " + str(round(self.volt_m, 1)) + "V"

    def get_volt_perc(self):
        return self.volt_b / (6 * 4.2)

    def get_temp(self):
        return str(round(self.temperature, 0)) + "C"

    def get_temp_perc(self):
        return self.temperature / 120.0

    def get_temp_dev(self):
        return str(round(self.temperature_dev, 0)) + "C"

    def get_temp_dev_perc(self):
        return self.temperature_dev / 120.0

    def get_temp_color(self):
        if self.temperature < 0:
            return 0
        elif self.temperature < 60:
            return 1
        else:
            return 0.5

    def get_temp_dev_color(self):
        if self.temperature_dev < 0:
            return 0
        elif self.temperature_dev < 100:
            return 1
        else:
            return 0.5


class INDIMessage(object):
    """Actuator commands (u) of the STAB_ATTITUDE message."""

    def __init__(self, msg):
        self.u = np.array(msg['u'], dtype=float)

    def get_u(self, id):
        return str(round(self.u[id], 0)) + " PPRZ"

    def get_len(self):
        return len(self.u)

    def get_u_perc(self, id):
        return self.u[id] / 9600.

    def get_u_color(self, id):
        if self.u[id] < 9600 / 0.8:
            return 1
        else:
            return 0.5


class RWStatusMessage(object):
    """ROTATING_WING_STATE message, with the status bitfield unpacked."""

    STATES = ['FORCE_HOVER', 'REQ_HOVER', 'FORCE_FW', 'REQ_FW', 'FREE']

    def __init__(self, msg):
        self.state = int(msg['state'])
        self.nav_state = int(msg['state'])
        self.status = int(msg['status'])
        self.meas_skew_angle = float(msg['meas_skew_angle'])
        self.sp_skew_angle = float(msg['sp_skew_angle'])
        self.nav_airspeed = float(msg['nav_airspeed'])
        self.min_airspeed = float(msg['min_airspeed'])
        self.max_airspeed = float(msg['max_airspeed'])

        # Unpack the status bitfields
        self.skew_angle_valid = bool(self.status & (0x1 << 0))
        self.hover_motors_enabled = bool(self.status & (0x1 << 1))
        self.hover_motors_idle = bool(self.status & (0x1 << 2))
        self.hover_motors_running = bool(self.status & (0x1 << 3))
        self.pusher_motor_running = bool(self.status & (0x1 << 4))
        self.skew_forced = bool(self.status & (0x1 << 5))

    def get_state(self):
        return self.STATES[self.state]

    def get_nav_state(self):
        return self.STATES[self.nav_state]


class AIRDATAMessage(object):
    """Measured and true airspeed of the AIR_DATA message."""

    def __init__(self, msg):
        self.airspeed = float(msg['airspeed'])
        self.tas = float(msg['tas'])


class INFOMSGMessage(object):
    """INFO_MSG text; 'ASC: ... <ratio>' gives the MS45XX in-flight airspeed calibration ratio."""

    def __init__(self, msg):
        words = ''.join(msg['msg']).replace('\"', '').split(" ")
        if words[0] == 'ASC:':
            self.ratio = float(words[-1])


class IMUHEATERMessage(object):
    """Measured IMU temperature of the IMU_HEATER message."""

    def __init__(self, msg):
        self.meas_temp = msg['meas_temp']


class POWERDEVICEMessage(object):
    """POWER_DEVICE message: current and voltage of one circuit of one node."""

    def __init__(self, msg):
        self.node_id = int(msg['node_id'])
        self.circuit = int(msg['circuit'])
        self.current = float(msg['current'])
        self.voltage = float(msg['voltage'])


class FUELCELLMessage(object):
    """FUELCELL message, with the state, error and sub-error codes as text."""

    STATES = ['FCPM off', 'Starting', 'Running', 'Stopping', 'Go to sleep']

    ERRORS = {
        0: 'No error',
        1: 'Minor internal',
        4: 'Reconditioning needed',
        6: 'Auto recondition active',
        7: 'Reconditioning active',
        8: 'Recovery mode active',
        12: 'Tank pressure low',
        14: 'Battery voltage low',
        20: 'Reduced power',
        25: 'SPM lost',
        30: 'Tank empty',
        32: 'Start denied',
        34: 'FC off - recovery required',
        36: 'FC off - system error',
        40: 'Battery critical',
    }

    SUBERRORS = {
        0: 'No context',
        1: 'SPM1',
        2: 'SPM2',
        3: 'SPM1 AND SPM2',
        4: 'ALL SPM LOST',
        5: 'Inlet pressure low',
        6: 'Inlet pressure high',
        7: 'Tank pressure low',
        8: 'Tank pressure high',
        9: 'Stack voltage low',
        10: 'Internal tests (INT ERR)',
        11: 'Stack temperature high (STK HT)',
        12: 'Stack temperature low (STK LT)',
        13: 'Reconditioning active',
        14: 'Reconditioning complete',
        15: 'Thermal management',
        16: 'Start faults',
        17: 'LPI missing',
        254: 'Complete flight - contact support',
        255: 'Stop using unit - contact support',
    }

    def __init__(self, msg):
        self.pressure = msg['pressure']
        self.press_reg = float(msg['press_reg'])
        self.volt_bat = float(msg['volt_bat'])
        self.power_out = float(msg['power_out'])
        self.power_cell = float(msg['power_cell'])
        self.power_batt = float(msg['power_batt'])
        self.state = int(msg['state'])
        self.error = int(msg['error'])
        self.suberror = int(msg['suberror'])
        self.state_str = '?'
        if self.state < len(self.STATES):
            self.state_str = self.STATES[self.state]
        self.error_str = self.ERRORS.get(self.error, 'Unknown error')
        self.suberror_str = self.SUBERRORS.get(self.suberror, 'Unknown suberror')


class PowerList(object):
    """Last POWER_DEVICE message of every (node, circuit)."""

    def __init__(self):
        self.power = []

    def fill_from_power_msg(self, power):
        for i in range(len(self.power)):
            if self.power[i].node_id == power.node_id and self.power[i].circuit == power.circuit:
                self.power[i] = power
                return
        self.power.append(power)

    def get(self, node_id, circuit):
        for p in self.power:
            if p.node_id == node_id and p.circuit == circuit:
                return p
        return None

    def get_frontbat(self):
        return self.get(14, 0)

    def get_backbat(self):
        return self.get(6, 0)

    def get_backmot(self):
        return self.get(6, 1)


class MotorList(object):
    """Last ESC message of every motor id."""

    def __init__(self):
        self.mot = []

    def fill_from_esc_msg(self, esc):
        # Some corrections for the big one:
        if esc.id == 13:
            esc.id = 1
        if esc.temperature < -50:
            esc.temperature += 273.15
        if esc.temperature_dev < -50:
            esc.temperature_dev += 273.15
        for i in range(len(self.mot)):
            if self.mot[i].id == esc.id:
                self.mot[i] = esc
                return
        self.mot.append(esc)


# Ivy message name -> class that decodes it
MESSAGE_CLASSES = {
    "ESC": EscMessage,
    "STAB_ATTITUDE": INDIMessage,
    "ROTATING_WING_STATE": RWStatusMessage,
    "AIR_DATA": AIRDATAMessage,
    "INFO_MSG": INFOMSGMessage,
    "IMU_HEATER": IMUHEATERMessage,
    "FUELCELL": FUELCELLMessage,
    "POWER_DEVICE": POWERDEVICEMessage,
}

# Position (fraction of width, fraction of height) of the motor boxes, in the order of MOT_IDS
MOTOR_POS_W1 = 0.03
MOTOR_POS_W2 = 0.18
MOTOR_POS_W3 = 0.31
MOTOR_POS_W4 = 0.52
MOTOR_DW = 0.17
MOTOR_MW = 0.1   # box width
MOTOR_MH = 0.2   # box height
MOTOR_POS = [
    (0.5-0.5*MOTOR_MW-0.05, MOTOR_POS_W1), (0.5+MOTOR_DW+0.05+0.1, MOTOR_POS_W2),  # hover CAN1: front right
    (0.5-0.5*MOTOR_MW-0.05, MOTOR_POS_W3), (0.5-MOTOR_DW-MOTOR_MW+0.05-0.1, MOTOR_POS_W2),  # back left
    (0.5-0.5*MOTOR_MW-0.05, MOTOR_POS_W4),  # pusher CAN1
    (0.5-MOTOR_DW-MOTOR_MW-0.05, MOTOR_POS_W4+0.05), (0.5+MOTOR_DW-0.05, MOTOR_POS_W4+0.05),  # tail CAN1
    (0.5-0.5*MOTOR_MW+0.05, MOTOR_POS_W1), (0.5+MOTOR_DW-0.05+0.1, MOTOR_POS_W2),  # hover CAN2
    (0.5-0.5*MOTOR_MW+0.05, MOTOR_POS_W3), (0.5-MOTOR_DW-MOTOR_MW-0.05-0.1, MOTOR_POS_W2),
    (0.5-0.5*MOTOR_MW+0.05, MOTOR_POS_W4),  # pusher CAN2
    (0.5-MOTOR_DW-MOTOR_MW+0.05, MOTOR_POS_W4+0.05), (0.5+MOTOR_DW+0.05, MOTOR_POS_W4+0.05),  # tail CAN2
    (0.0, 0.75), (0.1, 0.75), (0.2, 0.75), (0.3, 0.75), (0.4, 0.75),  # extra
    (0.5, 0.75), (0.6, 0.75), (0.7, 0.75), (0.8, 0.75), (0.9, 0.75),
]
MOT_IDS = [
    0, 1, 2, 3,         # hover CAN1
    4,                  # pusher CAN1
    6, 7,               # elevator/rudder CAN1
    12, 13, 14, 15,     # hover CAN2
    16,                 # pusher CAN2
    18, 19,             # tail CAN2
    5, 8, 9, 10, 11,    # Extra's at the bottoms
    17, 20, 21, 22, 23
]


class RotWingWindow(QtWidgets.QWidget):
    """Main window: receives the Ivy messages and paints the whole status."""

    msg_sig = QtCore.pyqtSignal(str, object)  # message name, decoded message

    def __init__(self, args):
        super().__init__()
        self.setWindowTitle('Rot Wing Monitoring')
        self.setObjectName('RotWingFrame')

        self.settings = QtCore.QSettings('paparazzi', 'rotwing_mon')
        if not self.restoreGeometry(self.settings.value('geometry', QtCore.QByteArray())):
            self.resize(WIDTH, WIDTH + BARH)

        # Last decoded message of each type (None until received)
        self.motors = MotorList()
        self.powers = PowerList()
        self.indi = None
        self.rw_status = None
        self.air_data = None
        self.infomsg = None
        self.imu_heater = None
        self.fuelcell = None

        self.speech = None
        if args.speech:
            try:
                self.speech = Speech()
            except ImportError:
                print("Speech disabled: pyttsx3 is not installed (pip install pyttsx3)")

        self.msg_sig.connect(self.store_message)
        self.interface = IvyMessagesInterface("rotwingframe")
        self.interface.subscribe(self.message_recv)

    def message_recv(self, ac_id, msg):
        """Ivy thread: decode the messages we use and pass them to the GUI thread."""
        if msg.name in MESSAGE_CLASSES:
            self.msg_sig.emit(msg.name, MESSAGE_CLASSES[msg.name](msg))

    def store_message(self, name, data):
        """GUI thread: keep the new message and repaint."""
        if name == "ESC":
            self.motors.fill_from_esc_msg(data)
        elif name == "POWER_DEVICE":
            self.powers.fill_from_power_msg(data)
        elif name == "STAB_ATTITUDE":
            self.indi = data
        elif name == "ROTATING_WING_STATE":
            self.rw_status = data
        elif name == "AIR_DATA":
            self.air_data = data
        elif name == "INFO_MSG":
            if hasattr(data, 'ratio'):  # other info messages do not hide the airspeed ratio
                self.infomsg = data
                self.speak_airspeed_ratio()
        elif name == "IMU_HEATER":
            self.imu_heater = data
        elif name == "FUELCELL":
            self.fuelcell = data
        self.update()

    def speak_airspeed_ratio(self):
        """Spoken warning when the airspeed calibration ratio is off (only with speech on)."""
        if self.speech is None or self.rw_status is None:
            return
        error = abs(1 - self.infomsg.ratio)
        if error >= 0.15:
            self.speech.speak_every_interval("Airspeed is off by more than 15%, Please Calibrate", 5)
        elif error >= 0.08:
            self.speech.speak_every_interval("Airspeed is off by more than 8%, Please Calibrate", 30)

    def closeEvent(self, event):
        """Remember the window size and position, and stop Ivy."""
        self.settings.setValue('geometry', self.saveGeometry())
        self.interface.shutdown()
        super().closeEvent(event)

    def draw_text(self, p, x, y, txt, color=BLACK):
        """Draw txt with its top left corner at (x, y) and return its width."""
        p.save()
        p.setPen(color)
        p.drawText(int(x), int(y) + p.fontMetrics().ascent(), txt)
        p.restore()
        return p.fontMetrics().horizontalAdvance(txt)

    def status_box(self, p, dx, dy, row, txt, percent, color):
        """Draw one bar of a motor box: grey background, coloured fraction and text."""
        percent = min(max(percent, 0), 1)
        tdx = int(self.stat * 10.0 / 300.0)
        tdy = int(self.stat * 6.0 / 300.0)
        boxh = int(self.stat * 40.0 / 300.0)
        boxw = self.stat - 2 * tdx
        y = int(row * (boxh + 10) + tdx + dy)

        p.setPen(BLACK)
        p.setBrush(QtGui.QColor(220, 220, 220))
        p.drawRect(tdx + dx, y, int(boxw), boxh)
        text_color = BLACK
        if color < 0.2:
            text_color = WHITE
            p.setBrush(QtGui.QColor(250, 0, 0))
        elif color < 0.6:
            p.setBrush(QtGui.QColor(250, 180, 0))
        else:
            p.setBrush(QtGui.QColor(0, 250, 0))
        p.drawRect(tdx + dx, y, int(boxw * percent), boxh)
        self.draw_text(p, 18 + dx, y + tdy, txt, text_color)

    def paintEvent(self, event):
        """Draw the drone, the rotating wing status, the fuel cell and the motors."""
        w = self.width()
        h = self.height()
        self.stat = int(0.10 * w)

        p = QtGui.QPainter(self)
        p.fillRect(self.rect(), WHITE)
        fontscale = max(int(w * 11.0 / 1500.0), 6)
        p.setFont(QtGui.QFont("Serif", fontscale, QtGui.QFont.Bold))
        p.setPen(BLACK)

        # Draw Drone
        p.setBrush(QtGui.QColor(240, 240, 220))
        # Fuselage
        p.drawRoundedRect(int(0.475*w), int(0.05*h), int(0.05*w), int(0.75*h), int(0.015*w), int(0.015*w))
        p.drawRoundedRect(int(0.425*w), int(0.05*h), int(0.15*w), int(0.4*h), int(0.05*w), int(0.05*w))
        # Front Wing
        p.drawRect(int(0.05*w), int(0.25*h), int(0.9*w), int(0.15*h))
        # Back Wing
        p.drawRect(int(0.25*w), int(0.65*h), int(0.5*w), int(0.1*h))

        line = int(20.0 / 800.0 * h)
        if self.rw_status is not None:
            self.paint_rw_status(p, w, h, line)
        if self.fuelcell is not None:
            self.paint_fuelcell(p, w, h, line)
        self.paint_motors(p, w, h)
        p.end()

    def paint_rw_status(self, p, w, h, line):
        """Top left box: wing state, rotation, motors, airspeed, IMU temperature, power."""
        rw = self.rw_status
        p.setBrush(QtGui.QColor(200, 200, 100))
        p.drawRect(5, 5, int(0.29*w), int(0.20*h))
        self.draw_text(p, 10, 0.5*line, "State: " + rw.get_state() + " [NAV: " + rw.get_nav_state() + "]")

        color = RED
        if rw.skew_angle_valid:
            color = BLACK if abs(rw.meas_skew_angle - rw.sp_skew_angle) < 10 else BROWN
        self.draw_text(p, 10, 1.5*line, "Rotation: " + str(round(rw.meas_skew_angle, 1)) +
                       " (SP: " + str(round(rw.sp_skew_angle, 1)) + ")", color)

        x = 10 + self.draw_text(p, 10, 2.5*line, "Hover motors: ")
        if rw.hover_motors_running:
            x += self.draw_text(p, x, 2.5*line, "running ")
        else:
            x += self.draw_text(p, x, 2.5*line, "stopped ", RED)
        if rw.hover_motors_idle:
            x += self.draw_text(p, x, 2.5*line, "idle ")
        if rw.hover_motors_enabled:
            self.draw_text(p, x, 2.5*line, "(Enabled)")
        else:
            self.draw_text(p, x, 2.5*line, "(Disabled)", RED)

        x = 10 + self.draw_text(p, 10, 3.5*line, "Pusher motor: ")
        if rw.pusher_motor_running:
            self.draw_text(p, x, 3.5*line, "running")
        else:
            self.draw_text(p, x, 3.5*line, "stopped", RED)

        self.draw_text(p, 10, 4.5*line, "Nav airspeed: " + str(round(rw.nav_airspeed, 1)) +
                       " [min: " + str(round(rw.min_airspeed, 1)) + ", max:" + str(round(rw.max_airspeed, 1)) + "]")
        if self.air_data is not None:
            self.draw_text(p, 10, 5.5*line, "Meas airspeed: " + str(round(self.air_data.airspeed, 1)) +
                           " (TAS: " + str(round(self.air_data.tas, 1)) + ")")

        if self.infomsg is not None:
            # MS45XX inflight calibration monitoring
            error = abs(1 - self.infomsg.ratio)
            color = BLACK if error < 0.08 else (BROWN if error < 0.15 else RED)
            self.draw_text(p, 10, 6.5*line, "Airspeed ratio: " + str(round(self.infomsg.ratio, 2)), color)

        x = 10 + self.draw_text(p, 10, 7.5*line, "Force Skew: ")
        if rw.skew_forced:
            self.draw_text(p, x, 7.5*line, "(Enabled)", RED)
        else:
            self.draw_text(p, x, 7.5*line, "(Disabled)")

        if self.imu_heater is not None:
            imu_temp = float(self.imu_heater.meas_temp)
            color = BLACK if imu_temp < 65.0 else (BROWN if imu_temp < 85.0 else RED)
            self.draw_text(p, 10, 8.5*line, "Meas IMU temp: " + str(round(imu_temp, 0)), color)

        for row, label, power in [(9.5, "Back Bat: ", self.powers.get_backbat()),
                                  (10.5, "Front Bat: ", self.powers.get_frontbat()),
                                  (11.5, "Back Mot: ", self.powers.get_backmot())]:
            if power is not None:
                self.draw_text(p, 10, row*line, label + str(round(power.current * power.voltage, 1)) + "W (" +
                               str(round(power.voltage, 1)) + "V, " + str(round(power.current, 1)) + "A)")

    def paint_fuelcell(self, p, w, h, line):
        """Top right box: fuel cell state, pressures, powers and errors."""
        fc = self.fuelcell
        x = int(0.75*w + 5)
        p.setBrush(QtGui.QColor(200, 200, 200))
        p.drawRoundedRect(int(0.7*w), 5, int(0.29*w), int(0.18*h), int(0.05*h), int(0.05*h))
        self.draw_text(p, x, 0.5*line, "FuelCell state: [" + str(fc.state) + "] " + fc.state_str)
        self.draw_text(p, x, 1.5*line, "Cylinder press: " + str(fc.pressure) + "%  Reg=" + str(fc.press_reg) + "Bar")
        self.draw_text(p, x, 2.5*line, "Output: " + str(fc.power_out) + " Watt")
        self.draw_text(p, x, 3.5*line, "SPM: " + str(fc.power_cell) + " Watt, p=" +
                       str(round(float(fc.pressure)*3.630, 1)) + " Bar")
        self.draw_text(p, x, 4.5*line, "Battery: " + str(fc.power_batt) + " Watt, " + str(fc.volt_bat) + " Volt")
        self.draw_text(p, x, 5.5*line, "Error: [" + str(fc.error) + "] " + fc.error_str)
        self.draw_text(p, x, 6.5*line, "Sub-error: [" + str(fc.suberror) + "] " + fc.suberror_str)

    def paint_motors(self, p, w, h):
        """One box per motor with voltage, current, rpm, temperatures and INDI command."""
        for m in self.motors.mot:
            if m.id not in MOT_IDS:
                continue
            draw_id = MOT_IDS.index(m.id)
            if draw_id >= len(MOTOR_POS):
                continue
            pos = MOTOR_POS[draw_id]
            dx = int(pos[0]*w)
            dy = int(pos[1]*h)

            p.setPen(BLACK)
            p.setBrush(QtGui.QColor(200, 200, 100))
            p.drawRect(int((pos[0]+0.001)*w), int((pos[1]+0.001)*h), int((MOTOR_MW-0.002)*w), int((MOTOR_MH-0.002)*h))

            self.status_box(p, dx, dy, 0, m.get_volt(), m.get_volt_perc(), 1)
            self.status_box(p, dx, dy, 1, m.get_current(), m.get_current_perc(), 1)
            self.status_box(p, dx, dy, 2, m.get_rpm(), m.get_rpm_perc(), m.get_rpm_color())
            self.status_box(p, dx, dy, 3, m.get_temp(), m.get_temp_perc(), m.get_temp_color())
            self.status_box(p, dx, dy, 4, m.get_temp_dev(), m.get_temp_dev_perc(), m.get_temp_dev_color())

            # The pusher (motor 4) is actuator 8 in the INDI commands
            indi_id = 8 if m.id == 4 else m.id
            if self.indi is not None and indi_id < self.indi.get_len():
                self.status_box(p, dx, dy, 5, self.indi.get_u(indi_id), self.indi.get_u_perc(indi_id),
                                self.indi.get_u_color(indi_id))
