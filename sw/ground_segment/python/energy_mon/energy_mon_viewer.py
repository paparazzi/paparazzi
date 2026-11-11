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
Energy monitor window: battery cell state and flight time prediction.

Listens on Ivy to the ENERGY, TEMP_ADC and AIR_DATA telemetry messages and shows,
per battery cell (see battery_model.py for the pack layout), coloured status bars
(voltage, current, used charge, temperature, power, predicted hover and forward
flight time left) next to the cell discharge curves of batterymodel.csv.
"""

import sys
import os

from PyQt5 import QtCore, QtGui, QtWidgets

import battery_model as bat

PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface

WIDTH = 600
BARH = 140


def get_text_from_seconds(secs):
    """Seconds as mm:ss"""
    m, s = divmod(int(secs), int(60))
    return "{:02d}:{:02d}".format(m, s)

class AirDataMessage(object):
    """Fields used from AIR_DATA"""
    def __init__(self, msg):
        self.airspeed = float(msg['airspeed'])

class EnergyMessage(object):
    """Fields used from ENERGY (whole battery); energy is the used charge in mAh"""
    def __init__(self, msg):
        self.volt = float(msg['voltage'])
        self.current = float(msg['current'])
        self.power = float(msg['power'])
        self.energy = float(msg['charge']) * 1000.0

class TempMessage(object):
    """Fields used from TEMP_ADC (temp2 is the battery)"""
    def __init__(self, msg):
        self.motor = float(msg['temp1'])
        self.battery = float(msg['temp2'])

class BatteryCell(object):
    """State of one battery cell, with texts, bar fractions (0..1) and colours (0.1 red, 0.5 orange, 1 green)"""
    def __init__(self):
        self.voltage = 0
        self.current = 0
        self.energy = 0
        self.model = 0
        self.temperature = 0
    def fill_from_energy_msg(self, energy):
        self.voltage = energy.volt / bat.cells_in_series
        self.current = energy.current / bat.cells_in_parallel
        self.energy  = energy.energy / 6.0
        self.model = bat.mah_from_volt_and_current(self.voltage,self.current)
    def fill_from_temp_msg(self, temp):
        self.temperature = temp.battery

    def get_volt(self):
        return "Cell Volt = "+str(round(self.voltage,2)) + " V"
    def get_mah_from_volt(self):
        return "Cap(U,I) = "+str(round(self.model/1000.0,2)) + " Ah"
    def get_current(self):
        return "Cell Amps = "+str(round(self.current,2)) + " A"
    def get_energy(self):
        return "Cell mAh  = "+str(round(self.energy/1000.0 ,2)) + " Ah"
    def get_temp(self):
        return "Cell Temp = "+str(round(self.temperature ,2))
    def get_power_text(self):
        return "Battery Power: {:.0f}W".format(self.get_power() * bat.cells_in_battery)
    def get_volt_perc(self):
        return self.get_volt_percent(self.voltage)
    def get_volt_percent(self,volt):
        return (volt - 2.5) / (4.3 - 2.5)
    def get_power(self):
        return self.voltage * self.current
    def get_power_per_cell(self):
        return self.get_power() / bat.cells_in_parallel / bat.cells_in_series
    def get_temp_perc(self):
        return (self.temperature / 60)
    def get_current_perc(self):
        return (self.current / 10)
    def get_energy_perc(self):
        return (self.energy / bat.capacity)
    def get_model_perc(self):
        return (self.model / bat.capacity)
    def get_power_perc(self):
        return (self.get_power() - 200) / (800 - 200)

    def get_volt_color(self):
        if self.voltage < 3.2:
            return 0.1
        elif self.voltage < 3.6:
            return 0.5
        return 1

    def get_current_color(self):
        if self.current < 2.5:
            return 1
        elif self.current > 4.5:
            return 0.1
        return 0.5

    def get_energy_color(self):
        if self.energy > 3000:
            return 0.1
        elif self.energy < 2000:
            return 1
        return 0.5

    def get_temp_color(self):
        if (self.temperature > 20) & (self.temperature < 40):
            return 1
        elif (self.temperature > 10) & (self.temperature < 55):
            return 0.5
        return 0.1

    def get_power_color(self):
        return 0.5

class EnergyPrediction(object):
    """Flight time left from the cell state, the airspeed and the battery model"""
    coeffs_power_from_airspeed = [2.9229, -95.559, 1029.8] # Coefficients from matlab
    energy_land = 3  # Wh required to land
    energy_takeoff = 3  # Wh required for take-off
    charge_land = 0.3 * 1000 # mAh for battery
    power_hover = 700  # W required for hover per battery
    power_hover_cell = power_hover / bat.cells_in_battery
    min_allowable_voltage = 3.0
    expected_landing_time = 30

    def __init__(self, battery_cell):
        self.battery_cell = battery_cell
        self.airspeed = 0

    def fill_from_air_data_msg(self, air_data):
        self.airspeed = air_data.airspeed

    def get_expected_power_from_airspeed(self, airspeed):
        return sum(self.coeffs_power_from_airspeed[i] * airspeed ** (2 - i) for i in range(3))

    def get_expected_power(self):
        """Calculate expected power based on airspeed; if airspeed < 15, this model is invalid and hover power is assumed
        Power for whole battery"""
        if self.airspeed > 15:
            return self.get_expected_power_from_airspeed(self.airspeed)
        return self.power_hover

    def get_power_fraction(self):
        return self.battery_cell.get_power() * bat.cells_in_battery / self.get_expected_power()

    def get_power_fraction_text(self):
        return "Fraction: {:.2f}".format(self.get_power_fraction())

    def get_power_fraction_color(self):
        if self.get_power_fraction() > 1.2:
            return 0.1
        if self.get_power_fraction() > 1.0:
            return 0.5
        return 1

    def get_time_to_empty_battery_from_power(self, power):
        volt, amp = bat.volt_amp_from_mAh_power(self.battery_cell.model, power)
        if volt >= self.min_allowable_voltage:
            time_to_empty_battery, _ = bat.time_mAh_from_volt_to_volt_power(volt, self.min_allowable_voltage, power)
        else:
            time_to_empty_battery = 0
        return time_to_empty_battery

    def get_hover_seconds_color(self):
        if self.get_hover_seconds_left() > 90:
            return 1
        if self.get_hover_seconds_left() > 30:
            return 0.5
        return 0.1

    def get_hover_seconds_fraction(self):
        fraction = self.get_hover_seconds_left() / 120
        return min(1, fraction)

    def get_hover_seconds_left(self):
        return self.get_time_to_empty_battery_from_power(self.power_hover_cell)

    def get_hover_seconds_left_text(self):
        return "{} hover left".format(get_text_from_seconds(self.get_hover_seconds_left()))

    def get_fw_seconds_color(self):
        if self.get_fw_seconds_left() > 120:
            return 1
        if self.get_fw_seconds_left() > 60:
            return 0.5
        return 0.1

    def get_fw_seconds_fraction(self):
        fraction = self.get_fw_seconds_left() / 120
        return min(1, fraction)

    def get_fw_seconds_left(self):
        return self.get_time_to_empty_battery_from_power(self.get_expected_power() / bat.cells_in_battery)

    def get_fw_seconds_left_text(self):
        return "{} fw left".format(get_text_from_seconds(self.get_fw_seconds_left()))

    def get_fw_seconds_left_20mps(self):
        return self.get_time_to_empty_battery_from_power(self.get_expected_power_from_airspeed(20) / bat.cells_in_battery)

    def get_fw_seconds_left_20mps_text(self):
        return "{} left@20".format(get_text_from_seconds(self.get_fw_seconds_left_20mps()))

    def get_fw_seconds_left_20mps_fraction(self):
        fraction = self.get_fw_seconds_left_20mps() / 120
        return min(1, fraction)

    def get_fw_seconds_left_20mps_color(self):
        if self.get_fw_seconds_left_20mps() > 120:
            return 1
        if self.get_fw_seconds_left_20mps() > 60:
            return 0.5
        return 0.1


    def get_max_hover_charge(self):
        vmin = self.min_allowable_voltage
        Areq = self.power_hover / bat.cells_in_series / bat.cells_in_parallel / vmin
        mAh_vmin = bat.mah_from_volt_and_current(vmin, Areq)
        return mAh_vmin


class EnergyMonWindow(QtWidgets.QWidget):
    """Window drawing the status bars (left) and the discharge plot (right)"""

    # Ivy messages are handed from the Ivy thread to the GUI thread with this signal
    msg_received = QtCore.pyqtSignal(object)

    # Discharge curves of batterymodel.csv: 2, 4, 6, 8 and 10 A per cell
    CURVE_COLORS = [(0, 0, 0), (0, 0, 255), (0, 255, 0), (255, 255, 0), (255, 0, 0)]

    def __init__(self, ivy_bus=None):
        super().__init__()
        self.setWindowTitle('Energy Monitoring')
        self.setWindowIcon(QtGui.QIcon(PPRZ_SRC + "/sw/ground_segment/python/energy_mon/energy_mon.ico"))

        # Size and position of the last session
        self.settings = QtCore.QSettings('paparazzi', 'energymon_conf')
        self.resize(self.settings.value('size', QtCore.QSize(WIDTH, WIDTH + BARH)))
        if self.settings.contains('pos'):
            self.move(self.settings.value('pos'))

        self.cell = BatteryCell()
        self.energy_prediction = EnergyPrediction(self.cell)

        self.msg_received.connect(self.handle_message)
        if ivy_bus:
            self.interface = IvyMessagesInterface("energymonframe", ivy_bus=ivy_bus)
        else:
            self.interface = IvyMessagesInterface("energymonframe")
        self.interface.subscribe(self.message_recv, '^([^ ]* +(ENERGY|TEMP_ADC|AIR_DATA)( .*|$))')

    def message_recv(self, ac_id, msg):
        """Ivy thread: pass the message to the GUI thread"""
        self.msg_received.emit(msg)

    def handle_message(self, msg):
        """GUI thread: update the cell state and the prediction, then redraw"""
        if msg.name == "ENERGY":
            self.cell.fill_from_energy_msg(EnergyMessage(msg))
        elif msg.name == "TEMP_ADC":
            self.cell.fill_from_temp_msg(TempMessage(msg))
        elif msg.name == "AIR_DATA":
            self.energy_prediction.fill_from_air_data_msg(AirDataMessage(msg))
        self.update()

    def closeEvent(self, event):
        """Save the window geometry and stop Ivy"""
        self.settings.setValue('size', self.size())
        self.settings.setValue('pos', self.pos())
        self.interface.shutdown()
        super().closeEvent(event)

    def draw_text(self, p, x, y, txt):
        """Draw text with its top left corner at (x, y)"""
        p.drawText(x, y + p.fontMetrics().ascent(), txt)

    def status_box(self, p, nr, txt, percent, color):
        """Status bar number nr: grey box, coloured fill of percent (0..1), text"""
        percent = min(max(percent, 0), 1)
        tdx = int(self.stat * 10.0 / 300.0)
        tdy = int(self.stat * 6.0 / 300.0)
        boxh = int(self.stat * 40.0 / 300.0)
        boxw = self.stat - 2 * tdx
        top = int(nr * (boxh + 10) + tdx)

        p.setPen(QtGui.QColor(0, 0, 0))
        p.setBrush(QtGui.QColor(220, 220, 220))
        p.drawRect(tdx, top, boxw, boxh)
        text_color = QtGui.QColor(0, 0, 0)
        if color < 0.2:
            text_color = QtGui.QColor(255, 255, 255)
            p.setBrush(QtGui.QColor(250, 0, 0))
        elif color < 0.6:
            p.setBrush(QtGui.QColor(250, 180, 0))
        else:
            p.setBrush(QtGui.QColor(0, 250, 0))
        p.drawRect(tdx, top, int(boxw * percent), boxh)
        p.setPen(text_color)
        self.draw_text(p, 18, top + tdy, txt)

    def plot_x(self, x):
        """Plot x (0..1) to pixels"""
        return int(self.stat + self.tdx + x * (self.width() - self.stat - 2 * self.tdx))

    def plot_y(self, y):
        """Plot y (0..1) to pixels"""
        return int(self.tdx + (1 - y) * (self.height() - 2 * self.tdx))

    def plot_point(self, volt, mah):
        """Pixel position of a (cell voltage, mAh) point of the discharge plot"""
        return QtCore.QPoint(self.plot_x(mah / 3500), self.plot_y((volt - 2.5) / (4.2 - 2.5)))

    def vertical_line(self, p, x):
        """Vertical line over the whole plot at x (0..1)"""
        p.drawLine(self.plot_x(x), self.plot_y(0), self.plot_x(x), self.plot_y(1))

    def discharge_plot(self, p):
        """Discharge curves with the current cell state and charge limits"""
        self.tdx = int(self.stat * 10.0 / 300.0)
        p.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0), 1))
        p.setBrush(QtGui.QColor(250, 250, 250))
        p.drawRect(self.plot_x(0.0), self.plot_y(1.0), self.width() - self.stat - 2 * self.tdx,
                   self.height() - 2 * self.tdx)

        # Grid
        for i in range(0, 6):
            p.drawLine(self.plot_x(0.0), self.plot_y(i / 6.0), self.plot_x(1.0), self.plot_y(i / 6.0))
        for i in range(0, 7):
            self.vertical_line(p, i / 7.0)

        # Current cell state: charge used (model) and voltage
        p.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0), 4))
        self.vertical_line(p, self.cell.model / 3500)
        p.drawLine(self.plot_x(0.0), self.plot_y(self.cell.get_volt_perc()),
                   self.plot_x(1.0), self.plot_y(self.cell.get_volt_perc()))

        # Maximum charge for hover, and competition latest land at joe (18km/h0degwind, 25m/s)
        p.setPen(QtGui.QPen(QtGui.QColor(0, 0, 255), 2))
        self.vertical_line(p, self.energy_prediction.get_max_hover_charge() / 3500)
        self.vertical_line(p, 1415. / 3500)

        # Axis labels
        p.setPen(QtGui.QColor(0, 0, 0))
        p.setFont(QtGui.QFont("Serif", 8))
        for i in range(0, 3500, 500):
            self.draw_text(p, self.plot_x(float(i) / 3500.0), self.plot_y(1.0), str(i) + "mAh")
        for i in range(25, 43, 3):
            self.draw_text(p, self.plot_x(0), self.plot_y(self.cell.get_volt_percent(float(i / 10.0))),
                           str(round(i / 10.0, 1)) + "V")

        # Discharge curves
        for curve, color in enumerate(self.CURVE_COLORS):
            p.setPen(QtGui.QPen(QtGui.QColor(*color), 3))
            points = [self.plot_point(row[0], row[curve + 1]) for row in bat.batmodel]
            p.drawPolyline(QtGui.QPolygon(points))

    def paintEvent(self, event):
        """Draw the whole window, scaled to its size"""
        self.stat = max(int(self.width() / 4), 100)

        p = QtGui.QPainter(self)
        p.fillRect(self.rect(), QtGui.QColor(255, 255, 255))

        font = QtGui.QFont("Serif", max(int(self.width() * 11.0 / 800.0), 6))
        font.setBold(True)
        p.setFont(font)

        cell = self.cell
        pred = self.energy_prediction
        self.status_box(p, 0, cell.get_volt(), cell.get_volt_perc(), cell.get_volt_color())
        self.status_box(p, 1, cell.get_current(), cell.get_current_perc(), cell.get_current_color())
        self.status_box(p, 2, cell.get_energy(), cell.get_energy_perc(), cell.get_energy_color())
        self.status_box(p, 3, cell.get_mah_from_volt(), cell.get_energy_perc(), cell.get_energy_color())
        self.status_box(p, 4, cell.get_temp(), cell.get_temp_perc(), cell.get_temp_color())
        self.status_box(p, 6, cell.get_power_text(), cell.get_power_perc(), cell.get_power_color())
        self.status_box(p, 7, pred.get_power_fraction_text(), pred.get_power_fraction(),
                        pred.get_power_fraction_color())
        self.status_box(p, 8, pred.get_hover_seconds_left_text(), pred.get_hover_seconds_fraction(),
                        pred.get_hover_seconds_color())
        self.status_box(p, 9, pred.get_fw_seconds_left_text(), pred.get_fw_seconds_fraction(),
                        pred.get_fw_seconds_color())
        self.status_box(p, 10, pred.get_fw_seconds_left_20mps_text(), pred.get_fw_seconds_left_20mps_fraction(),
                        pred.get_fw_seconds_left_20mps_color())

        self.discharge_plot(p)
        p.end()


if __name__ == '__main__':
    # Quick check of the prediction without Ivy or GUI
    energy_message = EnergyMessage({"voltage": 22, "current": 18, "power": 22 * 18, "charge": 10.0})
    air_data_message = AirDataMessage({"airspeed": 21})
    cell = BatteryCell()
    cell.fill_from_energy_msg(energy_message)

    energy_prediction = EnergyPrediction(cell)
    energy_prediction.fill_from_air_data_msg(air_data_message)

    print(energy_prediction.get_expected_power())
    print(energy_prediction.get_hover_seconds_left_text())
    print(energy_prediction.get_fw_seconds_left_text())
    print(energy_prediction.get_fw_seconds_left_20mps_text())
