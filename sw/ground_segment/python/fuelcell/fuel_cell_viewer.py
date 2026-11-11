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
Nederdrone status window (PyQt5).

Draws a top view of the Nederdrone with coloured status bars for the fuel cell
(PAYLOAD message), the battery (ENERGY, TEMP_ADC) and every CAN ESC (ESC).
The data classes below turn the Ivy messages into texts, bar lengths (0..1)
and colours (below 0.2 red, below 0.6 orange, else green).
"""

import sys
import os
from datetime import datetime

from PyQt5 import QtCore, QtGui, QtWidgets

PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface

WIDTH = 600
BARH = 140


class AirDataMessage(object):
    """ AIR_DATA message: airspeed """
    def __init__(self, msg):
        self.airspeed = float(msg['airspeed'])

class EnergyMessage(object):
    """ ENERGY message: battery voltage, current, power and energy """
    def __init__(self, msg):
        self.volt = float(msg['voltage'])
        self.current = float(msg['current'])
        self.power = float(msg['power'])
        self.energy = float(msg['energy'])
        self.averagepower = float(msg['avg_power'])

class TempMessage(object):
    """ TEMP_ADC message: motor and battery temperature """
    def __init__(self, msg):
        self.motor = float(msg['temp1'])
        self.battery = float(msg['temp2'])

class EscMessage(object):
    """ ESC message of one CAN ESC, with texts, bars and colours for its status box """
    def __init__(self, msg):
        self.id = int(msg['node_id']) # motor_id
        self.amp = float(msg['amps'])
        self.rpm = float(msg['rpm'])
        self.volt_b = float(msg['bat_volts'])
        self.volt_m = float(msg['motor_volts'])
        self.temperature = float(msg['temperature']) - 273.5
        self.energy = float(msg['amps']) * float(msg['motor_volts'])
        self.errors = float(msg['energy'])
        self.msgtime = datetime.now()
    
    def get_current(self):
        return str(round(self.amp ,1)) + "A"
    def get_current_perc(self):
        return self.amp / 30

    def get_rpm(self):
        return str(round(self.rpm ,0)) + " rpm"
    def get_rpm_perc(self):
        return self.rpm / 7000
    def get_rpm_color(self):
        if self.rpm < 4800:
            return 1
        return 0.5

    def get_err(self):
        return str(self.errors) + " errs"
    def get_err_perc(self):
        return 0
    def get_err_color(self):
        return 0.5

    def get_volt(self):
        if (self.id in [10,17]):
            return "Servo " + str(self.id) + " " +str(round(self.volt_m ,1)) + "V"
        elif (self.id in [1,6]):
            return "Old " + str(self.id) + " " +str(round(self.volt_m ,1)) + "V"
        elif (self.id in [2,3,4,5,11,12,13,14,15,16]):
            return "New " + str(self.id) + " " +str(round(self.volt_m ,1)) + "V"
        else:
            return "? " + str(self.id) + " " +str(round(self.volt_m ,1)) + "V"
    def get_volt_perc(self):
        return self.volt_b / (6*4.2)

    def get_temp(self):
        if self.temperature < -200:
            return "xxx"
        return str(round(self.temperature ,1)) + "C"
    def get_temp_perc(self):
        return self.temperature / 120.0

    def get_temp_color(self):
        if self.temperature < 0:
            return 0
        elif self.temperature < 60:
            return 1
        elif self.temperature < 90:
            return 0.5
        else:
            return -1

    def get_timestamp(self, CPUTIME):
        now = datetime.now()
        return 'Age: '+str(round((now - self.msgtime).total_seconds(),1))+'s'

    def get_timestamp_color(self, CPUTIME):
        now = datetime.now()
        if ((now - self.msgtime).total_seconds()) > 30:
            return 0
        return 1

class MotorList(object):
    """ Last ESC message of every ESC, by node id """
    def __init__(self):
        self.mot = []

    def fill_from_esc_msg(self, esc):
        added = False
        for i in range(len(self.mot)):
            if self.mot[i].id == esc.id:
                self.mot[i] = esc
                added = True
                break
        if not added:
            self.mot.append(esc)


class BatteryCell(object):
    """ Battery state from ENERGY and TEMP_ADC, with texts, bars and colours """
    def __init__(self):
        self.voltage = 0
        self.current = 0
        self.power = 0
        self.averagepower = 0
        self.energy = 0
        self.model = 0
        self.temperature = 0
    def fill_from_energy_msg(self, energy):
        self.voltage = energy.volt
        self.current = energy.current
        self.power = energy.power
        self.energy  = energy.energy
        self.averagepower = energy.averagepower
        self.model = 0
    def fill_from_temp_msg(self, temp):
        self.temperature = temp.battery

    def get_volt(self):
        return "Volt = "+str(round(self.voltage,2)) + " V"

    def get_current(self):
        return "Amps = "+str(round(self.current,2)) + " A"
    def get_energy(self):
        return "Cell mAh  = "+str(round(self.energy/1000.0 ,2)) + " Ah"
    def get_temp(self):
        return "Cell Temp = "+str(round(self.temperature ,2))
    def get_power_text(self):
        return "Battery Power: {:.0f}W".format(self.get_power())
    def get_averagepower_text(self):
        return "Average Power: {:.0f}W".format(self.get_averagepower())
    def get_power2_text(self):
        return "Battery Power: {:.0f}W".format(self.get_power2())
    def get_volt_perc(self):
        return self.get_volt_percent(self.voltage)
    def get_volt_percent(self,volt):
        return (volt/6 - 2.5) / (4.3 - 2.5)

    def get_power(self):
        return self.power
    def get_averagepower(self):
        return self.averagepower
    def get_power2(self):
        return self.voltage * self.current

    def get_temp_perc(self):
        return (self.temperature / 60)
    def get_current_perc(self):
        return (self.current / 200)
    def get_energy_perc(self):
        return (self.energy / 4* 4500 + 30000)

    def get_power_perc(self):
        return (self.get_power()) / (2500)
    def get_averagepower_perc(self):
        return (self.get_averagepower()) / (2500)

    def get_volt_color(self):
        if self.voltage < 3.2:
            return 0.1
        elif self.voltage < 3.6:
            return 0.5
        return 1

    def get_current_color(self):
        if self.current < 150:
            return 1
        elif self.current > 200:
            return 0.1
        return 0.5
    
    def get_energy_color(self):
        if self.energy > 3000:
            return 0.1
        elif self.energy < 2000:
            return 1
        return 0.5

    def get_averagepower_color(self):
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
        if self.power < 800:
            return 1
        return 0.5


class PayloadMessage(object):
    """ PAYLOAD message: the bytes as a text string """
    def __init__(self, msg):
        self.values = ''.join(chr(int(x)) for x in msg['values'])



class FuelCellStatus(object):
    """ Fuel cell state from its PAYLOAD string "<tank%,battery%,status,0xerrors>" """
    def __init__(self):
        self.blink = 0
        self.msgtime = datetime.now()

    def update(self,msg):
        self.msg = msg
        self.msgtime = datetime.now()
        self.blink = 1 - self.blink
        elements = self.msg.strip('<').strip('>').split(',')
        if (len(elements) == 4):
            try:
                tank = float(elements[0])
                battery = float(elements[1])
                int(elements[2])
                hex = '0000'
                if (len(elements[3]) >= 4):
                    hex = elements[3][2:6]
                #array of 16 error codes
                error_bin = bin(int(hex, 16))[2:].zfill(16)
            except ValueError:
                # garbled string: keep the last good values (it is still shown raw)
                return
            self.tank = tank
            self.battery = battery
            self.status = elements[2]
            self.error = elements[3]
            self.error_bin = error_bin

    def get_age(self):
        now = datetime.now()
        return 'Age: '+str(round((now - self.msgtime).total_seconds(),1))+'s'
    def get_raw(self):
        return self.msg
    def get_raw_color(self):
        if self.blink == 0:
            return 0
        return 1.0

    def get_tank(self):
        bar = round(5 + self.tank / 100 * 295,1)
        return 'Cylinder ' + str(bar) + ' Bar ('+str(self.tank)+'%)'
    def get_tank_perc(self):
        return (self.tank) / 100.0
    def get_tank_color(self):
        if (self.tank) < 10:
            return 0.1
        elif int(self.tank) < 20:
            return 0.5
        else:
            return 1
                
    def get_battery(self):
        volt = round( self.battery / 100.0 * (24.0-19.6) + 19.6, 2)
        return str(volt) + ' V ('+str(self.battery)+'%)'
    def get_battery_perc(self):
        return (self.battery) / 100.0
    def get_battery_color(self):
        if (self.battery) < 10:
            return 0.1
        elif (self.battery) < 20:
            return 0.5
        else:
            return 1
                

    def get_status(self):
        if int(self.status) == 0:
            return 'Starting'
        elif int(self.status) == 1:
            return 'Ready (wait for button press)'
        elif int(self.status) == 2:
            return 'Running'
        elif int(self.status) == 3:
            return 'Fault'
        elif int(self.status) == 4:
            return 'Battery only'
        else:
            return self.status
    def get_status_perc(self):
        return 0.0
    def get_status_color(self):
        if int(self.status) < 1:
            return 0.1
        elif int(self.status) == 1:
            return 0.5
        else:
            return 0.5
                

    def get_error(self):
        return self.error

    def get_error_perc(self):
        return 1.0
    def get_error_color(self):
        if self.error[2:6] == '0000':
            return 1.0
        return 0.1


    def get_error_nr(self, nr):
        err = ["Stack1 OverTemp", "Stack2 OverTemp", "AuxBat UnderVolt", "AuxBat Overtemp", "No Fan", "", "Stack1 OverTemp", "Stack2 OverTemp", 
               "AuxBat UnderVolt", "AuxBat OverTemp", "Mast Timeout Start", "Master Timeout Stop", "Start Under Pressue", "Tank UnderPress", "Tank LowPress", "SafFlag bf Mastr En.", ""]
        return self.error_bin[nr] + ' ' + err[nr]
    def get_error_nr_color(self, nr):
        if self.error_bin[nr] == '1':
            return 0.1
        return 1

                




class FuelCellWidget(QtWidgets.QWidget):
    """ Window that draws the drone with all status boxes, updated from Ivy """

    # Ivy messages are passed from the Ivy thread to the GUI thread with this signal
    msg_sig = QtCore.pyqtSignal(object, object)

    def __init__(self, ivy_bus=None):
        super().__init__()
        self.setWindowTitle('Fuel Cell Monitoring')
        self.setWindowIcon(QtGui.QIcon(PPRZ_SRC + "/sw/ground_segment/python/energy_mon/energy_mon.ico"))

        # restore the last window size and position
        self.settings = QtCore.QSettings('paparazzi', 'fuel_cell')
        self.resize(WIDTH, WIDTH + BARH)
        if self.settings.contains('geometry'):
            self.restoreGeometry(self.settings.value('geometry'))

        self.CPUTIME = 0
        self.bat = {}
        self.temp = {}
        self.cell = BatteryCell()
        self.motors = MotorList()
        self.fuelcell = FuelCellStatus()
        self.fuelcell.update('<0,0,0,0x00000000>')

        # redraw every second so the ages keep counting when no messages arrive
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(1000)

        self.msg_sig.connect(self.handle_message)
        if ivy_bus is None:
            self.interface = IvyMessagesInterface("fuelcellframe")
        else:
            self.interface = IvyMessagesInterface("fuelcellframe", ivy_bus=ivy_bus)
        self.interface.subscribe(self.message_recv)

    def message_recv(self, ac_id, msg):
        """ Ivy thread: hand the message to the GUI thread """
        self.msg_sig.emit(ac_id, msg)

    def handle_message(self, ac_id, msg):
        """ GUI thread: store the message data and redraw """
        try:
            if msg.name == "ENERGY":
                self.bat = EnergyMessage(msg)
                self.cell.fill_from_energy_msg(self.bat)
            elif msg.name == "TEMP_ADC":
                self.temp = TempMessage(msg)
                self.cell.fill_from_temp_msg(self.temp)
            elif msg.name == "AIR_DATA":
                self.air_data = AirDataMessage(msg)
            elif msg.name == "ESC":
                self.esc = EscMessage(msg)
                self.motors.fill_from_esc_msg(self.esc)
            elif msg.name == "PAYLOAD":
                self.payload = PayloadMessage(msg)
                self.fuelcell.update(self.payload.values)
            elif msg.name == "ROTORCRAFT_STATUS":
                self.CPUTIME = float(msg['cpu_time'])
                return
            else:
                return
        except (KeyError, ValueError, TypeError) as e:
            print("Bad {} message: {}".format(msg.name, e))
            return
        self.update()

    def status_box(self, p, dx, dy, row, col, txt, percent, color):
        """ Draw one status box: grey background, coloured bar of length percent (0..1) and text """
        if percent < 0:
            percent = 0
        if percent > 1:
            percent = 1
        boxw = self.stat
        tdx = int(boxw * 10.0 / 300.0)
        tdy = int(boxw * 6.0 / 300.0)
        boxh = int(boxw * 45.0 / 300.0)
        boxw = self.stat - 2*tdx
        spacing = boxh+3

        p.setPen(QtGui.QColor(0, 0, 0))
        p.setBrush(QtGui.QColor(220, 220, 220))
        p.drawRect(tdx + col * 200 + dx, int(row*spacing+tdx) + dy, int(boxw), boxh)
        text_color = QtGui.QColor(0, 0, 0)
        if color < 0.2:
            text_color = QtGui.QColor(255, 255, 255)
            p.setBrush(QtGui.QColor(250, 0, 0))
        elif color < 0.6:
            p.setBrush(QtGui.QColor(250, 180, 0))
        else:
            p.setBrush(QtGui.QColor(0, 250, 0))
        p.drawRect(tdx + col * 200 + dx, int(row*spacing+tdx+dy), int(boxw * percent), boxh)
        # drawText takes the baseline, so add the font ascent to start at the top
        p.setPen(text_color)
        p.drawText(18 + col * 200 + dx, int(row*spacing+tdy+tdx+dy) + p.fontMetrics().ascent(), txt)

    def paintEvent(self, event):
        """ Draw the drone, the fuel cell, the battery, the fuel cell errors and the motors """
        # Automatic Scaling
        w = self.width()
        h = self.height()

        p = QtGui.QPainter(self)
        p.fillRect(self.rect(), QtGui.QColor(255, 255, 255))

        fontscale = int(w * 11.0 / 1500.0)
        if fontscale < 6:
            fontscale = 6
        p.setFont(QtGui.QFont("Serif", fontscale, QtGui.QFont.Bold))

        # Draw Drone
        p.setPen(QtGui.QColor(0, 0, 0))
        p.setBrush(QtGui.QColor(240, 240, 220))
        # Fuselage
        p.drawRoundedRect(int(0.37*w), int(0.05*h), int(0.26*w), int(0.9*h), int(0.1*w), int(0.1*w))
        # Front Wing
        p.drawRect(int(0.05*w), int(0.25*h), int(0.9*w), int(0.15*h))
        # Back Wing
        p.drawRect(int(0.01*w), int(0.65*h), int(0.98*w), int(0.15*h))

        # Fuel Cell
        p.setBrush(QtGui.QColor(100, 100, 100))
        p.drawRoundedRect(int(0.39*w), int(0.07*h), int(0.22*w), int(0.35*h), int(0.08*w), int(0.08*w))

        self.stat = int(0.14*w)

        # Battery
        dx = int(0.43*w)
        dy = int(0.43*h)
        self.status_box(p, dx, dy, 0, 0, self.cell.get_volt(), self.cell.get_volt_perc(), self.cell.get_volt_color())
        self.status_box(p, dx, dy, 1, 0, self.cell.get_current(), self.cell.get_current_perc(), self.cell.get_current_color())
        self.status_box(p, dx, dy, 2, 0, self.cell.get_power_text(), self.cell.get_power_perc(), self.cell.get_power_color())
        self.status_box(p, dx, dy, 3, 0, self.cell.get_energy(), self.cell.get_energy_perc(), self.cell.get_energy_color())
        self.status_box(p, dx, dy, 4, 0, self.cell.get_averagepower_text(), self.cell.get_averagepower_perc(), self.cell.get_averagepower_color())

        # Fuel cell
        dx = int(0.43*w)
        dy = int(0.15*h)
        self.status_box(p, dx, dy, 0, 0, self.fuelcell.get_raw(), 0, self.fuelcell.get_raw_color())
        self.status_box(p, dx, dy, 1, 0, self.fuelcell.get_tank(), self.fuelcell.get_tank_perc(), self.fuelcell.get_tank_color())
        self.status_box(p, dx, dy, 2, 0, self.fuelcell.get_battery(), self.fuelcell.get_battery_perc(), self.fuelcell.get_battery_color())
        self.status_box(p, dx, dy, 3, 0, self.fuelcell.get_status(), self.fuelcell.get_status_perc(), self.fuelcell.get_status_color())
        self.status_box(p, dx, dy, 4, 0, self.fuelcell.get_error(), self.fuelcell.get_error_perc(), self.fuelcell.get_error_color())
        self.status_box(p, dx, dy, 5, 0, self.fuelcell.get_age(), 0, 1)

        # Warnings
        self.stat = int(0.14*w)
        p.setBrush(QtGui.QColor(70, 70, 40))
        p.drawRect(int(0.36*w), int(0.63*h), int(0.28*w), int(0.35*h))
        dx = int(0.36*w)
        dy = int(0.63*h)
        for i in range(0, 8, 1):
            self.status_box(p, dx, dy, i, 0, self.fuelcell.get_error_nr(i*2), 1, self.fuelcell.get_error_nr_color(i*2))
            self.status_box(p, int(0.5*w), dy, i, 0, self.fuelcell.get_error_nr(i*2+1), 1, self.fuelcell.get_error_nr_color(i*2+1))

        # Motors: position of ESC node id 1..20 on the drawing (-100: not drawn)
        self.stat = int(0.10*w)

        p.setPen(QtGui.QColor(0, 0, 0))
        p.setBrush(QtGui.QColor(200, 200, 100))
        w1 = 0.20
        w2 = 0.60
        dw = 0.11
        mw = 0.1
        mm = [(0.03,w1), (0.03+dw,w1), (0.03+2*dw,w1), (0.97-mw-2*dw,w1), (0.97-mw-dw,w1), (0.97-mw,w1), (-100,0), (-100,0), (-100,0), (0.03,w2+0.19),
              (0.03,w2), (0.03+dw,w2), (0.03+2*dw,w2), (0.97-mw-2*dw,w2), (0.97-mw-dw,w2), (0.97-mw,w2), (0.97-mw,w2+0.19), (-100,0), (-100,0), (-100,0)]
        for m in mm:
            p.drawRect(int(m[0]*w), int(m[1]*h), int(mw*w), int(0.15*h))

        for m in self.motors.mot:
            if m.id < 1 or m.id > len(mm):
                continue  # node id without a place on the drawing
            mo_co = mm[m.id-1]
            dx = int(mo_co[0]*w)
            dy = int(mo_co[1]*h)
            self.status_box(p, dx, dy, 0, 0, m.get_volt(), m.get_volt_perc(), 1)
            self.status_box(p, dx, dy, 1, 0, m.get_current(), m.get_current_perc(), 1)
            self.status_box(p, dx, dy, 2, 0, m.get_rpm(), m.get_rpm_perc(), m.get_rpm_color())
            self.status_box(p, dx, dy, 3, 0, m.get_temp(), m.get_temp_perc(), m.get_temp_color())
            self.status_box(p, dx, dy, 4, 0, m.get_err(), m.get_err_perc(), m.get_err_color())
            self.status_box(p, dx, dy, 5, 0, m.get_timestamp(self.CPUTIME), m.get_err_perc(), m.get_timestamp_color(self.CPUTIME))

        p.end()

    def closeEvent(self, event):
        """ Save the window size and position and stop Ivy """
        self.settings.setValue('geometry', self.saveGeometry())
        self.timer.stop()
        self.interface.shutdown()
        event.accept()
