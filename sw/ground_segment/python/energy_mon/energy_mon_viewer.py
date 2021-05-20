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

import wx

import sys
import os
import math
import datetime

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
    m, s = divmod(int(secs), int(60))
    return "{:02d}:{:02d}".format(m, s)

class AirDataMessage(object):
    def __init__(self, msg):
        self.airspeed = float(msg['airspeed'])

class EnergyMessage(object):
    def __init__(self, msg):
        self.volt = float(msg['bat'])
        self.current = float(msg['amp'])
        self.power = float(msg['power'])
        self.energy = float(msg['energy'])

class TempMessage(object):
    def __init__(self, msg):
        self.motor = float(msg['temp1'])
        self.battery = float(msg['temp2'])

class BatteryCell(object):
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


class EnergyMonFrame(wx.Frame):
    def message_recv(self, ac_id, msg):
        if msg.name == "ENERGY":
            self.bat = EnergyMessage(msg)
            self.cell.fill_from_energy_msg(self.bat)

            wx.CallAfter(self.update)
        elif msg.name == "TEMP_ADC":
            self.temp = TempMessage(msg)
            self.cell.fill_from_temp_msg(self.temp)
            wx.CallAfter(self.update)

        elif msg.name == "AIR_DATA":
            self.air_data = AirDataMessage(msg)
            self.energy_prediction.fill_from_air_data_msg(self.air_data)
            wx.CallAfter(self.update)    

    def update(self):
        self.Refresh()

    def OnSize(self, event):
        self.w = event.GetSize().x
        self.h = event.GetSize().y
        self.cfg.Write("width", str(self.w));
        self.cfg.Write("height", str(self.h));
        self.Refresh()

    def OnMove(self, event):
        self.x = event.GetPosition().x
        self.y = event.GetPosition().y
        self.cfg.Write("left", str(self.x));
        self.cfg.Write("top", str(self.y));


    def StatusBox(self, dc, nr, txt, percent, color):
        if percent < 0:
            percent = 0
        if percent > 1:
            percent = 1
        boxw = self.stat
        tdx = int(boxw * 10.0 / 300.0)
        tdy = int(boxw * 6.0 / 300.0)
        boxh = int(boxw * 40.0 / 300.0)
        boxw = self.stat - 2*tdx
        spacing = boxh+10

        dc.SetPen(wx.Pen(wx.Colour(0,0,0))) 
        dc.SetBrush(wx.Brush(wx.Colour(220,220,220))) 
        dc.DrawRectangle(tdx, int(nr*spacing+tdx), int(boxw), boxh)
        dc.SetTextForeground(wx.Colour(0, 0, 0))
        if color < 0.2:
            dc.SetTextForeground(wx.Colour(255, 255, 255))
            dc.SetBrush(wx.Brush(wx.Colour(250,0,0)))
        elif color < 0.6:
            dc.SetBrush(wx.Brush(wx.Colour(250,180,0)))
        else:
            dc.SetBrush(wx.Brush(wx.Colour(0,250,0)))
#        dc.DrawLine(200,50,350,50)
        dc.DrawRectangle(tdx, int(nr*spacing+tdx), int(boxw * percent), boxh)
        dc.DrawText(txt,18,int(nr*spacing+tdy+tdx))

    def plot_x(self, x):
        return int(self.stat+self.tdx +  x * (self.w-self.stat-2*self.tdx))

    def plot_y(self, y):
        return int(self.tdx + (1-y) * (self.h-self.tdx-self.tdx))

    def plot(self, dc, i1, i2):
        dc.DrawLine(self.plot_x(i1[1]/3500), self.plot_y((i1[0]-2.5)/(4.2-2.5)), self.plot_x(i2[1]/3500), self.plot_y((i2[0]-2.5)/(4.2-2.5)))

    def DischargePlot(self, dc):
        self.tdx = int(self.stat * 10.0 / 300.0)
        dc.SetPen(wx.Pen(wx.Colour(0,0,0),1))
        dc.SetBrush(wx.Brush(wx.Colour(250,250,250))) 
        dc.DrawRectangle(self.plot_x(0.0), self.plot_y(1.0), self.w-self.stat-2*self.tdx, self.h-2*self.tdx)
        
        for i in range(0,6):
            dc.DrawLine(self.plot_x(0.0), self.plot_y(i/6.0), self.plot_x(1.0), self.plot_y(i/6.0))
        for i in range(0,7):
            dc.DrawLine(self.plot_x(i/7.0), self.plot_y(0), self.plot_x(i/7.0), self.plot_y(1))

        
        dc.SetPen(wx.Pen(wx.Colour(0,0,0),4))
        dc.DrawLine(self.plot_x(self.cell.model/3500), self.plot_y(0), self.plot_x(self.cell.model/3500), self.plot_y(1))
        dc.DrawLine(self.plot_x(0.0), self.plot_y(self.cell.get_volt_perc()), self.plot_x(1.0), self.plot_y(self.cell.get_volt_perc()))

        # Draw maximum charge point
        dc.SetPen(wx.Pen(wx.Colour(0,0,255),2))
        dc.DrawLine(self.plot_x(self.energy_prediction.get_max_hover_charge() / 3500), self.plot_y(0), self.plot_x(self.energy_prediction.get_max_hover_charge() / 3500), self.plot_y(1))
        dc.DrawLine(self.plot_x(1415. / 3500), self.plot_y(0), self.plot_x(1415. / 3500), self.plot_y(1))  # Competition latest land at joe (18km/h0degwind, 25m/s)


        font = wx.Font(8, wx.ROMAN, wx.NORMAL, wx.NORMAL)
        dc.SetFont(font)
        for i in range(0,3500,500):
            dc.DrawText(str(i) + "mAh", self.plot_x(float(i)/3500.0), self.plot_y(1.0))
        for i in range(25,43,3):
            dc.DrawText(str(round(i/10.0,1)) + "V", self.plot_x(0), self.plot_y(self.cell.get_volt_percent(float(i/10.0))))

        thickness = 3
        dc.SetPen(wx.Pen(wx.Colour(0,0,0),thickness))
        li = bat.batmodel[0,[0,1]]
        for i in bat.batmodel[:,[0,1]]:
            self.plot(dc,li,i)
            li=i

        dc.SetPen(wx.Pen(wx.Colour(0,0,255),thickness))
        li = bat.batmodel[0,[0,2]]
        for i in bat.batmodel[:,[0,2]]:
            self.plot(dc,li,i)
            li=i
            
        dc.SetPen(wx.Pen(wx.Colour(0,255,0),thickness))
        li = bat.batmodel[0,[0,3]]
        for i in bat.batmodel[:,[0,3]]:
            self.plot(dc,li,i)
            li=i
            
        dc.SetPen(wx.Pen(wx.Colour(255,255,0),thickness))
        li = bat.batmodel[0,[0,4]]
        for i in bat.batmodel[:,[0,4]]:
            self.plot(dc,li,i)
            li=i
            
        dc.SetPen(wx.Pen(wx.Colour(255,0,0),thickness))
        li = bat.batmodel[0,[0,5]]
        for i in bat.batmodel[:,[0,5]]:
            self.plot(dc,li,i)
            li=i
            

    def OnPaint(self, e):
        # Automatic Scaling
        w = self.w
        h = self.h

        self.stat = int(w/4)
        if self.stat<100:
            self.stat=100

        dc = wx.PaintDC(self)
        brush = wx.Brush("white")
        dc.SetBackground(brush)
        dc.Clear()

        # Background
        dc.SetBrush(wx.Brush(wx.Colour(0, 0, 0), wx.TRANSPARENT))

        fontscale = int(w * 11.0 / 800.0)
        if fontscale < 6:
            fontscale = 6
        font = wx.Font(fontscale, wx.FONTFAMILY_ROMAN, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD)
        dc.SetFont(font)

        self.StatusBox(dc,0, self.cell.get_volt(), self.cell.get_volt_perc(), self.cell.get_volt_color())
        self.StatusBox(dc,1, self.cell.get_current(), self.cell.get_current_perc(), self.cell.get_current_color() )
        self.StatusBox(dc,2, self.cell.get_energy(), self.cell.get_energy_perc(), self.cell.get_energy_color() )
        self.StatusBox(dc,3, self.cell.get_mah_from_volt(), self.cell.get_energy_perc(), self.cell.get_energy_color() )
        self.StatusBox(dc,4, self.cell.get_temp(), self.cell.get_temp_perc(), self.cell.get_temp_color())
        self.StatusBox(dc,6, self.cell.get_power_text(), self.cell.get_power_perc(), self.cell.get_power_color())
        self.StatusBox(dc,7, self.energy_prediction.get_power_fraction_text(), self.energy_prediction.get_power_fraction(), self.energy_prediction.get_power_fraction_color())
        self.StatusBox(dc,8, self.energy_prediction.get_hover_seconds_left_text(), self.energy_prediction.get_hover_seconds_fraction(), self.energy_prediction.get_hover_seconds_color())
        self.StatusBox(dc,9, self.energy_prediction.get_fw_seconds_left_text(), self.energy_prediction.get_fw_seconds_fraction(), self.energy_prediction.get_fw_seconds_color())
        self.StatusBox(dc,10, self.energy_prediction.get_fw_seconds_left_20mps_text(), self.energy_prediction.get_fw_seconds_left_20mps_fraction(), self.energy_prediction.get_fw_seconds_left_20mps_color())

        self.DischargePlot(dc)

    def __init__(self):

        self.w = WIDTH
        self.h = WIDTH + BARH

        self.cfg = wx.Config('energymon_conf')
        if self.cfg.Exists('width'):
            self.w = int(self.cfg.Read('width'))
            self.h = int(self.cfg.Read('height'))

        wx.Frame.__init__(self, id=-1, parent=None, name=u'EnergyMonFrame',
                          size=wx.Size(self.w, self.h), title=u'Energy Monitoring')

        if self.cfg.Exists('left'):
            self.x = int(self.cfg.Read('left'))
            self.y = int(self.cfg.Read('top'))
            self.SetPosition(wx.Point(self.x,self.y))

        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Bind(wx.EVT_MOVE, self.OnMove)
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        ico = wx.Icon(PPRZ_SRC + "/sw/ground_segment/python/energy_mon/energy_mon.ico", wx.BITMAP_TYPE_ICO)
        self.SetIcon(ico)

        self.bat = {}
        self.temp = {}
        self.cell = BatteryCell()
        self.energy_prediction = EnergyPrediction(self.cell)

        self.interface = IvyMessagesInterface("energymonframe")
        self.interface.subscribe(self.message_recv)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()

if __name__ == '__main__':
    energy_message = EnergyMessage({"bat": 22, "amp": 18, "power": 22 * 18, "energy": 10000})
    air_data_message = AirDataMessage({"airspeed": 21})
    cell = BatteryCell()
    cell.fill_from_energy_msg(energy_message)

    energy_prediction = EnergyPrediction(cell)
    energy_prediction.fill_from_air_data_msg(air_data_message)

    print(energy_prediction.get_expected_power())
    print(energy_prediction.get_hover_seconds_left_text())
    print(energy_prediction.get_fw_seconds_left_text())
    print(energy_prediction.get_fw_seconds_left_20mps_text())
    import matplotlib.pyplot as plt
    import numpy as np
    energies = np.arange(0, 3200*6, 10)
    seconds_left = np.zeros(energies.shape)

    # for i, energy in enumerate(energies):
    #     cell.fill_from_energy_msg(EnergyMessage({"bat": 22, "amp": 18, "power": 22 * 18, "energy": energy}))
    #     seconds_left[i] = energy_prediction.get_hover_seconds_left()
    #
    #
    # plt.plot(energies, seconds_left)
    # plt.show()
