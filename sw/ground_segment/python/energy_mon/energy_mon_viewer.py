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

import battery_model as bat

PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface

WIDTH = 600
BARH = 140


def QIColour(qi):
    return {
        0: wx.Colour(64, 64, 64),  # This channel is idle
        1: wx.Colour(128, 128, 128),  # Searching
        2: wx.Colour(0, 128, 128),  # Signal aquired
        3: wx.Colour(255, 0, 0),  # Signal detected but unusable
        4: wx.Colour(0, 0, 255),  # Code Lock on Signal
        5: wx.Colour(0, 255, 0),  # Code and Carrier locked
        6: wx.Colour(0, 255, 0),  # Code and Carrier locked
        7: wx.Colour(0, 255, 0),  # Code and Carrier locked
    }[qi]


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
        self.voltage = 0;
        self.current = 0;
        self.energy = 0;
        self.model = 0;
        self.temperature = 0;
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

    def get_volt_perc(self):
        return (self.voltage - 2.5) / (4.2 - 2.5);
    def get_current_perc(self):
        return (self.current / 10)
    def get_energy_perc(self):
        return (self.energy / bat.capacity)
    def get_model_perc(self):
        return (self.model / bat.capacity)
    def get_temp_perc(self):
        return (self.temperature / 60);

    def get_volt_color(self):
        if self.voltage < 3.4:
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
        elif (self.temperature > 10) & (self.temperature < 50):
            return 0.1
        return 0

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

    def update(self):
        self.Refresh()

    def OnSize(self, event):
        self.w = event.GetSize()[0]
        self.h = event.GetSize()[1]
        self.Refresh()

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
        if color < 0.2:
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
        
        for i in range(0,5):
            dc.DrawLine(self.plot_x(0.0), self.plot_y(i/5.0), self.plot_x(1.0), self.plot_y(i/5.0))
        for i in range(0,7):
            dc.DrawLine(self.plot_x(i/7.0), self.plot_y(0), self.plot_x(i/7.0), self.plot_y(1))

        
        dc.SetPen(wx.Pen(wx.Colour(255,180,0),4))
        dc.DrawLine(self.plot_x(self.cell.model/3500), self.plot_y(0), self.plot_x(self.cell.model/3500), self.plot_y(1))
        dc.DrawLine(self.plot_x(0.0), self.plot_y(self.cell.get_volt_perc()), self.plot_x(1.0), self.plot_y(self.cell.get_volt_perc()))

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

        self.DischargePlot(dc)

    def __init__(self):

        self.w = WIDTH
        self.h = WIDTH + BARH

        wx.Frame.__init__(self, id=-1, parent=None, name=u'EnergyMonFrame',
                          size=wx.Size(self.w, self.h), title=u'Energy Monitoring')
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        ico = wx.Icon(PPRZ_SRC + "/sw/ground_segment/python/energy_mon/energy_mon.ico", wx.BITMAP_TYPE_ICO)
        self.SetIcon(ico)

        self.bat = {}
        self.temp = {}
        self.cell = BatteryCell();

        self.interface = IvyMessagesInterface("energymonframe")
        self.interface.subscribe(self.message_recv)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()
