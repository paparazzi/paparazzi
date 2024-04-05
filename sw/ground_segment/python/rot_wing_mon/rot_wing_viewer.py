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

import wx

import sys
import os
import math
import datetime
import numpy as np


PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface

WIDTH = 600
BARH = 140

class EscMessage(object):
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
        return str(round(self.amp ,1)) + "A"
    def get_current_perc(self):
        return self.amp / 30

    def get_rpm(self):
        return str(round(self.rpm ,0)) + " rpm"
    def get_rpm_perc(self):
        return self.rpm / 5150
    def get_rpm_color(self):
        if self.rpm < 4000:
            return 1
        return 0.5


    def get_volt(self):
        if (self.id in [6,7,8,9,16,17,18,19]):
            return "Servo " + str(self.id) + " " +str(round(self.volt_m ,1)) + "V"
        else:
            return "Mot " + str(self.id) + " " +str(round(self.volt_m ,1)) + "V"
    def get_volt_perc(self):
        return self.volt_b / (6*4.2)

    def get_temp(self):
        #if self.temperature < -200:
        #    return "xxx"
        return str(round(self.temperature ,1)) + "C"
    def get_temp_perc(self):
        return self.temperature / 120.0
    
    def get_temp_dev(self):
        #if self.temperature_dev < -200:
        #    return "xxx"
        return str(round(self.temperature_dev, 1)) + "C"
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
    def __init__(self, msg):
        self.u = np.array(msg['u'], dtype=np.float)
    
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

class MotorList(object):
    def __init__(self):
        self.mot = []

    def fill_from_esc_msg(self, esc):
        added = False
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
                added = True
                break
        if not added:
            self.mot.append(esc)

class RotWingFrame(wx.Frame):
    def message_recv(self, ac_id, msg):
        if msg.name == "ESC":
            self.esc = EscMessage(msg)
            self.motors.fill_from_esc_msg(self.esc)
            wx.CallAfter(self.update)    

        if msg.name == "STAB_ATTITUDE":
            self.indi = INDIMessage(msg)
            wx.CallAfter(self.update)

    def update(self):
        self.Refresh()

    def OnSize(self, event):
        self.w = event.GetSize().x
        self.h = event.GetSize().y
        self.cfg.Write("width", str(self.w))
        self.cfg.Write("height", str(self.h))
        self.Refresh()

    def OnMove(self, event):
        self.x = event.GetPosition().x
        self.y = event.GetPosition().y
        self.cfg.Write("left", str(self.x))
        self.cfg.Write("top", str(self.y))

    def StatusBox(self, dc, dx, dy, row, col, txt, percent, color):
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
        dc.DrawRectangle(tdx +  col * 200 + dx, int(row*spacing+tdx) +dy, int(boxw), boxh)
        dc.SetTextForeground(wx.Colour(0, 0, 0))
        if color < 0.2:
            dc.SetTextForeground(wx.Colour(255, 255, 255))
            dc.SetBrush(wx.Brush(wx.Colour(250,0,0)))
        elif color < 0.6:
            dc.SetBrush(wx.Brush(wx.Colour(250,180,0)))
        else:
            dc.SetBrush(wx.Brush(wx.Colour(0,250,0)))
#        dc.DrawLine(200,50,350,50)
        dc.DrawRectangle(tdx +  col * 200 + dx, int(row*spacing+tdx+dy), int(boxw * percent), boxh)
        dc.DrawText(txt,18 + col * 200 + dx,int(row*spacing+tdy+tdx+dy))

    def OnPaint(self, e):
        # Automatic Scaling
        w = self.w
        h = self.h - 25

        dc = wx.PaintDC(self)
        brush = wx.Brush("white")
        dc.SetBackground(brush)
        dc.Clear()

        # Background
        dc.SetBrush(wx.Brush(wx.Colour(0, 0, 0), wx.TRANSPARENT))

        fontscale = int(w * 11.0 / 1500.0)
        if fontscale < 6:
            fontscale = 6
        font = wx.Font(fontscale, wx.FONTFAMILY_ROMAN, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD)
        dc.SetFont(font)

        # Draw Drone
        dc.SetPen(wx.Pen(wx.Colour(0,0,0))) 
        dc.SetBrush(wx.Brush(wx.Colour(240,240,220))) 
        # Fuselage
        dc.DrawRoundedRectangle(int(0.475*w), int(0.05*h),int(0.05*w), int(0.75*h), int(0.015*w))
        dc.DrawRoundedRectangle(int(0.425*w), int(0.05*h),int(0.15*w), int(0.4*h), int(0.05*w))
        # Front Wing
        dc.DrawRectangle(int(0.05*w), int(0.25*h),int(0.9*w), int(0.15*h))
        # Back Wing
        dc.DrawRectangle(int(0.25*w), int(0.65*h),int(0.5*w), int(0.1*h))

        # Motors
        self.stat = int(0.10*w)
        w1 = 0.03
        w2 = 0.18
        w3 = 0.31
        w4 = 0.52
        dw = 0.17
        mw = 0.1
        mh = 0.2

        mm = [(0.5-0.5*mw-0.05,w1), (0.5+dw-0.05+0.1,w2), (0.5-0.5*mw-0.05,w3), (0.5-dw-mw-0.05-0.1,w2), # hover CAN1: front right back left
            (0.5-0.5*mw-0.05,w4), # pusher CAN1
            (0.5-dw-mw-0.05,w4+0.05), (0.5+dw-0.05,w4+0.05), # tail CAN1
            (0.5-0.5*mw+0.05,w1), (0.5+dw+0.05+0.1,w2), (0.5-0.5*mw+0.05,w3), (0.5-dw-mw+0.05-0.1,w2), # hover CAN2
            (0.5-0.5*mw+0.05,w4), # pusher CAN2
            (0.5-dw-mw+0.05,w4+0.05), (0.5+dw+0.05,w4+0.05), # Tail CAN2
            (0.0, 0.75), (0.1, 0.75), (0.2, 0.75), (0.3, 0.75), (0.4, 0.75), # extra
            (0.5, 0.75), (0.6, 0.75), (0.7, 0.75), (0.8, 0.75), (0.9, 0.75),
        ]

        mot_ids = [ 0, 1, 2, 3,     # hover CAN1
            4,                      # pusher CAN1
            6, 7,                   # elevator/rudder CAN1
            12,13,14,15,            # hover CAN2
            16,                     # pusher CAN2
            18, 19,                 # tail CAN2
            5, 8, 9, 10, 11,        # Extra's at the bottoms
            17, 20, 21, 22, 23
        ]

        for m in self.motors.mot:
            mot_id = m.id
            
            # Drawing ID
            if not (mot_id in mot_ids):
                continue
            draw_id = mot_ids.index(mot_id)
            if draw_id >= len(mm):
                continue
            mo_co = mm[draw_id]
            #print(m.id, mo_co)
            dx = int(mo_co[0]*w)
            dy = int(mo_co[1]*h)

            dc.SetBrush(wx.Brush(wx.Colour(200,200,100))) 
            dc.DrawRectangle( int((mo_co[0]+0.001)*w), int((mo_co[1]+0.001)*h),int((mw-0.002)*w), int((mh-0.002)*h) )

            self.StatusBox(dc, dx, dy, 0, 0, m.get_volt(), m.get_volt_perc(), 1)
            self.StatusBox(dc, dx, dy, 1, 0, m.get_current(), m.get_current_perc(), 1)
            self.StatusBox(dc, dx, dy, 2, 0, m.get_rpm(), m.get_rpm_perc(), m.get_rpm_color())
            self.StatusBox(dc, dx, dy, 3, 0, m.get_temp(), m.get_temp_perc(), m.get_temp_color())
            self.StatusBox(dc, dx, dy, 4, 0, m.get_temp_dev(), m.get_temp_dev_perc(), m.get_temp_dev_color())
            try:
                indi_id = m.id
                if m.id == 4:
                    indi_id=8

                if indi_id < self.indi.get_len():
                    self.StatusBox(dc, dx, dy, 5, 0, self.indi.get_u(indi_id), self.indi.get_u_perc(indi_id), self.indi.get_u_color(indi_id))
                
            except:
                pass

    def __init__(self):

        self.w = WIDTH
        self.h = WIDTH + BARH

        self.cfg = wx.Config('rot_wing_conf')
        if self.cfg.Exists('width'):
            self.w = int(self.cfg.Read('width'))
            self.h = int(self.cfg.Read('height'))

        wx.Frame.__init__(self, id=-1, parent=None, name=u'RotWingFrame',
                          size=wx.Size(self.w, self.h), title=u'Rot Wing Monitoring')

        if self.cfg.Exists('left'):
            self.x = int(self.cfg.Read('left'))
            self.y = int(self.cfg.Read('top'))
            self.SetPosition(wx.Point(self.x,self.y))

        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Bind(wx.EVT_MOVE, self.OnMove)
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        #ico = wx.Icon(PPRZ_SRC + "/sw/ground_segment/python/energy_mon/energy_mon.ico", wx.BITMAP_TYPE_ICO)
        #self.SetIcon(ico)

        self.motors = MotorList()
     
        self.interface = IvyMessagesInterface("rotwingframe")
        self.interface.subscribe(self.message_recv)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()

