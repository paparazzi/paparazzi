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
        return str(round(self.temperature ,0)) + "C"
    def get_temp_perc(self):
        return self.temperature / 120.0
    
    def get_temp_dev(self):
        #if self.temperature_dev < -200:
        #    return "xxx"
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
        if self.status & (0x1 << 0):
            self.skew_angle_valid = True
        else:
            self.skew_angle_valid = False
        if self.status & (0x1 << 1):
            self.hover_motors_enabled = True
        else:
            self.hover_motors_enabled = False
        if self.status & (0x1 << 2):
            self.hover_motors_idle = True
        else:
            self.hover_motors_idle = False
        if self.status & (0x1 << 3):
            self.hover_motors_running = True
        else:
            self.hover_motors_running = False
        if self.status & (0x1 << 4):
            self.pusher_motor_running = True
        else:
            self.pusher_motor_running = False
        if self.status & (0x1 << 5):
            self.skew_forced = True
        else:
            self.skew_forced = False
    
    def get_state(self):
        states = ['FORCE_HOVER', 'REQ_HOVER', 'FORCE_FW', 'REQ_FW', 'FREE']
        return states[self.state]
    
    def get_nav_state(self):
        states = ['FORCE_HOVER', 'REQ_HOVER', 'FORCE_FW', 'REQ_FW', 'FREE']
        return states[self.nav_state]
    
class AIRDATAMessage(object):
    def __init__(self, msg):
        self.airspeed = float(msg['airspeed'])
        self.tas = float(msg['tas'])

class IMUHEATERMessage(object):
    def __init__(self, msg):
        self.meas_temp = msg['meas_temp']

#    <message name="POWER_DEVICE" id="19">
#      <field name="node_id" type="uint8"/>
#      <field name="circuit" type="uint8"/>
#      <field name="current" type="float"/>
#      <field name="voltage" type="float"/>
#    </message>


class PowerList(object):
    def __init__(self):
        self.power = {}
    

class POWERDEVICEMessage(object):
    def __init__(self, msg):
        self.node_id = int(msg['node_id'])
        self.circuit = int(msg['circuit'])
        self.current = float(msg['current'])
        self.voltage = float(msg['voltage'])



class FUELCELLMessage(object):
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
        states = ['FCPM off', 'Starting', 'Running', 'Stopping', 'Go to sleep']
        self.state_str = '?'
        if self.state < len(states):
            self.state_str = states[self.state]
        if self.error == 0:
            self.error_str = 'No error'
        elif self.error == 1:
            self.error_str = 'Minor internal'
        elif self.error == 4:
            self.error_str = 'Reconditioning needed'
        elif self.error == 6:
            self.error_str = 'Auto recondition active'
        elif self.error == 7:
            self.error_str = 'Reconditioning active'
        elif self.error == 8:
            self.error_str = 'Recovery mode active'
        elif self.error == 12:
            self.error_str = 'Tank pressure low'
        elif self.error == 14:
            self.error_str = 'Battery voltage low'
        elif self.error == 20:
            self.error_str = 'Reduced power'
        elif self.error == 25:
            self.error_str = 'SPM lost'
        elif self.error == 30:
            self.error_str = 'Tank empty'
        elif self.error == 32:
            self.error_str = 'Start denied'
        elif self.error == 34:
            self.error_str = 'FC off - recovery required'
        elif self.error == 36:
            self.error_str = 'FC off - system error'
        elif self.error == 40:
            self.error_str = 'Battery critical'
        else:
            self.error_str = 'Unknown error'

        if self.suberror == 0:
            self.suberror_str = 'No context'
        elif self.suberror == 1:
            self.suberror_str = 'SPM1'
        elif self.suberror == 2:
            self.suberror_str = 'SPM2'
        elif self.suberror == 3:
            self.suberror_str = 'SPM1 AND SPM2'
        elif self.suberror == 4:
            self.suberror_str = 'ALL SPM LOST'
        elif self.suberror == 5:
            self.suberror_str = 'Inlet pressure low'
        elif self.suberror == 6:
            self.suberror_str = 'Inlet pressure high'
        elif self.suberror == 7:
            self.suberror_str = 'Tank pressure low'
        elif self.suberror == 8:
            self.suberror_str = 'Tank pressure high'
        elif self.suberror == 9:
            self.suberror_str = 'Stack voltage low'
        elif self.suberror == 10:
            self.suberror_str = 'Internal tests (INT ERR)'
        elif self.suberror == 11:
            self.suberror_str = 'Stack temperature high (STK HT)'
        elif self.suberror == 12:
            self.suberror_str = 'Stack temperature low (STK LT)'
        elif self.suberror == 13:
            self.suberror_str = 'Reconditioning active'
        elif self.suberror == 14:
            self.suberror_str = 'Reconditioning complete'
        elif self.suberror == 15:
            self.suberror_str = 'Thermal management'
        elif self.suberror == 16:
            self.suberror_str = 'Start faults'
        elif self.suberror == 17:
            self.suberror_str = 'LPI missing'
        elif self.suberror == 254:
            self.suberror_str = 'Complete flight - contact support'
        elif self.suberror == 255:
            self.suberror_str = 'Stop using unit - contact support'
        else:
            self.suberror_str = 'Unknown suberror'

class PowerList(object):
    def __init__(self):
        self.power = []

    def fill_from_power_msg(self, power):
        added = False
        for i in range(len(self.power)):
            if self.power[i].node_id == power.node_id and self.power[i].circuit == power.circuit:
                self.power[i] = power
                added = True
                break
        if not added:
            self.power.append(power)

    def get_frontbat(self):
        for i in range(len(self.power)):
            if self.power[i].node_id == 14 and self.power[i].circuit == 0:
                return self.power[i]
        return None

    def get_backbat(self):
        for i in range(len(self.power)):
            if self.power[i].node_id == 6 and self.power[i].circuit == 0:
                return self.power[i]
        return None
    
    def get_backmot(self):
        for i in range(len(self.power)):
            if self.power[i].node_id == 6 and self.power[i].circuit == 1:
                return self.power[i]
        return None

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
        
        if msg.name =="ROTATING_WING_STATE":
            self.rw_status = RWStatusMessage(msg)
            wx.CallAfter(self.update)

        if msg.name == "AIR_DATA":
            self.air_data = AIRDATAMessage(msg)
            wx.CallAfter(self.update)

        if msg.name == "IMU_HEATER":
            self.imu_heater = IMUHEATERMessage(msg)
            wx.CallAfter(self.update)
        
        if msg.name == "FUELCELL":
            self.fuelcell = FUELCELLMessage(msg)
            wx.CallAfter(self.update)

        if msg.name == "POWER_DEVICE":
            self.powermessage = POWERDEVICEMessage(msg)
            self.powers.fill_from_power_msg(self.powermessage)
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

        # Draw rotwing status
        self.stat = int(0.10*w)
        if hasattr(self, 'rw_status'):
            line = int(20.0 / 800.0 * h)
            dc.SetBrush(wx.Brush(wx.Colour(200,200,100))) 
            dc.DrawRectangle(int(5), int(5),int(0.29*w), int(0.20*h))
            dc.DrawText("State: " + self.rw_status.get_state() + " [NAV: " + self.rw_status.get_nav_state() +"]", 10, int(0.5*line))
            if self.rw_status.skew_angle_valid:
                if abs(self.rw_status.meas_skew_angle - self.rw_status.sp_skew_angle) < 10:
                    dc.SetTextForeground(wx.Colour(0, 0, 0))
                else:
                    dc.SetTextForeground(wx.Colour(139, 64, 0))
                dc.DrawText("Rotation: " + str(round(self.rw_status.meas_skew_angle, 1)) + " (SP: " + str(round(self.rw_status.sp_skew_angle, 1)) + ")", 10, int(1.5*line))
            else:
                dc.SetTextForeground(wx.Colour(255, 0, 0))
                dc.DrawText("Rotation: " + str(round(self.rw_status.meas_skew_angle, 1)) + " (SP: " + str(round(self.rw_status.sp_skew_angle, 1)) + ")", 10, int(1.5*line))
            
            dc.SetTextForeground(wx.Colour(0, 0, 0))
            dc.DrawText("Hover motors: ", 10, int(2.5*line))
            lbw = dc.GetTextExtent("Hover Motors: ").width
            if self.rw_status.hover_motors_running:
                dc.SetTextForeground(wx.Colour(0, 0, 0))
                dc.DrawText("running ", 10 + lbw, int(2.5*line))
                lbw += dc.GetTextExtent("running ").width
            else:
                dc.SetTextForeground(wx.Colour(255, 0, 0))
                dc.DrawText("stopped ", 10 + lbw, int(2.5*line))
                lbw += dc.GetTextExtent("stopped ").width

            if self.rw_status.hover_motors_idle:
                dc.SetTextForeground(wx.Colour(0, 0, 0))
                dc.DrawText("idle ", 10 + lbw, int(2.5*line))
                lbw += dc.GetTextExtent("idle ").width

            if self.rw_status.hover_motors_enabled:
                dc.SetTextForeground(wx.Colour(0, 0, 0))
                dc.DrawText("(Enabled)", 10 + lbw, int(2.5*line))
            else:
                dc.SetTextForeground(wx.Colour(255, 0, 0))
                dc.DrawText("(Disabled)", 10 + lbw, int(2.5*line))

            dc.SetTextForeground(wx.Colour(0, 0, 0))
            dc.DrawText("Pusher motor: ", 10, int(3.5*line))
            lbw = dc.GetTextExtent("Pusher motor: ").width
            if self.rw_status.pusher_motor_running:
                dc.SetTextForeground(wx.Colour(0, 0, 0))
                dc.DrawText("running", 10 + lbw, int(3.5*line))
            else:
                dc.SetTextForeground(wx.Colour(255, 0, 0))
                dc.DrawText("stopped", 10 + lbw, int(3.5*line))
            
            dc.SetTextForeground(wx.Colour(0, 0, 0))
            dc.DrawText("Nav airspeed: " + str(round(self.rw_status.nav_airspeed,1 )) + " [min: " + str(round(self.rw_status.min_airspeed,1 )) + ", max:" + str(round(self.rw_status.max_airspeed,1 )) + "]", 10, int(4.5*line))
            if hasattr(self, 'air_data'):
                dc.DrawText("Meas airspeed: " + str(round(self.air_data.airspeed,1 )) + " (TAS: " + str(round(self.air_data.tas,1 )) + ")", 10, int(5.5*line))
            #self.StatusBox(dc, 5, 5, 0, 0, self.rw_status.get_state(), 1, 1)
            dc.SetTextForeground(wx.Colour(0, 0, 0))
            dc.DrawText("Force Skew: ", 10, int(6.5*line))
            lbw = dc.GetTextExtent("Force Skew: ").width
            if self.rw_status.skew_forced:
                dc.SetTextForeground(wx.Colour(255, 0, 0))
                dc.DrawText("(Enabled)", 10 + lbw, int(6.5*line))
            else:
                dc.SetTextForeground(wx.Colour(0, 0, 0))
                dc.DrawText("(Disabled)", 10 + lbw, int(6.5*line))
            
            if hasattr(self, 'imu_heater'):
                imu_temp = float(self.imu_heater.meas_temp)
                if imu_temp < 65.0:
                    dc.SetTextForeground(wx.Colour(0, 0, 0))
                    dc.DrawText("Meas IMU temp: " + str(round(imu_temp, 0)), 10, int(7.5*line))
                elif imu_temp < 85.0:
                    dc.SetTextForeground(wx.Colour(139, 64, 0))
                    dc.DrawText("Meas IMU temp: " + str(round(imu_temp, 0)), 10, int(7.5*line))
                else:
                    dc.SetTextForeground(wx.Colour(255, 0, 0))
                    dc.DrawText("Meas IMU temp: " + str(round(imu_temp, 0)), 10, int(7.5*line))

            if self.powers.get_backbat() != None:
                dc.DrawText("Back Bat: " + str(round(self.powers.get_backbat().current*self.powers.get_backbat().voltage, 1)) + "W (" + str(round(self.powers.get_backbat().voltage, 1)) + "V, " + str(round(self.powers.get_backbat().current, 1)) + "A)", 10, int(8.5*line))
            if self.powers.get_frontbat() != None:
                dc.DrawText("Front Bat: " + str(round(self.powers.get_frontbat().current*self.powers.get_frontbat().voltage, 1)) + "W (" + str(round(self.powers.get_frontbat().voltage, 1)) + "V, " + str(round(self.powers.get_frontbat().current, 1)) + "A)", 10, int(9.5*line))
            if self.powers.get_backmot() != None:
                dc.DrawText("Back Mot: " + str(round(self.powers.get_backmot().current*self.powers.get_backmot().voltage, 1)) + "W (" + str(round(self.powers.get_backmot().voltage, 1)) + "V, " + str(round(self.powers.get_backmot().current, 1)) + "A)", 10, int(10.5*line))

        if hasattr(self, 'fuelcell'):
            dc.SetBrush(wx.Brush(wx.Colour(200,200,200)))
            line = int(20.0 / 800.0 * h)
            dc.SetTextForeground(wx.Colour(0, 0, 0))
            dc.DrawRoundedRectangle(int(0.7*w), int(5),int(0.29*w), int(0.18*h), int(0.05*h))
            dc.DrawText("FuelCell state: ["+str(self.fuelcell.state)+"] " + self.fuelcell.state_str, int(0.75*w+5), int(0.5*line))
            dc.DrawText("Cylinder press: " + str(self.fuelcell.pressure) + "%  Reg="+str(self.fuelcell.press_reg)+"Bar", int(0.75*w+5), int(1.5*line))
            dc.DrawText("Output: " + str(self.fuelcell.power_out) + " Watt", int(0.75*w+5), int(2.5*line))
            dc.DrawText("SPM: " + str(self.fuelcell.power_cell) + " Watt, p=" + str(round(float(self.fuelcell.pressure)*3.630,1)) + " Bar", int(0.75*w+5), int(3.5*line))
            dc.DrawText("Battery: " + str(self.fuelcell.power_batt) + " Watt, " + str(self.fuelcell.volt_bat) + " Volt", int(0.75*w+5), int(4.5*line))
            dc.DrawText("Error: ["+str(self.fuelcell.error)+"] " + self.fuelcell.error_str, int(0.75*w+5), int(5.5*line))
            dc.DrawText("Sub-error: ["+str(self.fuelcell.suberror)+"] " + self.fuelcell.suberror_str, int(0.75*w+5), int(6.5*line))



        # Motors
        w1 = 0.03
        w2 = 0.18
        w3 = 0.31
        w4 = 0.52
        dw = 0.17
        mw = 0.1
        mh = 0.2

        mm = [(0.5-0.5*mw-0.05,w1), (0.5+dw+0.05+0.1,w2), (0.5-0.5*mw-0.05,w3), (0.5-dw-mw+0.05-0.1,w2), # hover CAN1: front right back left
            (0.5-0.5*mw-0.05,w4), # pusher CAN1
            (0.5-dw-mw-0.05,w4+0.05), (0.5+dw-0.05,w4+0.05), # tail CAN1
            (0.5-0.5*mw+0.05,w1), (0.5+dw-0.05+0.1,w2), (0.5-0.5*mw+0.05,w3), (0.5-dw-mw-0.05-0.1,w2), # hover CAN2
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

        self.cfg = wx.Config('rotwing_conf')
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
        self.powers = PowerList()
     
        self.interface = IvyMessagesInterface("rotwingframe")
        self.interface.subscribe(self.message_recv)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()

