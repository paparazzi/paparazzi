#
# Copyright (C) 2016 TUDelft
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

# This is not the main script. Run dist.py to have a distance counter.

import wx
import sys
import os
import threading
import socket
import array
from io import StringIO
import wx
import array
from PIL import Image
import math

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../..')))

sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")

from pprzlink.ivy import IvyMessagesInterface

WIDTH = 300



class DistanceCounterFrame(wx.Frame):

    def message_recv(self, ac_id, msg):
        if msg.name == "INS":

            self.msg_count = self.msg_count + 1

            newx = float(msg.get_field(0)) / 256.0
            newy = float(msg.get_field(1)) / 256.0

            moved = ((newx - self.ins_msg_x) ** 2 + (newy - self.ins_msg_y) ** 2)
            if self.init == 0:
                self.init = 1
            elif self.running:
                self.distance = self.distance + math.sqrt(moved)

            self.ins_msg_x = newx
            self.ins_msg_y = newy
            self.ins_msg_z = msg.get_field(2)


            # graphical update
            wx.CallAfter(self.update)
        if msg.name == "ROTORCRAFT_STATUS":
            self.msg_count_time = self.msg_count_time + 1
            time_new = float(msg['cpu_time'])
            if time_new > self.time_old and self.time_old != 0 and self.running:
                self.time_elapsed += time_new - self.time_old
            self.time_old = time_new
            # graphical update
            wx.CallAfter(self.update)

    def update(self):
        self.Refresh()

    def OnSize(self, event):
        self.w = event.GetSize()[0]
        self.h = event.GetSize()[1]
        self.Refresh()

    def OnPaint(self, e):
        # Paint Area
        dc = wx.PaintDC(self)
        brush = wx.Brush("white")
        dc.SetBackground(brush)
        dc.Clear()

        # Background
        dc.SetBrush(wx.Brush(wx.Colour(0,0,0), wx.TRANSPARENT))
        font = wx.Font(11, wx.DEFAULT, wx.NORMAL, wx.NORMAL)
        dc.SetFont(font)
        dc.DrawText("INS Packets:" + str(self.msg_count),2,2)
        dc.DrawText("Data: " + str(self.ins_msg_x) + ", " + str(self.ins_msg_y) + ", " + str(self.ins_msg_z) + ".",2,22)
        dc.DrawText("Distance: " + str(round(float(self.distance)/1.0,2)) + " m",2,22+20)
        dc.DrawText("Time elapsed: " + str(self.time_elapsed) + "s",2,22+20+20)
        if self.running:
            dc.DrawText("Counter running", 150, 22+20)
        else:
            dc.DrawText("Counter paused", 150, 22+20)

    def onStartStop(self, event):
        self.running = not self.running
        self.Refresh()

    def onReset(self, event):
        self.time_old = 0
        self.time_elapsed = 0
        self.distance = 0
        self.init = 0
        self.Refresh()
        return

    def __init__(self, _settings):
        # Command line arguments
        self.settings = _settings

        # Statistics
        self.data = { 'packets': 0, 'bytes': 0}

        self.w = WIDTH
        self.h = WIDTH

        # Frame
        wx.Frame.__init__(self, id=-1, parent=None, name=u'Distance Counter',
                          size=wx.Size(self.w, self.h), title=u'Distance Counter')

        start_stop_button = wx.Button(self, wx.ID_ANY, 'Start/Pause', (150, 58),size=(90, 25))
        start_stop_button.Bind(wx.EVT_BUTTON, self.onStartStop)
        reset_button = wx.Button(self, wx.ID_ANY, 'Reset', (245, 58), size=(50, 25))
        reset_button.Bind(wx.EVT_BUTTON, self.onReset)
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        # IVY
        self.interface = IvyMessagesInterface("DistanceCounter")
        self.interface.subscribe(self.message_recv)

        self.msg_count = 0
        self.msg_count_time = 0
        self.distance = 0
        self.time_old = 0
        self.time_elapsed = 0
        self.ins_msg_x = 0
        self.ins_msg_y = 0
        self.ins_msg_z = 0
        self.init = 0
        self.running = True

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()

if __name__ == '__main__':
    raise Exception('This is not the main script. Please run dist.py instead of distance_counter.py')
