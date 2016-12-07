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

import wx

import sys
import os
import time
import threading
import math
import pynotify

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                      '../../../..')))

sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")

from pprzlink.ivy import IvyMessagesInterface

WIDTH = 300
BARH = 140
CHANNEL = 16


def QIColour(qi):
    return {
        0: wx.Colour(64,64,64),  # This channel is idle
        1: wx.Colour(128,128,128),  # Searching
        2: wx.Colour(0,128,128),  # Signal aquired
        3: wx.Colour(255,0,0),  # Signal detected but unusable
        4: wx.Colour(0,0,255),  # Code Lock on Signal
        5: wx.Colour(0,255,0),  # Code and Carrier locked
        6: wx.Colour(0,255,0),  # Code and Carrier locked
        7: wx.Colour(0,255,0),  # Code and Carrier locked
    }[qi]

class SvChannel(object):
    def __init__(self, chn, msg):
        self.chn = chn
        self.SVID = int(msg['SVID'])
        self.Flags = int(msg['Flags'])
        self.QI = int(msg['QI'])
        self.CNO = int(msg['CNO'])
        self.Elev = int(msg['Elev'])
        self.Azim = int(msg['Azim'])


class SVInfoFrame(wx.Frame):
    def message_recv(self, ac_id, msg):
        if msg.name == "SVINFO":
            chn = int(msg['chn'])
            self.sv[chn] = SvChannel(chn, msg)
            wx.CallAfter(self.update)

    def update(self):
        self.Refresh()

    def OnSize(self, event):
        self.w = event.GetSize()[0]
        self.h = event.GetSize()[1]
        self.Refresh()

    def OnPaint(self, e):
        tdx = -5
        tdy = -7
        th = 15

        w = self.w
        h = self.w
        if h < self.w + 50:
            h = self.w + 50
        bar = self.h-w-th-th


        dc = wx.PaintDC(self)
        brush = wx.Brush("white")
        dc.SetBackground(brush)
        dc.Clear()

        # Background
        dc.SetBrush(wx.Brush(wx.Colour(0,0,0), wx.TRANSPARENT))
        dc.DrawCircle(w/2,w/2,w/2-1)
        dc.DrawCircle(w/2,w/2,w/4-1)
        dc.DrawCircle(w/2,w/2,1)
        font = wx.Font(11, wx.ROMAN, wx.BOLD, wx.NORMAL)
        dc.SetFont(font)

        dc.DrawText("N",w/2+tdx,2)
        dc.DrawText("S",w/2+tdx,w-17)
        dc.DrawText("E",w-15,w/2+tdy)
        dc.DrawText("W",2,w/2+tdy)

        # SV
        for chn in self.sv:
            sv = self.sv[chn]

            c = QIColour(sv.QI)
            used = sv.Flags & 1
            s = 7 + used * 5

            el = float(sv.Elev) / 90.0 * float(w) / 2.0
            az = float(sv.Azim) * math.pi / 180.0

            y = float(w)/2.0 - math.cos(az) * el
            x = float(w)/2.0 + math.sin(az) * el

            dc.SetBrush(wx.Brush(c, wx.SOLID))
            dc.DrawCircle(int(x),int(y),s)

            font = wx.Font(8, wx.ROMAN, wx.NORMAL, wx.NORMAL)
            dc.SetFont(font)
            dc.DrawText(str(sv.SVID),x+tdx,y+tdy)

            bh = float(bar-th-th) * float(sv.CNO) / 55.0
            dc.DrawRectangle(w/CHANNEL*chn+5 * (1-used),self.h-th-bh,w/CHANNEL-2 - 10 * (1-used),bh)
            dc.DrawText(str(chn),w/CHANNEL*chn,self.h-th)
            dc.DrawText(str(sv.CNO),w/CHANNEL*chn,self.h-bar)


    def __init__(self):

        self.w = WIDTH
        self.h = WIDTH + BARH

        wx.Frame.__init__(self, id=-1, parent=None, name=u'SVInfoFrame',
                          size=wx.Size(self.w, self.h), title=u'SV Info')
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        ico = wx.Icon(PPRZ_SRC + "/sw/ground_segment/python/svinfo/svinfo.ico", wx.BITMAP_TYPE_ICO)
        self.SetIcon(ico)

        self.sv = {}

        self.interface = IvyMessagesInterface("svinfoframe")
        self.interface.subscribe(self.message_recv)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()
