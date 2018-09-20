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
import array
from cStringIO import StringIO

PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface

WIDTH = 300


class AtcFrame(wx.Frame):

    def message_recv(self, ac_id, msg):
        if msg.name == "INS_REF":
            self.qfe = round(float(msg['baro_qfe'])  / 100.0,1)
            wx.CallAfter(self.update)
        elif msg.name =="ROTORCRAFT_FP_MIN":
            self.gspeed = round(float(msg['gspeed']) / 100.0 * 3.6 / 1.852,1)
            self.alt = round(float(msg['up']) * 0.0039063 * 3.28084 ,1)
            wx.CallAfter(self.update)
        elif msg.name =="ROTORCRAFT_FP":
            self.alt = round(float(msg['up']) * 0.0039063 * 3.28084 ,1)
            wx.CallAfter(self.update)
        elif msg.name =="AIR_DATA":
            self.airspeed = round(float(msg['airspeed']) * 3.6 / 1.852,1)
            self.qnh = round(float(msg['qnh']),1)
            self.amsl = round(float(msg['amsl_baro']) * 3.28084,1)
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
        if color < 0.2
            dc.SetBrush(wx.Brush(wx.Colour(250,0,0)))
        elif color < 0.6:
            dc.SetBrush(wx.Brush(wx.Colour(250,180,0)))
        else:
            dc.SetBrush(wx.Brush(wx.Colour(0,250,0)))
#        dc.DrawLine(200,50,350,50)
        dc.DrawRectangle(tdx, int(nr*spacing+tdx), int(boxw * percent), boxh)
        dc.DrawText(txt,18,int(nr*spacing+tdy+tdx))


    def OnPaint(self, e):

        w = self.w
        h = self.h

        if (float(w)/float(h)) > (7.0/5.0):
          w = int(h * 7.0/5.0)
        else:
          h = int(w * 5.0/7.0)

	tdy = int(w * 75.0 / 700.0)
        tdx = int(w * 15.0 / 700.0)

        dc = wx.PaintDC(self)
        #brush = wx.Brush("white")
        #dc.SetBackground(brush)
        #dc.Clear()

	fontscale = int(w * 40.0 / 700.0)
        if fontscale < 6:
            fontscale = 6

        # Background
        dc.SetBrush(wx.Brush(wx.Colour(0,0,0), wx.TRANSPARENT))
        #dc.DrawCircle(w/2,w/2,w/2-1)
        font = wx.Font(fontscale, wx.FONTFAMILY_ROMAN, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD)
        dc.SetFont(font)
        dc.DrawText("Airspeed: " + str(self.airspeed) + " kt",tdx,tdx)
        dc.DrawText("Ground Speed: " + str(self.gspeed) + " kt",tdx,tdx+tdy*1)

        dc.DrawText("AMSL: " + str(self.amsl) + " ft",tdx,tdx+tdy*2)
        dc.DrawText("QNH: " + str(self.qnh) + " ",tdx,tdx+tdy*3)

        dc.DrawText("ALT: " + str(self.alt) + " ",tdx,tdx+tdy*4)
        dc.DrawText("QFE: " + str(self.qfe) + " ",tdx,tdx+tdy*5)

        #dc.DrawText("HMSL: " + str(self.hmsl) + " ft",tdx,tdx+tdy*6)

        #c = wx.Colour(0,0,0)
        #dc.SetBrush(wx.Brush(c, wx.SOLID))
        #dc.DrawCircle(int(w/2),int(w/2),10)



    def __init__(self):

        self.w = 700
        self.h = 500

        self.airspeed = 0;

        self.amsl = 0;
        self.qnh = 0;

        self.qfe = 0;
        self.alt = 0;

        #self.hmsl = 0;
        self.gspeed = 0;

        wx.Frame.__init__(self, id=-1, parent=None, name=u'ATC Center',
                          size=wx.Size(self.w, self.h), title=u'ATC Center')


        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        ico = wx.Icon(PPRZ_SRC + "/sw/ground_segment/python/atc/atc.ico", wx.BITMAP_TYPE_ICO)
        self.SetIcon(ico)

        self.interface = IvyMessagesInterface("ATC-Center")
        self.interface.subscribe(self.message_recv)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()
