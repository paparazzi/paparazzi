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
import array
from cStringIO import StringIO

PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface

HEIGHT = 580.0
WIDTH = 700.0


class AtcFrame(wx.Frame):

    def message_recv(self, ac_id, msg):
        if (ac_id == 6) | (ac_id == 7):
            self.callsign = "PH-3MM"
        elif (ac_id == 17) | (ac_id == 18):
            self.callsign = "PH-2OI"
        elif (ac_id == 54) | (ac_id == 55):
            self.callsign = "PH-4HP"
        else:
            self.callsign = "ID=" + str(ac_id)

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
        elif msg.name =="ENERGY":
            bat = float(msg['bat'])
            if bat < 10.0:
                self.safe_to_approach = "Afirm"
            else:
                self.safe_to_approach = "Negative!"
            wx.CallAfter(self.update)

    def update(self):
        self.Refresh()

    def OnSize(self, event):
<<<<<<< HEAD
        self.w = event.GetSize()[0]
        self.h = event.GetSize()[1]
        self.Refresh()

=======
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

>>>>>>> delftacopter
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


    def OnPaint(self, e):

        w = self.w
        h = self.h

        if (float(w)/float(h)) > (WIDTH/HEIGHT):
          w = int(h * WIDTH/HEIGHT)
        else:
          h = int(w * HEIGHT/WIDTH)

        tdy = int(w * 75.0 / WIDTH)
        tdx = int(w * 15.0 / WIDTH)

        dc = wx.PaintDC(self)
        #brush = wx.Brush("white")
        #dc.SetBackground(brush)
        #dc.Clear()

        fontscale = int(w * 40.0 / WIDTH)
        if fontscale < 6:
            fontscale = 6

        # Background
        dc.SetBrush(wx.Brush(wx.Colour(0,0,0), wx.TRANSPARENT))
        #dc.DrawCircle(w/2,w/2,w/2-1)
        font = wx.Font(fontscale, wx.FONTFAMILY_ROMAN, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD)
        dc.SetFont(font)
        dc.DrawText("Callsign: " + str(self.callsign) + " ",tdx,tdx+tdy*0)
        dc.DrawText("Airspeed: " + str(self.airspeed) + " kt",tdx,tdx+tdy*1)
        dc.DrawText("Ground Speed: " + str(self.gspeed) + " kt",tdx,tdx+tdy*2)

        dc.DrawText("AMSL: " + str(self.amsl) + " ft (<2700ft)",tdx,tdx+tdy*3)
        dc.DrawText("AGL: " + str(self.alt) + " ft (<1500ft)",tdx,tdx+tdy*4)

        dc.DrawText("QNH: " + str(self.qnh*100.0) + " QFE: " + str(self.qfe) + "",tdx,tdx+tdy*5)


        dc.DrawText("Safe to approach: " + self.safe_to_approach + " ",tdx,tdx+tdy*6)

        #dc.DrawText("HMSL: " + str(self.hmsl) + " ft",tdx,tdx+tdy*6)

        #c = wx.Colour(0,0,0)
        #dc.SetBrush(wx.Brush(c, wx.SOLID))
        #dc.DrawCircle(int(w/2),int(w/2),10)



    def __init__(self):

        self.w = WIDTH
        self.h = HEIGHT

        self.airspeed = 0;

        self.amsl = 0;
        self.qnh = 0;

        self.qfe = 0;
        self.alt = 0;

        #self.hmsl = 0;
        self.gspeed = 0;
        self.callsign = ""
  
        self.safe_to_approach = "";

        self.cfg = wx.Config('atc_conf')
        if self.cfg.Exists('width'):
            self.w = int(self.cfg.Read('width'))
            self.h = int(self.cfg.Read('height'))

        wx.Frame.__init__(self, id=-1, parent=None, name=u'ATC Center',
                          size=wx.Size(self.w, self.h), title=u'ATC Center')


        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Bind(wx.EVT_MOVE, self.OnMove)
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        ico = wx.Icon(PPRZ_SRC + "/sw/ground_segment/python/atc/atc.ico", wx.BITMAP_TYPE_ICO)
        self.SetIcon(ico)

        self.interface = IvyMessagesInterface("ATC-Center")
        self.interface.subscribe(self.message_recv)

        if self.cfg.Exists('left'):
            self.x = int(self.cfg.Read('left'))
            self.y = int(self.cfg.Read('top'))
            self.SetPosition(wx.Point(self.x,self.y), wx.SIZE_USE_EXISTING)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()
