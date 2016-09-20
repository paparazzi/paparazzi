import wx

import sys
import os
import time
import threading
import math
import pynotify
#import pygame.mixer

sys.path.append(os.getenv("PAPARAZZI_HOME") + "/sw/ext/pprzlink/lib/v1.0/python")

from pprzlink.ivy import IvyMessagesInterface

WIDTH = 300
HEIGHT = 300

BARH = 140
CHANNEL = 16

#UPDATE_INTERVAL = 250


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
                
            #self.gui_update();
            #print('----------------------------')
            #for s in self.sv:
            #    e = self.sv[s]
            #    print(s,e.QI, e.CNO, e.SVID)
            wx.CallAfter(self.update)

    def update(self):
        self.Refresh()

    def OnPaint(self, e):
        w = WIDTH
        h = HEIGHT

        tdx = -5
        tdy = -7
        th = 15

        dc = wx.PaintDC(self) 
        brush = wx.Brush("white")  
        dc.SetBackground(brush)  
        dc.Clear() 
        
        # Background
        dc.SetBrush(wx.Brush(wx.Colour(0,0,0), wx.TRANSPARENT)) 
        dc.DrawCircle(w/2,h/2,w/2-1) 
        dc.DrawCircle(w/2,h/2,w/4-1) 
        dc.DrawCircle(w/2,h/2,1) 
        font = wx.Font(11, wx.ROMAN, wx.BOLD, wx.NORMAL) 
        dc.SetFont(font)

        dc.DrawText("N",w/2+tdx,2) 
        dc.DrawText("S",w/2+tdx,h-17) 
        dc.DrawText("E",w-15,h/2+tdy) 
        dc.DrawText("W",2,h/2+tdy) 

        # SV
        for chn in self.sv:
            sv = self.sv[chn]

            c = wx.Colour(255,0,0)
            if (sv.QI > 0):
                c = wx.Colour(0,128,128)
                if (sv.QI >= 7):
                    c = wx.Colour(0,255,0)

            el = float(sv.Elev) / 90.0 * float(w) / 2.0
            az = float(sv.Azim) * math.pi / 180.0

            y = float(w)/2.0 - math.cos(az) * el
            x = float(w)/2.0 + math.sin(az) * el

            dc.SetBrush(wx.Brush(c, wx.SOLID)) 
            dc.DrawCircle(int(x),int(y),10) 
		
            font = wx.Font(8, wx.ROMAN, wx.NORMAL, wx.NORMAL) 
            dc.SetFont(font) 
            dc.DrawText(str(sv.SVID),x+tdx,y+tdy)

            h = float(BARH-th-th) * float(sv.CNO) / 55.0
            dc.DrawRectangle(w/CHANNEL*chn,HEIGHT+BARH-th-th-h,w/CHANNEL-2,h)
            dc.DrawText(str(chn),w/CHANNEL*chn,HEIGHT+BARH-th-th)
            dc.DrawText(str(sv.CNO),w/CHANNEL*chn,HEIGHT+BARH-th)
		

    def __init__(self):
        wx.Frame.__init__(self, id=-1, parent=None, name=u'SVInfoFrame',
                          size=wx.Size(WIDTH, HEIGHT+BARH), title=u'SV Info')
        self.Bind(wx.EVT_PAINT, self.OnPaint) 
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        self.sv = {}

        self.interface = IvyMessagesInterface("svinfoframe")
        self.interface.subscribe(self.message_recv)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()
