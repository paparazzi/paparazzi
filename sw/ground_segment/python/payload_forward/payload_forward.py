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
import threading
import socket
import array
import jpeg100_decoder
from io import StringIO


PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../..')))

sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface

WIDTH = 300


# Minimal Decoder
class MinimalDecoder:
    def __init__(self):
        self.data = []
    def add_payload(self,bytes):
        self.data = bytes

    def draw(self,dc,x,y):
        dc.DrawText( "Payload: " + str(self.data),x,y)


class PayloadForwarderFrame(wx.Frame):

    def message_recv(self, ac_id, msg):
        if msg.name == "PAYLOAD":
            # convert text to binary
            pld = msg.get_field(0).split(",")
            b = []
            for p in pld:
                b.append(int(p))

            # forward over UDP
            self.data['packets'] = self.data['packets'] + 1
            self.data['bytes'] = self.data['bytes'] + len(b)
            self.sock.sendto(bytearray(b), (self.settings.ip, self.settings.port))

            # send to decoder
            self.decoder.add_payload(b)

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
        font = wx.Font(11, wx.FONTFAMILY_ROMAN, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD)

        dc.SetFont(font)
        dc.DrawText("UDP: " + self.settings.ip + ":" + str(self.settings.port),2,2)
        dc.DrawText("Data: " + str(self.data['packets']) + " packets, " + str(round(float(self.data['bytes'])/1024.0,2)) + "kb)",2,22)
        dc.DrawText("Decoder: " + self.settings.decoder ,2,42)

        # Payload visual representation
        self.decoder.draw(dc, 2, 62)


    def __init__(self, _settings):

        # Command line arguments
        self.settings = _settings

        # Statistics
        self.data = { 'packets': 0, 'bytes': 0}

        # Decoder
        if (self.settings.decoder == 'jpeg100'):
            self.decoder = jpeg100_decoder.ThumbNailFromPayload()
        else:
            self.decoder = MinimalDecoder()

        self.w = WIDTH
        self.h = WIDTH

        # Socket
        self.sock = socket.socket(socket.AF_INET, # Internet
socket.SOCK_DGRAM) # UDP

        # Frame
        wx.Frame.__init__(self, id=-1, parent=None, name=u'Payload Forwarding',
                          size=wx.Size(self.w, self.h), title=u'Payload Forwarding')
        ico = wx.Icon(os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)))) + "/payload.ico", wx.BITMAP_TYPE_ICO)
        self.SetIcon(ico)

        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        # IVY
        self.interface = IvyMessagesInterface("PayloadForwarder")
        self.interface.subscribe(self.message_recv)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()
