import wx

import sys
import os
import time
import threading
import math
import pynotify
import pygame.mixer

sys.path.append(os.getenv("PAPARAZZI_HOME") + "/sw/ext/pprzlink/lib/v1.0/python")

from pprzlink.ivy import IvyMessagesInterface

WIDTH = 150
HEIGHT = 40
UPDATE_INTERVAL = 250


class RadioWatchFrame(wx.Frame):
    def message_recv(self, ac_id, msg):
        if msg.name == "ROTORCRAFT_STATUS":
            self.rc_status = int(msg['rc_status'])
        if self.rc_status != 0 and not self.alertChannel.get_busy():
            self.warn_timer = wx.CallLater(5, self.rclink_alert)
            # else:
            # self.notification.close()

    def gui_update(self):
        self.rc_statusText.SetLabel(["OK", "LOST", "REALLY LOST"][self.rc_status])
        self.update_timer.Restart(UPDATE_INTERVAL)

    def rclink_alert(self):
        self.alertChannel.queue(self.alertSound)
        self.notification.show()
        time.sleep(5)

    def setFont(self, control):
        font = control.GetFont()
        size = font.GetPointSize()
        font.SetPointSize(size * 1.4)
        control.SetFont(font)

    def __init__(self):
        wx.Frame.__init__(self, id=-1, parent=None, name=u'RCWatchFrame',
                          size=wx.Size(WIDTH, HEIGHT), title=u'RC Status')
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        self.rc_statusText = wx.StaticText(self, -1, "UNKWN")

        pygame.mixer.init()
        self.alertSound = pygame.mixer.Sound("crossing.wav")
        self.alertChannel = pygame.mixer.Channel(False)

        self.setFont(self.rc_statusText)

        self.notification = pynotify.Notification("RC Link Warning!",
                                                  "RC Link status not OK!",
                                                  "dialog-warning")

        self.rc_status = -1

        pynotify.init("RC Status")

        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.rc_statusText, 1, wx.EXPAND)
        self.SetSizer(sizer)
        sizer.Layout()
        self.interface = IvyMessagesInterface("radiowatchframe")
        self.interface.subscribe(self.message_recv)
        self.update_timer = wx.CallLater(UPDATE_INTERVAL, self.gui_update)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()
