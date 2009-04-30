import wx

import sys
import os
import time
import threading
import math
import pynotify

sys.path.append(os.getenv("PAPARAZZI_HOME") + "/sw/lib/python")

import messages_tool

WIDTH = 120
HEIGHT = 120
UPDATE_INTERVAL = 250
WARN_INTERVAL = 3000
BATTERY_WARN = 6.5
RADS_SEC_TO_RPM = 60 / (2 * math.pi)

class DashboardFrame(wx.Frame):
  def message_recv(self, ac_id, name, values):
    if name == "BAT":
      self.batteryVolts = float(values[1]) / 10
    elif name == "WHIRLY_MOTORCONTROL":
      self.throttle1 = int(values[0])
      self.throttle2 = int(values[0])
    elif name == "WHIRLY_IMU":
      self.rpmYaw = (float(values[3]) + float(values[12])) / 2
      self.rpmYaw = -self.rpmYaw * RADS_SEC_TO_RPM
      self.uptime = float(values[0])

  def gui_update(self):
    self.batteryText.SetLabel("%0.1f V" % self.batteryVolts)
    self.rpmTextYaw.SetLabel("%0.1f RPM" % self.rpmYaw)
    self.throttleText.SetLabel("%0.1i RPM %0.1i RPM" % (self.throttle1, self.throttle2))
    self.uptimeText.SetLabel("%02i:%02i:%02i" % (self.uptime / (60 * 60), (self.uptime / 60) % 60, self.uptime % 60))
    self.update_timer.Restart(UPDATE_INTERVAL)

  def battery_notify(self):
    if (self.batteryVolts < BATTERY_WARN and self.batteryVolts > 0):
      self.notification = pynotify.Notification("Battery Warning",
        "Paparazzi battery is at %s." % self.batteryText.GetLabel(),
        "dialog-warning")
      self.notification.show()
    self.warn_timer.Restart(WARN_INTERVAL)

  def setFont(self, control):
    font = control.GetFont()
    size = font.GetPointSize()
    font.SetPointSize(size * 1.4)
    control.SetFont(font)

  def __init__(self):
    wx.Frame.__init__(self, id=-1, parent=None, name=u'DashboardFrame',
      size=wx.Size(WIDTH, HEIGHT), title=u'Dashboard')
    self.aircrafts = {}

    self.batteryText = wx.StaticText(self, -1, "V")
    self.rpmTextYaw = wx.StaticText(self, -1, "RPM")
    self.throttleText = wx.StaticText(self, -1, "%")
    self.uptimeText = wx.StaticText(self, -1, "::")

    self.setFont(self.batteryText)
    self.setFont(self.rpmTextYaw)
    self.setFont(self.throttleText)
    self.setFont(self.uptimeText)

    self.batteryVolts = -1
    self.rpmYaw = -1
    self.throttle1 = -1
    self.throttle2 = -1
    self.uptime = 0

    pynotify.init("Paparazzi Dashboard")

    sizer = wx.BoxSizer(wx.VERTICAL)
    sizer.Add(self.batteryText, 1, wx.EXPAND)
    sizer.Add(self.rpmTextYaw, 1, wx.EXPAND)
    sizer.Add(self.throttleText, 1, wx.EXPAND)
    sizer.Add(self.uptimeText, 1, wx.EXPAND)
    self.SetSizer(sizer)
    sizer.Layout()
    self.interface = messages_tool.IvyMessagesInterface(self.message_recv)
    self.update_timer = wx.CallLater(UPDATE_INTERVAL, self.gui_update)
    self.warn_timer = wx.CallLater(WARN_INTERVAL, self.battery_notify)
