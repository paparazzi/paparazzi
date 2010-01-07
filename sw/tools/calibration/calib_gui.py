import wx

WIDTH = 240
HEIGHT = 120
UPDATE_INTERVAL = 250

class CalibGui(wx.Frame):

    def gui_update(self):
        self.batteryText.SetLabel("%0.1f V" % self.batteryVolts)
        self.batteryVolts += 0.025
        self.update_timer.Restart(UPDATE_INTERVAL)

    def __init__(self):
        wx.Frame.__init__(self, id=-1, parent=None, name=u'TestGui',
                          size=wx.Size(WIDTH, HEIGHT), title=u'TestGui')
        self.batteryText = wx.StaticText(self, -1, "V")
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.batteryText, 1, wx.EXPAND)
        self.SetSizer(sizer)
        sizer.Layout()
        self.update_timer = wx.CallLater(UPDATE_INTERVAL, self.gui_update)
        self.batteryVolts = 0.;
