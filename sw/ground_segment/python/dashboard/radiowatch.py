#!/usr/bin/env python

import wx
import radiowatchframe

class RadioWatchApp(wx.App):
    def OnInit(self):
        self.main = radiowatchframe.RadioWatchFrame()
        self.main.Show()
        self.SetTopWindow(self.main)
        return True

def main():
    application = RadioWatchApp(0)
    application.MainLoop()

if __name__ == '__main__':
    main()
