#!/usr/bin/env python

import wx
import getopt
import sys
import dashboardframe

class DashboardApp(wx.App):
    def OnInit(self):
        self.main = dashboardframe.DashboardFrame()
        self.main.Show()
        self.SetTopWindow(self.main)
        return True

def main():
    application = DashboardApp(0)
    application.MainLoop()

if __name__ == '__main__':
    main()
