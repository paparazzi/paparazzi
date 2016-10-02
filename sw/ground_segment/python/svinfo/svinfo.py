#!/usr/bin/env python

import wx
import svinfoviewer

class SVInfoFrame(wx.App):
    def OnInit(self):
        self.main = svinfoviewer.SVInfoFrame()
        self.main.Show()
        self.SetTopWindow(self.main)
        return True

def main():
    application = SVInfoFrame(0)
    application.MainLoop()

if __name__ == '__main__':
    main()
