#!/usr/bin/env python

import wx
import getopt
import sys
import plotframe


class RealTimePlotApp(wx.App):
    def OnInit(self):
        self.main = plotframe.create(None)
        self.main.Show()
        self.SetTopWindow(self.main)
        opts, args = getopt.getopt(sys.argv[1:], "p:", ["plot"])
        for o, a in opts:
            if o in ("-p", "--plot"):
                [ac_id, message, field, color, use_x] = a.split(':')
                self.main.AddPlot(int(ac_id), message, field, color, bool(int(use_x)))
        return True


def main():
    application = RealTimePlotApp(0)
    application.MainLoop()

if __name__ == '__main__':
    main()
