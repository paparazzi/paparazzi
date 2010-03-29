#!/usr/bin/env python

import wx
import sys
import os

import calib_gui

PPRZ_HOME = os.getenv("PAPARAZZI_HOME")
import code; code.interact(local=locals())
sys.path.append(PPRZ_HOME + "/sw/lib/python")

import messages_tool

class TestGui(wx.App):

    def message_recv(self, ac_id, name, values):
        print "foo "+str(ac_id)
        return

    def OnInit(self):
        self.main = calib_gui.CalibGui()
        self.main.Show()
        self.SetTopWindow(self.main)
        self.interface = messages_tool.IvyMessagesInterface(self.message_recv)
        return True

def main():
    application = TestGui(0)
    application.MainLoop()

if __name__ == '__main__':
    main()
