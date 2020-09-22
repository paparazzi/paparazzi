#!/usr/bin/env python3

import sys
import wx
import gvfframe

class MessagesApp(wx.App):
    def __init__(self, wtf, ac_id):
        self.ac_id = ac_id
        wx.App.__init__(self, wtf)

    def OnInit(self):
        self.main = gvfframe.GVFFrame(self.ac_id)

        self.main.Show()
        self.SetTopWindow(self.main)
        return True

def main():
    if len(sys.argv) != 2:
        print ("Usage: gvfApp id_aircraft")
        return
    id_ac = int(sys.argv[1])
    application = MessagesApp(0, id_ac)
    application.MainLoop()

if __name__ == '__main__':
    main()
