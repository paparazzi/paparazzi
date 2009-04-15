#!/usr/bin/env python

import wx
import getopt
import sys
import messagesframe

class MessagesApp(wx.App):
    def OnInit(self):
        self.main = messagesframe.MessagesFrame()
        self.main.Show()
        self.SetTopWindow(self.main)
        #opts, args = getopt.getopt(sys.argv[1:], "p:",
        #["plot"])
        #for o,a in opts:
          #if o in ("-p", "--plot"):
            #[ac_id, message, field, color, use_x] = a.split(':')
            #self.main.AddPlot(int(ac_id), message, field, color, bool(int(use_x)))
        return True

def main():
    application = MessagesApp(0)
    application.MainLoop()

if __name__ == '__main__':
    main()
