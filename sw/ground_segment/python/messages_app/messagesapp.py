#!/usr/bin/env python

import wx
import messagesframe


class MessagesApp(wx.App):
    def OnInit(self):
        self.main = messagesframe.MessagesFrame()
        self.main.Show()
        self.SetTopWindow(self.main)
        return True


def main():
    application = MessagesApp(0)
    application.MainLoop()

if __name__ == '__main__':
    main()
