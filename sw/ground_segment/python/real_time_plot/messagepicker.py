#!/usr/bin/env python

from __future__ import absolute_import, print_function

import wx
import getopt
import sys
import os

sys.path.append(os.getenv("PAPARAZZI_HOME") + "/sw/lib/python")


import messages_tool

class MessagePicker(wx.Frame):
    def __init__(self, parent, callback, initIvy = True):
        wx.Frame.__init__(self, parent, name="MessagePicker", title=u'Message Picker', size=wx.Size(320,640))

        self.aircrafts = {}
        self.callback = callback

        self.tree = wx.TreeCtrl(self)
        self.root = self.tree.AddRoot("Telemetry")
        self.tree.Bind(wx.EVT_LEFT_DCLICK, self.OnDoubleClick)
        self.tree.Bind(wx.EVT_CHAR, self.OnKeyChar)
        self.Bind( wx.EVT_CLOSE, self.OnClose)
        self.message_interface = messages_tool.IvyMessagesInterface(self.msg_recv, initIvy)

    def OnClose(self, event):
        self.message_interface.Stop()
        self.Destroy()

    def msg_recv(self, ac_id, name, values):
        self.tree.Expand(self.root)
        if ac_id not in self.aircrafts:
            ac_node = self.tree.AppendItem(self.root, str(ac_id))
            self.aircrafts[ac_id] = messages_tool.Aircraft(ac_id)
            self.aircrafts[ac_id].messages_book = ac_node

        aircraft = self.aircrafts[ac_id]
        ac_node = aircraft.messages_book

        if name not in aircraft.messages:
            msg_node = self.tree.AppendItem(ac_node, str(name))
            self.tree.SortChildren(ac_node)
            aircraft.messages[name] = messages_tool.Message("telemetry", name)
            for field in aircraft.messages[name].field_names:
                item = self.tree.AppendItem(msg_node, field)

    def OnKeyChar(self, event):
        if event.GetKeyCode() != 13:
            return False
        node = self.tree.GetSelection()
        field_name = self.tree.GetItemText(node)

        parent = self.tree.GetItemParent(node)
        message_name = self.tree.GetItemText(parent)

        grandparent = self.tree.GetItemParent(parent)
        ac_id = self.tree.GetItemText(grandparent)

        if node == self.root or parent == self.root or grandparent == self.root:
            # if not leaf, double click = expand
            if self.tree.IsExpanded(node):
                self.tree.Collapse(node)
            else:
                self.tree.Expand(node)
            return

        self.callback(int(ac_id), message_name, field_name)

    def OnDoubleClick(self, event):
        node = self.tree.GetSelection()
        field_name = self.tree.GetItemText(node)

        parent = self.tree.GetItemParent(node)
        message_name = self.tree.GetItemText(parent)

        grandparent = self.tree.GetItemParent(parent)
        ac_id = self.tree.GetItemText(grandparent)

        if node == self.root or parent == self.root or grandparent == self.root:
            # if not leaf, double click = expand
            if self.tree.IsExpanded(node):
                self.tree.Collapse(node)
            else:
                self.tree.Expand(node)
            return

        self.callback(int(ac_id), message_name, field_name)

class TestApp(wx.App):
    def OnInit(self):
        self.main = MessagePicker(None, callback)
        self.main.Show()
        self.SetTopWindow(self.main)

        return True

def test():
    application = TestApp(0)
    application.MainLoop()

def callback(ac_id, message, field):
    print(ac_id, message, field)

if __name__ == '__main__':
    test()
