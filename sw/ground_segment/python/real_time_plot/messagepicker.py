#!/usr/bin/env python

from __future__ import absolute_import, print_function

import wx
import sys
import time
from os import path, getenv

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")

from ivy_msg_interface import IvyMessagesInterface
from pprz_msg.message import PprzMessage


class Message(PprzMessage):
    def __init__(self, class_name, name):
        super(Message, self).__init__(class_name, name)
        self.field_controls = []
        self.index = None
        self.last_seen = time.clock()


class Aircraft(object):
    def __init__(self, ac_id):
        self.ac_id = ac_id
        self.messages = {}
        self.messages_book = None


class MessagePicker(wx.Frame):
    def __init__(self, parent, callback, initIvy=True):
        wx.Frame.__init__(self, parent, name="MessagePicker", title=u'Message Picker', size=wx.Size(320,640))

        self.aircrafts = {}
        self.callback = callback

        self.tree = wx.TreeCtrl(self)
        self.root = self.tree.AddRoot("Telemetry")
        self.tree.Bind(wx.EVT_LEFT_DCLICK, self.OnDoubleClick)
        self.tree.Bind(wx.EVT_CHAR, self.OnKeyChar)
        self.Bind(wx.EVT_CLOSE, self.OnClose)
        self.message_interface = IvyMessagesInterface(self.msg_recv, initIvy)

    def OnClose(self, event):
        # if we have a parent (like the plotpanel) only hide instead of shutdown
        if self.GetParent() is not None:
            self.Hide()
        else:
            self.message_interface.shutdown()
            self.Destroy()

    def msg_recv(self, ac_id, msg):
        if msg.msg_class != "telemetry":
            return

        self.tree.Expand(self.root)
        if ac_id not in self.aircrafts:
            ac_node = self.tree.AppendItem(self.root, str(ac_id))
            self.aircrafts[ac_id] = Aircraft(ac_id)
            self.aircrafts[ac_id].messages_book = ac_node

        aircraft = self.aircrafts[ac_id]
        ac_node = aircraft.messages_book

        if msg.name not in aircraft.messages:
            msg_node = self.tree.AppendItem(ac_node, str(msg.name))
            self.tree.SortChildren(ac_node)
            aircraft.messages[msg.name] = Message("telemetry", msg.name)
            for field in aircraft.messages[msg.name].fieldnames:
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
