#! /usr/bin/python
#

import pygtk
pygtk.require('2.0')
import gtk
import time
from ivy.std_api import *
import logging

class Base:
    def __init__(self):
        self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.window.connect("destroy", self.destroy)

        self.entry = gtk.Entry()
        self.entry.set_width_chars(120)
        self.entry.connect("key-release-event", self.key_release_event)
        self.entry.show()

        self.window.add(self.entry)
        self.window.show()
        self.ivy_init()
        self.ticks = 0

    def ontick(self):
        if self.ticks == 5:
            IvyStop()
        elif self.ticks <= 2:
            IvySendMsg("1 BAT " + self.text)
        self.ticks = self.ticks + 1

    def ivy_init(self):
        logging.getLogger('Ivy').setLevel(logging.WARN)
        IvyInit("Log Annotate",
                "Annotate Ready Msg",
                0)

    def key_release_event(self, widget, event, data=None):
        if event.string == '\r': # Return
            self.text = self.entry.get_text()
            self.destroy(self, None)
        if event.string == '\033': # Escape
            self.destroy(self, None)
        return False

    def delete_event(self, widget, event, data=None):
        return False

    def destroy(self, widget, data=None):
        gtk.main_quit()

    def main(self):
        IvyStart("")
        gtk.main()

        if self.text:
            timerid = IvyTimerRepeatAfter(0,     # number of time to be called
                                          100,  # delay in ms between calls
                                          self.ontick # handler to call
                                          )
        IvyMainLoop()

if __name__ == "__main__":
  base = Base()
  base.main()
