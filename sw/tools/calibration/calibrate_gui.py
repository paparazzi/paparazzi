#!/usr/bin/env python

#  $Id$
#  Copyright (C) 2010 Antoine Drouin
#
# This file is part of Paparazzi.
#
# Paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# Paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.  
#

import pygtk
pygtk.require('2.0')
import gtk
import os

import sensor_calibration

class CalibrateGui:

    #
    # loads a log
    #
    def on_load_log(self, widget, data=None):
        print "Loading log"
        dialog = gtk.FileChooserDialog("Open..",
                               None,
                               gtk.FILE_CHOOSER_ACTION_OPEN,
                               (gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
                                gtk.STOCK_OPEN, gtk.RESPONSE_OK))
        dialog.set_default_response(gtk.RESPONSE_OK)
        pprz_home = os.environ.get("PAPARAZZI_HOME")
        if pprz_home != None:
            dialog.set_current_folder(pprz_home+"/var/logs")

        filter = gtk.FileFilter()
        filter.set_name("Logs")
        filter.add_mime_type("paparazzi/logs")
        filter.add_pattern("*.data")
        dialog.add_filter(filter)

        filter = gtk.FileFilter()
        filter.set_name("All files")
        filter.add_pattern("*")
        dialog.add_filter(filter)
        
        response = dialog.run()
        if response == gtk.RESPONSE_OK:
            print dialog.get_filename(), 'selected'
            ac_ids = sensor_calibration.get_ids_in_log(dialog.get_filename())
        elif response == gtk.RESPONSE_CANCEL:
            print 'Closed, no files selected'
        dialog.destroy()
        
    def delete_event(self, widget, event, data=None):
        print "delete event occurred"
        return False
    
    def destroy(self, widget, data=None):
        print "destroy signal occurred"
        gtk.main_quit()

    #
    # build gui
    #
    def build_gui(self):
        self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.window.connect("delete_event", self.delete_event)
        self.window.connect("destroy", self.destroy)
        self.window.set_title("Paparazzi Sensor Calibration")
        self.window.set_border_width(10)
        table = gtk.Table(2, 2, True)
        self.window.add(table)
        table.attach(gtk.Label("Log :"), 0, 1, 0, 1)
        self.label_log = gtk.Label("None")
        table.attach(self.label_log, 1, 2, 0, 1)
        self.button_load_log = gtk.Button("Load log")
        self.button_load_log.connect("clicked", self.on_load_log, None)
        table.attach(self.button_load_log, 1, 2, 1, 2)
        self.window.show_all()   

        
    def __init__(self):
        self.build_gui();

    def main(self):
        gtk.main()

if __name__ == "__main__":
    app = CalibrateGui()
    app.main()    
