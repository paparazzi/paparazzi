#!/usr/bin/env python2

from __future__ import print_function

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GdkPixbuf

from os import path


def filechooser(pathname):
    dialog = Gtk.FileChooserDialog("Open ...", None,
                                   Gtk.FileChooserAction.OPEN,
                                   (Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL,
                                    Gtk.STOCK_OPEN, Gtk.ResponseType.OK))

    dialog.set_default_response(Gtk.ResponseType.OK)
    dialog.set_current_folder(pathname)

    filter = Gtk.FileFilter()
    filter.set_name("Airframe File")
    filter.add_pattern("*.xml")
    dialog.add_filter(filter)

    response = dialog.run()
    filename = ""
    if response == Gtk.ResponseType.OK:
        filename = dialog.get_filename()
    elif response == Gtk.ResponseType.CANCEL:
        print("No file selected")

    dialog.destroy()
    return filename


def error_loading_xml(s):
    err_msg = Gtk.MessageDialog(None, Gtk.DIALOG_DESTROY_WITH_PARENT,
                                Gtk.MESSAGE_ERROR, Gtk.BUTTONS_CLOSE,
                                "Error Loading XML: " + s)
    err_msg.run()
    err_msg.destroy()


def about(home):
    about_d = Gtk.AboutDialog()
    about_d.set_program_name("Paparazzi Airframe Editor")
    about_d.set_version("0.1")
    about_d.set_copyright("(c) GPL v2")
    about_d.set_comments("Airframe Editor")
    about_d.set_website("http://paparazzi.github.io")
    about_d.set_logo(GdkPixbuf.Pixbuf.new_from_file(path.join(home, "data/pictures/penguin_icon.png")))
    about_d.run()
    about_d.destroy()

