#!/usr/bin/env python
# coding: utf-8

r"""UI dialogs of the airframe editor"""

from __future__ import print_function

import gtk
from os.path import join

if gtk.pygtk_version < (2, 3, 90):
    print("Please upgrade your pygtk")
    raise SystemExit


def filechooser(pathname):
    r"""File choosing dialog for the airframe editor

    Parameters
    ----------
    pathname : str"""
    dialog = gtk.FileChooserDialog("Open ...",
                                   None,
                                   gtk.FILE_CHOOSER_ACTION_OPEN,
                                   (gtk.STOCK_CANCEL,
                                    gtk.RESPONSE_CANCEL,
                                    gtk.STOCK_OPEN,
                                    gtk.RESPONSE_OK))

    dialog.set_default_response(gtk.RESPONSE_OK)
    dialog.set_current_folder(pathname)

    filter_ = gtk.FileFilter()
    filter_.set_name("Airframe File")
    filter_.add_pattern("*.xml")
    dialog.add_filter(filter_)

    response = dialog.run()
    filename = ""
    if response == gtk.RESPONSE_OK:
        filename = dialog.get_filename()
    elif response == gtk.RESPONSE_CANCEL:
        print("No file selected")

    dialog.destroy()
    return filename


def error_loading_xml(s):
    r"""XML loading error message box

    Parameters
    ----------
    s : str

    """
    err_msg = gtk.MessageDialog(None,
                                gtk.DIALOG_DESTROY_WITH_PARENT,
                                gtk.MESSAGE_ERROR,
                                gtk.BUTTONS_CLOSE,
                                "Error Loading XML: " + s)
    err_msg.run()
    err_msg.destroy()


def about(home):
    r"""About box

    Parameters
    ----------
    home : str
        Path to paparazzi home

    """
    about_d = gtk.AboutDialog()
    about_d.set_program_name("Paparazzi Airframe Editor")
    about_d.set_version("0.1")
    about_d.set_copyright("(c) GPL v2")
    about_d.set_comments("Airframe Editor")
    about_d.set_website("http://paparazzi.github.io")
    icon_path = join(home, "data/pictures/penguin_icon.png")
    about_d.set_logo(gtk.gdk.pixbuf_new_from_file(icon_path))
    about_d.run()
    about_d.destroy()
