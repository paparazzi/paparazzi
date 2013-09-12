#!/usr/bin/env python

from __future__ import print_function

import pygtk
import gtk
pygtk.require('2.0')


import os
import shutil
import datetime
from fnmatch import fnmatch
import subprocess




class ConfChooser:

    # General Functions

    def update_combo(self,combo,list):
        combo.set_sensitive(False)
        combo.get_model().clear()
        for i in list:
            combo.append_text(i)
        combo.set_active(0)
        combo.set_sensitive(True)

    def update_label(self):
        r = subprocess.Popen("ls -altr ./conf/conf.xml", stdout=subprocess.PIPE,stderr=subprocess.STDOUT, shell=True).stdout.read()
        r = r.strip()
        self.explain.set_text("Currently set to: " + r)

    # CallBack Functions


    def find_conf_files(self):

        list_of_conf_files = []

        root = './conf/'
        pattern = "*conf.xml*"

        for path, subdirs, files in os.walk(root):
            for name in files:
                if fnmatch(name, pattern):
                    entry = os.path.join(path, name).replace("./conf/","")
                    if entry != "conf.xml":
                        list_of_conf_files.append(entry)

        self.update_combo(self.conf_file_combo,list_of_conf_files)


    def about(self):
        gui_dialogs.about(paparazzi.home_dir)

    def launch(self, widget):
        os.system("./paparazzi &");
        gtk.main_quit()

    def backupconf(self, use_personal=False):
        timestr = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M")
        if " -> " in self.explain.get_text():
            print("Symlink does not need backup");
        else:
            newname = "conf.xml." + timestr
            backup_file = os.path.join(self.conf_dir, newname)
            shutil.copyfile(self.conf_xml, backup_file)
            print("Made a backup: " + newname)

        if use_personal:
            conf_personal = os.path.join(self.conf_dir, "conf.xml.personal")
            conf_personal_backup = os.path.join(self.conf_dir, "conf.xml.personal." + timestr)
            if os.path.exists(conf_personal):
                print("Backup conf.xml.personal to conf.xml.personal." + timestr)
                shutil.copyfile(conf_personal, conf_personal_backup)

    def delete(self, widget):
        filename = os.path.join(self.conf_dir, self.conf_file_combo.get_active_text())
        # TODO: dialog: are you certain?
        os.remove(filename)
        self.find_conf_files()


    def accept(self, widget):
        self.backupconf()
        link_source = self.conf_file_combo.get_active_text()
        os.remove(self.conf_xml)
        os.symlink(link_source, self.conf_xml)
        self.update_label()
        self.find_conf_files()

    def personal(self, widget):
        self.backupconf(True)
        template_file = os.path.join(self.conf_dir, self.conf_file_combo.get_active_text())
        personal_file = os.path.join(self.conf_dir, "conf.xml.personal")
        shutil.copyfile(template_file, personal_file)
        os.remove(self.conf_xml)
        os.symlink("conf.xml.personal", self.conf_xml)
        self.update_label()
        self.find_conf_files()

    # Constructor Functions

    def destroy(self, widget, data=None):
        gtk.main_quit()

    def __init__(self):
        self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.window.set_title("Paparazzi Configuration Chooser")

        self.my_vbox = gtk.VBox()

        # if PAPARAZZI_HOME not set, then assume the tree containing this
        # file is a reasonable substitute
        self.paparazzi_home = os.getenv("PAPARAZZI_HOME", os.path.dirname(os.path.abspath(__file__)))
        self.conf_dir = os.path.join(self.paparazzi_home, "conf")
        self.conf_xml = os.path.join(self.conf_dir, "conf.xml")

        # MenuBar
        mb = gtk.MenuBar()

        # File
        filemenu = gtk.Menu()

        # File Title
        filem = gtk.MenuItem("File")
        filem.set_submenu(filemenu)

        exitm = gtk.MenuItem("Exit")
        exitm.connect("activate", gtk.main_quit)
        filemenu.append(exitm)

        mb.append(filem)

        # Help
        helpmenu = gtk.Menu()

        # Help Title
        helpm = gtk.MenuItem("Help")
        helpm.set_submenu(helpmenu)

        aboutm = gtk.MenuItem("About")
        aboutm.connect("activate", self.about)
        helpmenu.append(aboutm)

        mb.append(helpm)

        self.my_vbox.pack_start(mb,False)

        # Combo Bar

        self.conf_label = gtk.Label("Conf:")

        self.conf_file_combo = gtk.combo_box_entry_new_text()
        self.find_conf_files()
#        self.firmwares_combo.connect("changed", self.parse_list_of_airframes)
        self.conf_file_combo.set_size_request(600,30)

        self.confbar = gtk.HBox()
        self.confbar.pack_start(self.conf_label)
        self.confbar.pack_start(self.conf_file_combo)
        self.my_vbox.pack_start(self.confbar, False)

        ##### Explain current config

        self.explain = gtk.Label("");
        self.update_label()

        self.exbar = gtk.HBox()
        self.exbar.pack_start(self.explain)

        self.my_vbox.pack_start(self.exbar, False)

        ##### Buttons
        self.btnAccept = gtk.Button("Set Selected Conf As Active")
        self.btnAccept.connect("clicked", self.accept)
        self.btnAccept.set_tooltip_text("Set Conf as Active")

        self.btnPersonal = gtk.Button("Create Personal Conf Based on Selected")
        self.btnPersonal.connect("clicked", self.personal)
        self.btnPersonal.set_tooltip_text("Create Personal Conf Based on Selected and Activate")

        self.btnDelete = gtk.Button("Delete Selected")
        self.btnDelete.connect("clicked", self.delete)
        self.btnDelete.set_tooltip_text("Permanently Delete")

        self.btnLaunch = gtk.Button("Launch Paparazzi")
        self.btnLaunch.connect("clicked", self.launch)
        self.btnLaunch.set_tooltip_text("Launch Paparazzi with current conf.xml")

        self.btnExit = gtk.Button("Exit")
        self.btnExit.connect("clicked", self.destroy)
        self.btnExit.set_tooltip_text("Close application")


        self.toolbar = gtk.HBox()
        self.toolbar.pack_start(self.btnAccept)
        self.toolbar.pack_start(self.btnPersonal)
        self.toolbar.pack_start(self.btnDelete)
        self.toolbar.pack_start(self.btnLaunch)
        self.toolbar.pack_start(self.btnExit)

        self.my_vbox.pack_start(self.toolbar, False)

        ##### Bottom

        self.window.add(self.my_vbox)
        self.window.show_all()
        self.window.set_position(gtk.WIN_POS_CENTER_ALWAYS)
        self.window.connect("destroy", self.destroy)

    def main(self):
        gtk.main()

if __name__ == "__main__":
    import sys
    if (len(sys.argv) > 1):
        airframe_file = sys.argv[1]
    gui = ConfChooser()
    gui.main()
