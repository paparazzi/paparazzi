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

    def update_combo(self, combo, clist, active):
        combo.set_sensitive(False)
        combo.get_model().clear()
        current_index = 0
        for (i, text) in enumerate(clist):
            combo.append_text(text)
            if os.path.join(self.conf_dir, text) == os.path.realpath(active):
                current_index = i
        combo.set_active(current_index)
        combo.set_sensitive(True)

    def update_conf_label(self):
        desc = "Current conf: "
        if not os.path.lexists(self.conf_xml):
            desc += "does not exist"
        else:
            if os.path.islink(self.conf_xml):
                if os.path.exists(self.conf_xml):
                    desc += "symlink to "
                else:
                    desc += "broken symlink to "
            real_conf_path = os.path.realpath(self.conf_xml)
            desc += os.path.relpath(real_conf_path, self.conf_dir)
        self.conf_explain.set_text(desc)

    def update_controlpanel_label(self):
        desc = "Current control_panel: "
        if not os.path.lexists(self.controlpanel_xml):
            desc += "does not exist"
        else:
            if os.path.islink(self.controlpanel_xml):
                if os.path.exists(self.controlpanel_xml):
                    desc += "symlink to "
                else:
                    desc += "broken symlink to "
            real_conf_path = os.path.realpath(self.controlpanel_xml)
            desc += os.path.relpath(real_conf_path, self.conf_dir)
        self.controlpanel_explain.set_text(desc)

    # CallBack Functions

    def find_conf_files(self):
        conf_files = []
        pattern = "*conf[._-]*xml"
        backup_pattern = "*conf[._-]*xml.20[0-9][0-9]-[01][0-9]-[0-3][0-9]_*"
        excludes = ["%gconf.xml"]

        for path, subdirs, files in os.walk(self.conf_dir):
            for name in files:
                if self.exclude_backups and fnmatch(name, backup_pattern):
                    continue
                if fnmatch(name, pattern):
                    filepath = os.path.join(path, name)
                    entry = os.path.relpath(filepath, self.conf_dir)
                    if not os.path.islink(filepath) and entry not in excludes:
                        conf_files.append(entry)

        conf_files.sort()
        self.update_combo(self.conf_file_combo, conf_files, self.conf_xml)

    def find_controlpanel_files(self):
        controlpanel_files = []
        pattern = "*control_panel[._-]*xml"
        backup_pattern = "*control_panel[._-]*xml.20[0-9][0-9]-[01][0-9]-[0-3][0-9]_*"
        excludes = []

        for path, subdirs, files in os.walk(self.conf_dir):
            for name in files:
                if self.exclude_backups and fnmatch(name, backup_pattern):
                    continue
                if fnmatch(name, pattern):
                    filepath = os.path.join(path, name)
                    entry = os.path.relpath(filepath, self.conf_dir)
                    if not os.path.islink(filepath) and entry not in excludes:
                        controlpanel_files.append(entry)

        controlpanel_files.sort()
        self.update_combo(self.controlpanel_file_combo, controlpanel_files, self.controlpanel_xml)

    def about(self, widget):
        about_d = gtk.AboutDialog()
        about_d.set_program_name("Paparazzi Configuration Selector")
        about_d.set_version("1.0")
        about_d.set_copyright("(c) GPL v2")
        about_d.set_comments("Select the active configuration")
        about_d.set_website("http://paparazzi.github.com")
        about_d.set_logo(gtk.gdk.pixbuf_new_from_file(os.path.join(self.paparazzi_home, "data/pictures/penguin_icon.png")))
        about_d.run()
        about_d.destroy()

    def sure(self, widget, filename):
        dialog = gtk.MessageDialog(self.window, gtk.DIALOG_DESTROY_WITH_PARENT, gtk.MESSAGE_QUESTION, gtk.BUTTONS_OK_CANCEL, "Are you sure you want to delete?")
        dialog.format_secondary_text( "File: " + filename)
        response = dialog.run()
        ret = False
        if response == gtk.RESPONSE_OK:
            ret = True
        dialog.destroy()
        return ret

    def set_backups(self, widget):
        self.exclude_backups = not widget.get_active()
        self.find_conf_files()
        self.find_controlpanel_files()

    def launch(self, widget):
        self.accept(widget)
        self.pp = subprocess.Popen("./paparazzi")
        self.window.destroy()

    def backupconf(self, use_personal=False):
        timestr = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M")
        if os.path.islink(self.conf_xml):
            if self.verbose:
                print("Symlink does not need backup")
        else:
            if os.path.exists(self.conf_xml):
                newname = "conf.xml." + timestr
                backup_file = os.path.join(self.conf_dir, newname)
                shutil.copyfile(self.conf_xml, backup_file)
                print("Made a backup: " + newname)

        if use_personal:
            backup_name = self.conf_personal_name + "." + timestr
            conf_personal_backup = os.path.join(self.conf_dir, backup_name)
            if os.path.exists(self.conf_personal):
                print("Backup conf.xml.personal to " + backup_name)
                shutil.copyfile(self.conf_personal, conf_personal_backup)

    def backupcontrolpanel(self, use_personal=False):
        timestr = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M")
        if os.path.islink(self.controlpanel_xml):
            if self.verbose:
                print("Symlink does not need backup")
        else:
            if os.path.exists(self.controlpanel_xml):
                newname = "control_panel.xml." + timestr
                backup_file = os.path.join(self.conf_dir, newname)
                shutil.copyfile(self.controlpanel_xml, backup_file)
                print("Made a backup: " + newname)

        if use_personal:
            backup_name = self.controlpanel_personal_name + "." + timestr
            controlpanel_personal_backup = os.path.join(self.conf_dir, backup_name)
            if os.path.exists(self.controlpanel_personal):
                print("Backup control_panel.xml.personal to " + backup_name)
                shutil.copyfile(self.controlpanel_personal, controlpanel_personal_backup)

    def delete_conf(self, widget):
        filename = os.path.join(self.conf_dir, self.conf_file_combo.get_active_text())
        ret = self.sure(widget, filename)
        if ret:
            if os.path.exists(filename):
                os.remove(filename)
            self.update_conf_label()
            self.find_conf_files()
            print("Deleted: " + filename)

    def delete_controlpanel(self, widget):
        filename = os.path.join(self.conf_dir, self.controlpanel_file_combo.get_active_text())
        ret = self.sure(widget, filename)
        if ret:
            if os.path.exists(filename):
                os.remove(filename)
            self.update_controlpanel_label()
            self.find_controlpanel_files()
            print("Deleted: " + filename)

    def accept(self, widget):
        selected = self.conf_file_combo.get_active_text()
        if selected == "conf.xml":
            print("conf.xml is not a symlink, maybe you want to copy it to your personal file first?")
        else:
            self.backupconf()
            link_source = self.conf_file_combo.get_active_text()
            if os.path.islink(self.conf_xml) or os.path.exists(self.conf_xml):
                os.remove(self.conf_xml)
            os.symlink(selected, self.conf_xml)
            self.update_conf_label()
            self.find_conf_files()

        selected = self.controlpanel_file_combo.get_active_text()
        if selected == "control_panel.xml":
            print("control_panel.xml is not a symlink, maybe you want to copy it to your personal file first?")
        else:
            self.backupcontrolpanel()
            link_source = self.controlpanel_file_combo.get_active_text()
            if os.path.islink(self.controlpanel_xml) or os.path.exists(self.controlpanel_xml):
                os.remove(self.controlpanel_xml)
            os.symlink(selected, self.controlpanel_xml)
            self.update_controlpanel_label()
            self.find_controlpanel_files()

    def personal_conf(self, widget):
        if os.path.exists(self.conf_personal):
            print("Your personal conf file already exists!")
        else:
            self.backupconf(True)
            template_file = os.path.join(self.conf_dir, self.conf_file_combo.get_active_text())
            shutil.copyfile(template_file, self.conf_personal)
            os.remove(self.conf_xml)
            os.symlink(self.conf_personal_name, self.conf_xml)
            self.update_conf_label()
            self.find_conf_files()

    def personal_controlpanel(self, widget):
        if os.path.exists(self.controlpanel_personal):
            print("Your personal control_panel file already exists!")
        else:
            self.backupcontrolpanel(True)
            template_file = os.path.join(self.conf_dir, self.controlpanel_file_combo.get_active_text())
            shutil.copyfile(template_file, self.controlpanel_personal)
            os.remove(self.controlpanel_xml)
            os.symlink(self.controlpanel_personal_name, self.controlpanel_xml)
            self.update_controlpanel_label()
            self.find_controlpanel_files()

    # Constructor Functions
    def __init__(self):
        # paparazzi process
        self.pp = None

        self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.window.set_title("Paparazzi Configuration Chooser")

        self.my_vbox = gtk.VBox()

        # if PAPARAZZI_HOME not set, then assume the tree containing this
        # file is a reasonable substitute
        self.paparazzi_home = os.getenv("PAPARAZZI_HOME", os.path.dirname(os.path.abspath(__file__)))
        self.conf_dir = os.path.join(self.paparazzi_home, "conf")
        self.conf_xml = os.path.join(self.conf_dir, "conf.xml")
        self.conf_personal_name = "conf_personal.xml"
        self.conf_personal = os.path.join(self.conf_dir, self.conf_personal_name)

        self.controlpanel_xml = os.path.join(self.conf_dir, "control_panel.xml")
        self.controlpanel_personal_name = "control_panel_personal.xml"
        self.controlpanel_personal = os.path.join(self.conf_dir, self.controlpanel_personal_name)

        self.exclude_backups = True
        self.verbose = False

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
        self.conf_label.set_size_request(100, 30)

        self.conf_file_combo = gtk.combo_box_new_text()
        self.find_conf_files()
        # self.firmwares_combo.connect("changed", self.parse_list_of_airframes)
        self.conf_file_combo.set_size_request(550, 30)

        self.btnDeleteConf = gtk.Button(None, gtk.STOCK_DELETE)
        self.btnDeleteConf.connect("clicked", self.delete_conf)
        self.btnDeleteConf.set_tooltip_text("Permanently Delete Conf")

        self.btnPersonalConf = gtk.Button(None, gtk.STOCK_COPY)
        self.btnPersonalConf.connect("clicked", self.personal_conf)
        self.btnPersonalConf.set_tooltip_text("Create Personal Conf Based on Selected and Activate")

        self.confbar = gtk.HBox()
        self.confbar.pack_start(self.conf_label)
        self.confbar.pack_start(self.conf_file_combo)
        self.confbar.pack_start(self.btnDeleteConf)
        self.confbar.pack_start(self.btnPersonalConf)
        self.my_vbox.pack_start(self.confbar, False)

        # Explain current conf config

        self.conf_explain = gtk.Label("")
        self.update_conf_label()
        self.conf_explain.set_size_request(0, 45)

        self.cfexbar = gtk.HBox()
        self.cfexbar.pack_start(self.conf_explain)

        self.my_vbox.pack_start(self.cfexbar, False)

        # Controlpanel
        self.controlpanel_label = gtk.Label("Controlpanel:")
        self.controlpanel_label.set_size_request(100, 30)

        self.controlpanel_file_combo = gtk.combo_box_new_text()
        self.find_controlpanel_files()
        self.controlpanel_file_combo.set_size_request(550, 30)

        # window

        self.btnDeleteControl = gtk.Button(None, gtk.STOCK_DELETE)
        self.btnDeleteControl.connect("clicked", self.delete_controlpanel)
        self.btnDeleteControl.set_tooltip_text("Permanently Delete")

        self.btnPersonalControl = gtk.Button(None, gtk.STOCK_COPY)
        self.btnPersonalControl.connect("clicked", self.personal_controlpanel)
        self.btnPersonalControl.set_tooltip_text("Create Personal Controlpanel Based on Selected and Activate")

        self.controlpanelbar = gtk.HBox(False)
        self.controlpanelbar.pack_start(self.controlpanel_label)
        self.controlpanelbar.pack_start(self.controlpanel_file_combo)
        self.controlpanelbar.pack_start(self.btnDeleteControl)
        self.controlpanelbar.pack_start(self.btnPersonalControl)
        self.my_vbox.pack_start(self.controlpanelbar, False)

        # Explain current controlpanel config

        self.controlpanel_explain = gtk.Label("")
        self.update_controlpanel_label()
        self.controlpanel_explain.set_size_request(0, 45)

        self.ctexbar = gtk.HBox()
        self.ctexbar.pack_start(self.controlpanel_explain)

        self.my_vbox.pack_start(self.ctexbar, False)

        # show backups button
        self.btnBackups = gtk.CheckButton("show backups")
        self.btnBackups.connect("toggled", self.set_backups)
        self.my_vbox.pack_start(self.btnBackups, False)

        # Buttons
        self.btnAccept = gtk.Button("Set Active")
        self.btnAccept.connect("clicked", self.accept)
        self.btnAccept.set_tooltip_text("Set selected Conf/Control_Panel as Active")

        self.btnLaunch = gtk.Button("Launch Paparazzi with selected configuration")
        self.btnLaunch.connect("clicked", self.launch)
        self.btnLaunch.set_tooltip_text("Launch Paparazzi with current conf.xml and control_panel.xml")

        self.btnExit = gtk.Button("Exit")
        self.btnExit.connect("clicked", gtk.main_quit)
        self.btnExit.set_tooltip_text("Close application")

        self.toolbar = gtk.HBox()
        self.toolbar.set_size_request(0, 60)
        self.toolbar.pack_start(self.btnLaunch)
        self.toolbar.pack_start(self.btnAccept)
        self.toolbar.pack_start(self.btnExit)

        self.my_vbox.pack_start(self.toolbar, False)

        # Bottom

        self.window.add(self.my_vbox)
        self.window.show_all()
        self.window.set_position(gtk.WIN_POS_CENTER_ALWAYS)
        self.window.connect("destroy", gtk.main_quit)

    def main(self):
        gtk.main()
        if self.pp:
            self.pp.wait()

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        airframe_file = sys.argv[1]
    gui = ConfChooser()
    gui.main()
