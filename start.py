#!/usr/bin/env python3

from __future__ import print_function

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GdkPixbuf


import os
import shutil
import datetime
import subprocess
import sys
import lsb_release

lib_path = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sw', 'lib', 'python'))
sys.path.append(lib_path)

import paparazzi
from paparazzi_health import PaparazziOverview

import xml.etree.ElementTree

release = lsb_release.get_distro_information()
isLegacy = False
if release['ID'] == 'Ubuntu':
    if release['RELEASE'] == '18.04' or release['RELEASE'] == '16.04':
        isLegacy = True


class ConfChooser(object):

    # General Functions

    def update_combo(self, combo, clist, active):
        combo.set_sensitive(False)
        combo.get_model().clear()
        current_index = 0
        combo.append_text("------None Selected------")
        for (i, text) in enumerate(clist):
            combo.append_text(text)
            if active is not None and os.path.join(paparazzi.conf_dir, text) == os.path.realpath(active):
                current_index = i + 1  # Add one due to ---None Selected---
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
            desc += os.path.relpath(real_conf_path, paparazzi.conf_dir)
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
            desc += os.path.relpath(real_conf_path, paparazzi.conf_dir)
        self.controlpanel_explain.set_text(desc)

    # CallBack Functions

    def find_conf_files(self, combo):
        conf_files = paparazzi.get_list_of_conf_files(self.exclude_backups)
        self.update_combo(combo, conf_files, self.conf_xml)

    def find_board_files(self, combo):
        board_files = paparazzi.get_list_of_boards()
        self.update_combo(combo, board_files, None)

    def find_controlpanel_files(self):
        controlpanel_files = paparazzi.get_list_of_controlpanel_files(self.exclude_backups)
        self.update_combo(self.controlpanel_file_combo, controlpanel_files, self.controlpanel_xml)

    @staticmethod
    def count_airframes_in_conf(combo, conf_airframes):
        airframes = 0
        releases = 0
        if combo.get_active_text() is None:
            return
        desc = ""
        afile = os.path.join(paparazzi.conf_dir, combo.get_active_text())
        if os.path.exists(afile):
            e = xml.etree.ElementTree.parse(afile).getroot()
            for atype in e.findall('aircraft'):
                if airframes > 0:
                    desc += ", "
                #print(atype.get('name'))
                airframes += 1
                if (not atype.get('release') is None) & (not atype.get('release') == ""):
                    releases += 1
                    desc += '<a href="https://github.com/paparazzi/paparazzi/commit/' + atype.get('release') + '">' + atype.get('name') + "</a>"
                else:
                    desc += atype.get('name')
        desc = "<b>" + str(airframes) + " airframes:</b> " + desc
        conf_airframes.set_markup(desc)
        return

    def about(self, widget):
        about_d = Gtk.AboutDialog()
        about_d.set_program_name("Paparazzi Configuration Selector")
        about_d.set_version("1.1")
        about_d.set_copyright("(c) GPL v2")
        about_d.set_comments("Select the active configuration")
        about_d.set_website("http://paparazzi.github.com")
        about_d.set_logo(GdkPixbuf.Pixbuf.new_from_file(os.path.join(paparazzi.PAPARAZZI_HOME, "data/pictures/penguin_icon.png")))
        about_d.run()
        about_d.destroy()

    def sure(self, widget, filename):
        dialog = Gtk.MessageDialog(self.window, Gtk.DIALOG_DESTROY_WITH_PARENT, Gtk.MESSAGE_QUESTION, Gtk.BUTTONS_OK_CANCEL, "Are you sure you want to delete?")
        dialog.format_secondary_text("File: " + filename)
        response = dialog.run()
        ret = False
        if response == Gtk.RESPONSE_OK:
            ret = True
        dialog.destroy()
        return ret

    def set_backups(self, widget):
        self.exclude_backups = not widget.get_active()
        self.find_conf_files(self.conf_file_combo)
        self.find_controlpanel_files()

    def more_info(self, widget):
        self.obj.run()

    def show_untested(self, widget, data):
        self.obj.run(show_af_detail=False, show_untested=True,
                     show_airframes=data["Airframes"].get_active(),
                     show_flightplans=data["Flightplans"].get_active(),
                     show_boards=data["Boards"].get_active(),
                     show_modules=data["Modules"].get_active())

    def module_usage(self, widget, data):
        if data["Conf"].get_active() != 0:
            self.obj.airframe_module_overview(data["Conf"].get_active_text())
        elif data["Board"].get_active() != 0:
            self.obj.airframe_module_overview(data["Board"].get_active_text() + ".makefile")

    def launch(self, widget):
        self.accept(widget)

        args = ["./paparazzi"]

        if self.btnPythonGUI.get_active():
            args += ["-python"]

        if isLegacy:
            args += ["-legacy"]

        self.pp = subprocess.Popen(args)
        self.window.destroy()

    def backupconf(self, use_personal=False):
        timestr = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M")
        if os.path.islink(self.conf_xml):
            if self.verbose:
                self.print_status("Symlink does not need backup")
        else:
            if os.path.exists(self.conf_xml):
                newname = "conf.xml." + timestr
                backup_file = os.path.join(paparazzi.conf_dir, newname)
                shutil.copyfile(self.conf_xml, backup_file)
                self.print_status("Made a backup: " + newname)

        if use_personal:
            backup_name = self.conf_personal_name + "." + timestr
            conf_personal_backup = os.path.join(paparazzi.conf_dir, backup_name)
            if os.path.exists(self.conf_personal):
                self.print_status("Backup conf.xml.personal to " + backup_name)
                shutil.copyfile(self.conf_personal, conf_personal_backup)

    def backupcontrolpanel(self, use_personal=False):
        timestr = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M")
        if os.path.islink(self.controlpanel_xml):
            if self.verbose:
                self.print_status("Symlink does not need backup")
        else:
            if os.path.exists(self.controlpanel_xml):
                newname = "control_panel.xml." + timestr
                backup_file = os.path.join(paparazzi.conf_dir, newname)
                shutil.copyfile(self.controlpanel_xml, backup_file)
                self.print_status("Made a backup: " + newname)

        if use_personal:
            backup_name = self.controlpanel_personal_name + "." + timestr
            controlpanel_personal_backup = os.path.join(paparazzi.conf_dir, backup_name)
            if os.path.exists(self.controlpanel_personal):
                self.print_status("Backup control_panel.xml.personal to " + backup_name)
                shutil.copyfile(self.controlpanel_personal, controlpanel_personal_backup)

    def delete_conf(self, widget):
        filename = os.path.join(paparazzi.conf_dir, self.conf_file_combo.get_active_text())
        ret = self.sure(widget, filename)
        if ret:
            if os.path.exists(filename):
                os.remove(filename)
            self.update_conf_label()
            self.count_airframes_in_conf(self.conf_file_combo, self.conf_airframes)
            self.find_conf_files(self.conf_file_combo)
            self.print_status("Deleted: " + filename)

    def delete_controlpanel(self, widget):
        filename = os.path.join(paparazzi.conf_dir, self.controlpanel_file_combo.get_active_text())
        ret = self.sure(widget, filename)
        if ret:
            if os.path.exists(filename):
                os.remove(filename)
            self.update_controlpanel_label()
            self.find_controlpanel_files()
            self.print_status("Deleted: " + filename)

    def accept(self, widget):
        selected = self.conf_file_combo.get_active_text()
        if selected == "conf.xml":
            self.print_status("conf.xml is not a symlink, maybe you want to copy it to your personal file first?")
        else:
            self.backupconf()
            if os.path.islink(self.conf_xml) or os.path.exists(self.conf_xml):
                os.remove(self.conf_xml)
            os.symlink(selected, self.conf_xml)
            self.update_conf_label()
            self.count_airframes_in_conf(self.conf_file_combo, self.conf_airframes)
            self.find_conf_files(self.conf_file_combo)

        selected = self.controlpanel_file_combo.get_active_text()
        if selected == "control_panel.xml":
            self.print_status("control_panel.xml is not a symlink, maybe you want to copy it to your personal file first?")
        else:
            self.backupcontrolpanel()
            if os.path.islink(self.controlpanel_xml) or os.path.exists(self.controlpanel_xml):
                os.remove(self.controlpanel_xml)
            os.symlink(selected, self.controlpanel_xml)
            self.update_controlpanel_label()
            self.find_controlpanel_files()

    def personal_conf(self, widget):
        if os.path.exists(self.conf_personal):
            self.print_status("Your personal conf file already exists!")
        else:
            self.backupconf(True)
            template_file = os.path.join(paparazzi.conf_dir, self.conf_file_combo.get_active_text())
            shutil.copyfile(template_file, self.conf_personal)
            os.remove(self.conf_xml)
            os.symlink(self.conf_personal_name, self.conf_xml)
            self.update_conf_label()
            self.count_airframes_in_conf(self.conf_file_combo, self.conf_airframes)
            self.find_conf_files(self.conf_file_combo)

    def personal_controlpanel(self, widget):
        if os.path.exists(self.controlpanel_personal):
            self.print_status("Your personal control_panel file already exists!")
        else:
            self.backupcontrolpanel(True)
            template_file = os.path.join(paparazzi.conf_dir, self.controlpanel_file_combo.get_active_text())
            shutil.copyfile(template_file, self.controlpanel_personal)
            os.remove(self.controlpanel_xml)
            os.symlink(self.controlpanel_personal_name, self.controlpanel_xml)
            self.update_controlpanel_label()
            self.find_controlpanel_files()

    def print_status(self, text):
        self.statusbar.push(self.context_id, text)

    def changed_cb(self, widget, data):
        self.count_airframes_in_conf(data["combo"], data["list"])

    @staticmethod
    def deactivate_cb(widget, combo):
        current_selection = widget.get_active()
        combo.set_active(0)
        widget.set_active(current_selection)

    def maintenance_window(self, widget):
        mtn_window = Gtk.Window()
        mtn_window.set_position(Gtk.WindowPosition.CENTER)
        mtn_window.set_size_request(750, 360)
        mtn_window.set_title("Maintenance Tools")

        mnt_vbox = Gtk.VBox()

        mnt_desc_label = Gtk.Label(label="")
        desc_text = "Show module usage of all airframes in a selected conf file <b>or</b> all airframes " \
                    "with a specific board across all conf files."
        mnt_desc_label.set_markup(desc_text)
        mnt_desc_label.set_size_request(720, 40)
        mnt_desc_label.set_line_wrap(True)

        mnt_conf_label = Gtk.Label(label="Conf:")
        mnt_conf_label.set_size_request(100, 30)
        mnt_conf_file_combo = Gtk.ComboBoxText.new()
        self.find_conf_files(mnt_conf_file_combo)
        mnt_conf_file_combo.set_size_request(500, 30)

        mnt_board_label = Gtk.Label(label="Board:")
        mnt_board_label.set_size_request(100, 30)
        mnt_board_file_combo = Gtk.ComboBoxText.new()
        self.find_board_files(mnt_board_file_combo)
        mnt_board_file_combo.set_size_request(500, 30)

        mnt_conf_airframes = Gtk.Label(label="")
        self.count_airframes_in_conf(mnt_conf_file_combo, mnt_conf_airframes)
        mnt_conf_airframes.set_size_request(650, 180)
        mnt_conf_airframes.set_line_wrap(True)

        combo_list = {"combo": mnt_conf_file_combo, "list": mnt_conf_airframes}
        mnt_conf_file_combo.connect("changed", self.changed_cb, combo_list)
        mnt_conf_file_combo.connect("changed", self.deactivate_cb, mnt_board_file_combo)

        mnt_board_file_combo.connect("changed", self.deactivate_cb, mnt_conf_file_combo)

        mnt_combos = {"Conf": mnt_conf_file_combo, "Board": mnt_board_file_combo}

        mnt_confbar = Gtk.HBox()
        mnt_confbar.pack_start(mnt_conf_label, True, True, 0)
        mnt_confbar.pack_start(mnt_conf_file_combo, True, True, 0)
        mnt_boardbar = Gtk.HBox()
        mnt_boardbar.pack_start(mnt_board_label, True, True, 0)
        mnt_boardbar.pack_start(mnt_board_file_combo, True, True, 0)
        mnt_vbox.pack_start(mnt_desc_label, True, True, 0)
        mnt_vbox.pack_start(mnt_confbar, False, True, 0)
        mnt_vbox.pack_start(mnt_boardbar, False, True, 0)

        btnModule = Gtk.Button(label="Module\nUsage")
        plop = Gtk.Button()
        btnModule.connect("clicked", self.module_usage, mnt_combos)
        btnModule.set_tooltip_text("More information on the modules used by these airframes")

        mnt_caexbar = Gtk.HBox()
        mnt_caexbar.pack_start(mnt_conf_airframes, True, True, 0)
        mnt_caexbar.pack_start(btnModule, True, True, 0)
        mnt_vbox.pack_start(mnt_caexbar, True, True, 0)

        separator = Gtk.HSeparator()
        mnt_vbox.pack_start(separator, True, True, 0)

        cbtnAirframes = Gtk.CheckButton()
        cbtnAirframes.set_label("Airframes")
        cbtnAirframes.set_active(True)

        cbtnFlightplans = Gtk.CheckButton()
        cbtnFlightplans.set_label("Flight plans")
        cbtnFlightplans.set_active(True)

        cbtnBoards = Gtk.CheckButton()
        cbtnBoards.set_label("Boards")
        cbtnBoards.set_active(True)

        cbtnModules = Gtk.CheckButton()
        cbtnModules.set_label("Modules")
        cbtnModules.set_active(False)

        selectedOptions = {"Airframes": cbtnAirframes, "Flightplans": cbtnFlightplans,
                           "Boards": cbtnBoards, "Modules": cbtnModules}

        btnUntested = Gtk.Button(label="Show Untested Files")
        btnUntested.connect("clicked", self.show_untested, selectedOptions)
        btnUntested.set_tooltip_text("For the selected options show the files not tested by any conf")

        untestedHBox = Gtk.HBox()
        cbtnVBox = Gtk.VBox()
        cbRowUpper = Gtk.HBox()
        cbRowLower = Gtk.HBox()

        cbRowUpper.pack_start(cbtnAirframes, True, True, 0)
        cbRowUpper.pack_start(cbtnBoards, True, True, 0)
        cbRowLower.pack_start(cbtnFlightplans, True, True, 0)
        cbRowLower.pack_start(cbtnModules, True, True, 0)
        cbtnVBox.pack_start(cbRowUpper, True, True, 0)
        cbtnVBox.pack_start(cbRowLower, True, True, 0)
        untestedHBox.pack_start(cbtnVBox, True, True, 0)
        untestedHBox.pack_start(btnUntested, True, True, 0)

        mnt_vbox.pack_start(untestedHBox, True, True, 0)

        mtn_window.add(mnt_vbox)
        mtn_window.show_all()

    # Constructor Functions
    def __init__(self):
        self.obj = PaparazziOverview(0)

        # paparazzi process
        self.pp = None

        self.window = Gtk.Window(type=Gtk.WindowType.TOPLEVEL)
        self.window.set_title("Paparazzi Configuration Chooser")

        self.my_vbox = Gtk.VBox()

        self.conf_xml = os.path.join(paparazzi.conf_dir, "conf.xml")
        self.conf_personal_name = "conf_personal.xml"
        self.conf_personal = os.path.join(paparazzi.conf_dir, self.conf_personal_name)

        self.controlpanel_xml = os.path.join(paparazzi.conf_dir, "control_panel.xml")
        self.controlpanel_personal_name = "control_panel_personal.xml"
        self.controlpanel_personal = os.path.join(paparazzi.conf_dir, self.controlpanel_personal_name)

        self.exclude_backups = True
        self.verbose = False

        # MenuBar
        mb = Gtk.MenuBar()

        # File
        filemenu = Gtk.Menu()

        # File Title
        filem = Gtk.MenuItem(label="File")
        filem.set_submenu(filemenu)

        exitm = Gtk.MenuItem(label="Exit")
        exitm.connect("activate", Gtk.main_quit)
        filemenu.append(exitm)

        mb.append(filem)

        # Help
        helpmenu = Gtk.Menu()

        # Help Title
        helpm = Gtk.MenuItem(label="Help")
        helpm.set_submenu(helpmenu)

        aboutm = Gtk.MenuItem(label="About")
        aboutm.connect("activate", self.about)
        helpmenu.append(aboutm)

        mb.append(helpm)

        self.my_vbox.pack_start(mb, False, True, 0)

        # Combo Bar

        self.conf_label = Gtk.Label(label="Conf:")
        self.conf_label.set_size_request(100, 30)

        self.conf_file_combo = Gtk.ComboBoxText.new()
        self.find_conf_files(self.conf_file_combo)
        self.conf_file_combo.set_size_request(550, 30)

        self.btnDeleteConf = Gtk.Button.new_with_label(label="Delete")
        self.btnDeleteConf.connect("clicked", self.delete_conf)
        self.btnDeleteConf.set_tooltip_text("Permanently Delete Conf")

        self.btnPersonalConf = Gtk.Button.new_with_label(label="Copy")
        self.btnPersonalConf.connect("clicked", self.personal_conf)
        self.btnPersonalConf.set_tooltip_text("Create Personal Conf Based on Selected and Activate")

        self.confbar = Gtk.HBox()
        self.confbar.pack_start(self.conf_label, True, True, 0)
        self.confbar.pack_start(self.conf_file_combo, True, True, 0)
        self.confbar.pack_start(self.btnDeleteConf, True, True, 0)
        self.confbar.pack_start(self.btnPersonalConf, True, True, 0)
        self.my_vbox.pack_start(self.confbar, False, True, 0)


        # Explain current conf config

        self.conf_explain = Gtk.Label(label="")
        self.update_conf_label()
        self.conf_explain.set_size_request(0, 45)

        self.cfexbar = Gtk.HBox()
        self.cfexbar.pack_start(self.conf_explain, True, True, 0)

        self.my_vbox.pack_start(self.cfexbar, False, True, 0)

        # Count Airframes
        self.conf_airframes = Gtk.Label(label="")
        self.count_airframes_in_conf(self.conf_file_combo, self.conf_airframes)
        self.conf_airframes.set_size_request(650, 180)
        self.conf_airframes.set_line_wrap(True)

        self.combo_list = {"combo": self.conf_file_combo, "list": self.conf_airframes}
        self.conf_file_combo.connect("changed", self.changed_cb, self.combo_list)

        self.btnInfo = Gtk.Button(label="More\nInfo")
        self.btnInfo.connect("clicked", self.more_info)
        self.btnInfo.set_tooltip_text("More information on airframe files")

        self.btnMaintenance = Gtk.Button(label="Maintenance\n\tTools")
        self.btnMaintenance.connect("clicked", self.maintenance_window)
        self.btnMaintenance.set_tooltip_text("Show maintenance tools")

        self.caexbar = Gtk.HBox()
        self.caexbar.pack_start(self.conf_airframes, True, True, 0)
        self.caexbar_btns = Gtk.VBox()
        self.caexbar.pack_start(self.caexbar_btns, True, True, 0)
        self.caexbar_btns.pack_start(self.btnInfo, True, True, 0)
        self.caexbar_btns.pack_start(self.btnMaintenance, True, True, 0)

        self.my_vbox.pack_start(self.caexbar, False, True, 0)

        # Controlpanel
        self.controlpanel_label = Gtk.Label(label="Controlpanel:")
        self.controlpanel_label.set_size_request(100, 30)

        self.controlpanel_file_combo = Gtk.ComboBoxText.new()
        self.find_controlpanel_files()
        self.controlpanel_file_combo.set_size_request(550, 30)

        # window

        self.btnDeleteControl = Gtk.Button.new_with_label(label="Delete")
        self.btnDeleteControl.connect("clicked", self.delete_controlpanel)
        self.btnDeleteControl.set_tooltip_text("Permanently Delete")

        self.btnPersonalControl = Gtk.Button.new_with_label(label="Copy")
        self.btnPersonalControl.connect("clicked", self.personal_controlpanel)
        self.btnPersonalControl.set_tooltip_text("Create Personal Controlpanel Based on Selected and Activate")

        self.controlpanelbar = Gtk.HBox(homogeneous=False)
        self.controlpanelbar.pack_start(self.controlpanel_label, True, True, 0)
        self.controlpanelbar.pack_start(self.controlpanel_file_combo, True, True, 0)
        self.controlpanelbar.pack_start(self.btnDeleteControl, True, True, 0)
        self.controlpanelbar.pack_start(self.btnPersonalControl, True, True, 0)
        self.my_vbox.pack_start(self.controlpanelbar, False, True, 0)

        # Explain current controlpanel config

        self.controlpanel_explain = Gtk.Label(label="")
        self.update_controlpanel_label()
        self.controlpanel_explain.set_size_request(0, 45)

        self.ctexbar = Gtk.HBox()
        self.ctexbar.pack_start(self.controlpanel_explain, True, True, 0)

        self.my_vbox.pack_start(self.ctexbar, False, True, 0)

        # show backups button
        self.btnBackups = Gtk.CheckButton(label="show backups")
        self.btnBackups.connect("toggled", self.set_backups)
        self.my_vbox.pack_start(self.btnBackups, False, True, 0)

        # show gui button
        self.btnPythonGUI = Gtk.CheckButton(label="new python center (beta)")
        self.my_vbox.pack_start(self.btnPythonGUI, False, True, 0)

        # Buttons
        self.btnAccept = Gtk.Button(label="Set Active")
        self.btnAccept.connect("clicked", self.accept)
        self.btnAccept.set_tooltip_text("Set selected Conf/Control_Panel as Active")

        self.btnLaunch = Gtk.Button(label="Launch Paparazzi with selected configuration")
        self.btnLaunch.connect("clicked", self.launch)
        self.btnLaunch.set_tooltip_text("Launch Paparazzi with current conf.xml and control_panel.xml")

        self.btnExit = Gtk.Button(label="Exit")
        self.btnExit.connect("clicked", Gtk.main_quit)
        self.btnExit.set_tooltip_text("Close application")

        self.toolbar = Gtk.HBox()
        self.toolbar.set_size_request(0, 60)
        self.toolbar.pack_start(self.btnLaunch, True, True, 0)
        self.toolbar.pack_start(self.btnAccept, True, True, 0)
        self.toolbar.pack_start(self.btnExit, True, True, 0)

        self.my_vbox.pack_start(self.toolbar, False, True, 0)

        # status bar
        self.statusbar = Gtk.Statusbar()
        self.context_id = self.statusbar.get_context_id("info")
        #self.statusbar.push(self.context_id, "Waiting for you to do something...")
        self.my_vbox.pack_end(self.statusbar, False, True, 0)

        # Bottom

        self.window.add(self.my_vbox)
        self.window.show_all()
        self.window.set_position(Gtk.WindowPosition.CENTER_ALWAYS)
        self.window.connect("destroy", Gtk.main_quit)

    def main(self):
        Gtk.main()
        if self.pp:
            self.pp.wait()


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        airframe_file = sys.argv[1]
    gui = ConfChooser()
    gui.main()
