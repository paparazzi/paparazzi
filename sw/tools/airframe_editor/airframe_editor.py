#!/usr/bin/env python2

from __future__ import print_function

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GdkPixbuf


from os import path

# Owm Modules
import gui_dialogs
import xml_airframe

import os, sys
lib_path = os.path.abspath(os.path.join('..', '..', 'lib', 'python'))
print(lib_path)
sys.path.append(lib_path)

import paparazzi


# Airframe File
airframe_file = path.join(paparazzi.airframes_dir, "examples/quadrotor_lisa_m_2_pwm_spektrum.xml")


class AirframeEditor:

    # General Functions

    def load_airframe_xml(self):
        global airframe_file
        self.tvcolumn.set_title(airframe_file.replace(paparazzi.airframes_dir, ""))
        [e, self.xml, self.xml_header] = xml_airframe.load(airframe_file)
        if e:
            gui_dialogs.error_loading_xml(e.__str__())
            raise e
        xml_airframe.fill_tree(self.xml, self.treestore)

    def update_combo(self, combo, c_list):
        combo.set_sensitive(False)
        combo.get_model().clear()
        for i in c_list:
            combo.append_text(i)
        combo.set_active(0)
        combo.set_sensitive(True)

    # CallBack Functions

    def find_firmwares(self, widget):
        list_of_firmwares = paparazzi.get_list_of_firmwares()
        self.update_combo(self.firmwares_combo, list_of_firmwares)

    def find_modules(self, widget):
        list_of_modules = paparazzi.get_list_of_modules()
        self.update_combo(self.modules_combo, list_of_modules)

    def find_subsystems(self, widget):
        self.textbox.set_text(self.firmwares_combo.get_active_text())
        list_of_subsystems = paparazzi.get_list_of_subsystems(self.firmwares_combo.get_active_text())
        self.update_combo(self.subsystems_combo, list_of_subsystems)

    def find_boards(self, widget):
        list_of_boards = paparazzi.get_list_of_boards()
        self.update_combo(self.boards_combo, list_of_boards)

    def find_module_defines(self, widget):
        mod = paparazzi.get_module_information(self.modules_combo.get_active_text())
        print(mod.description)
        txt = mod.description + "\n"
        for d in mod.defines:
            txt += "define: " + d[0].__str__() + " = " + d[1].__str__() + "; [" + d[2].__str__() + "] // " + d[3].__str__() + "\n"
        for c in mod.configures:
            txt += "configure: " + c[0].__str__() + " = " + c[1].__str__() + "; [" + c[2].__str__() + "] // " + c[3].__str__() + "\n"
        self.text_box.set_text(txt)
        self.gridstore.clear()
        for d in mod.defines:
            self.gridstore.append(["define", d[0], d[1], d[2], d[3]])

    def reorganize_xml(self, widget):
        self.xml = xml_airframe.reorganize_airframe_xml(self.xml)
        xml_airframe.fill_tree(self.xml, self.treestore)

    def about(self, widget):
        gui_dialogs.about(paparazzi.PAPARAZZI_HOME)

    def open(self, widget):
        global airframe_file
        filename = gui_dialogs.filechooser(paparazzi.airframes_dir)
        if filename == "":
            print("No file selected")
            return
        airframe_file = filename
        self.load_airframe_xml()

    def search(self, widget):
        ret = paparazzi.search(self.textbox.get_text())
        self.text_box.set_text(ret)
        print(ret)

    # Tree Callbacks

    def select_section(self, widget):
        #get data from highlighted selection
        treeselection = self.datagrid.get_selection()
        (model, row_iter) = treeselection.get_selected()
        if row_iter is not None:
            name_of_data = self.gridstore.get_value(row_iter, 1)
            #print("Selected ",name_of_data)
            self.textbox.set_text(name_of_data)
            # xml_airframe.defines(self.treestore.get_value(row_iter, 1), self.gridstore)

    def select(self, widget):
        #get data from highlighted selection
        treeselection = self.treeview.get_selection()
        (model, row_iter) = treeselection.get_selected()
        if row_iter is not None:
            name_of_data = self.treestore.get_value(row_iter, 0)
            #print("Selected ",name_of_data)
            self.textbox.set_text(name_of_data)
            xml_airframe.defines(self.treestore.get_value(row_iter, 1), self.gridstore)

    # Constructor Functions

    def fill_tree_from_airframe(self):

        # create a TreeStore with one string column to use as the model
        self.treestore = Gtk.TreeStore(str, object)

        # create the TreeView using treestore
        self.treeview = Gtk.TreeView(self.treestore)

        # create the TreeViewColumn to display the data
        self.tvcolumn = Gtk.TreeViewColumn('')

        # add self.tvcolumn to treeview
        self.treeview.append_column(self.tvcolumn)
        self.treeview.connect("cursor-changed", self.select)
        self.cell = Gtk.CellRendererText()
        self.tvcolumn.pack_start(self.cell, True)
        self.tvcolumn.add_attribute(self.cell, 'text', 0)
        self.treeview.set_reorderable(True)

    def fill_datagrid_from_section(self):

        # create a TreeStore with one string column to use as the model
        self.gridstore = Gtk.ListStore(str, str, str, str, str)

        self.datagrid = Gtk.TreeView(self.gridstore)

        self.type_column = Gtk.TreeViewColumn('Type')
        self.name_column = Gtk.TreeViewColumn('Name')
        self.value_column = Gtk.TreeViewColumn('Value')
        self.unit_column = Gtk.TreeViewColumn('Unit')
        self.desc_column = Gtk.TreeViewColumn('Description')

        self.datagrid.append_column(self.type_column)
        self.datagrid.append_column(self.name_column)
        self.datagrid.append_column(self.value_column)
        self.datagrid.append_column(self.unit_column)
        self.datagrid.append_column(self.desc_column)
        self.datagrid.connect("cursor-changed", self.select_section)

        self.type_cell = Gtk.CellRendererText()
        self.type_cell.Editable = False
        self.name_cell = Gtk.CellRendererText()
        self.name_cell.Editable = False
        self.value_cell = Gtk.CellRendererText()
        self.value_cell.Editable = True
        self.value_cell.set_property("editable", True)
        self.unit_cell = Gtk.CellRendererText()
        self.unit_cell.Editable = False
        self.desc_cell = Gtk.CellRendererText()
        self.desc_cell.Editable = False

        self.type_column.pack_start(self.type_cell, True)
        self.type_column.add_attribute(self.type_cell, 'text', 0)
        self.name_column.pack_start(self.name_cell, True)
        self.name_column.add_attribute(self.name_cell, 'text', 1)
        self.value_column.pack_start(self.value_cell, True)
        self.value_column.add_attribute(self.value_cell, 'text', 2)
        self.unit_column.pack_start(self.unit_cell, True)
        self.unit_column.add_attribute(self.unit_cell, 'text', 3)
        self.desc_column.pack_start(self.desc_cell, True)
        self.desc_column.add_attribute(self.desc_cell, 'text', 4)

        self.datagrid.set_search_column(1)
        self.name_column.set_sort_column_id(0)
        self.datagrid.set_reorderable(True)


    def destroy(self, widget, data=None):
        Gtk.main_quit()

    def __init__(self):
        self.window = Gtk.Window(Gtk.WindowType.TOPLEVEL)
        self.window.set_title("Paparazzi Airframe File Editor")

        self.my_vbox = Gtk.VBox()

        # MenuBar
        mb = Gtk.MenuBar()

        # File
        filemenu = Gtk.Menu()

        # File Title
        filem = Gtk.MenuItem("File")
        filem.set_submenu(filemenu)

        openm = Gtk.MenuItem("Open")
        openm.connect("activate", self.open)
        filemenu.append(openm)

        exitm = Gtk.MenuItem("Exit")
        exitm.connect("activate", Gtk.main_quit)
        filemenu.append(exitm)

        mb.append(filem)

        # Help
        helpmenu = Gtk.Menu()

        # Help Title
        helpm = Gtk.MenuItem("Help")
        helpm.set_submenu(helpmenu)

        aboutm = Gtk.MenuItem("About")
        aboutm.connect("activate", self.about)
        helpmenu.append(aboutm)

        mb.append(helpm)

        self.my_vbox.pack_start(mb, False, True, 0)

        ##### Buttons
        self.btnExit = Gtk.Button("Exit")
        self.btnExit.connect("clicked", self.destroy)
        self.btnExit.set_tooltip_text("Close application")

        self.btnOpen = Gtk.Button("Open")
        self.btnOpen.connect("clicked", self.open)

        self.btnRun = Gtk.Button("Reorganize XML")
        self.btnRun.connect("clicked", self.reorganize_xml)

        self.btnFirmwares = Gtk.Button("Firmwares")
        self.btnFirmwares.connect("clicked", self.find_firmwares)

        self.btnSubSystem = Gtk.Button("SubSystems")
        self.btnSubSystem.connect("clicked", self.find_subsystems)

        self.btnModules = Gtk.Button("Add Modules")
        self.btnModules.connect("clicked", self.find_modules)

        self.btnModuleDefines = Gtk.Button("Define")
        self.btnModuleDefines.connect("clicked", self.find_module_defines)

        self.btnAbout = Gtk.Button("About")
        self.btnAbout.connect("clicked", self.about)

        self.toolbar = Gtk.HBox()
        self.toolbar.pack_start(self.btnOpen, True, True, 0)
        self.toolbar.pack_start(self.btnRun, True, True, 0)
        self.toolbar.pack_start(self.btnAbout, True, True, 0)
        self.toolbar.pack_start(self.btnExit, True, True, 0)

        self.my_vbox.pack_start(self.toolbar, False, True, 0)



        self.firmwares_combo = Gtk.ComboBoxText.new_with_entry()
        self.find_firmwares(self.firmwares_combo)
        self.firmwares_combo.connect("changed", self.find_subsystems)

        self.subsystems_combo = Gtk.ComboBoxText.new_with_entry()

        self.boards_combo = Gtk.ComboBoxText.new_with_entry()
        self.find_boards(self.boards_combo)


        self.firmwarebar = Gtk.HBox()
        self.firmwarebar.pack_start(self.btnFirmwares, True, True, 0)
        self.firmwarebar.pack_start(self.btnSubSystem, True, True, 0)
        self.firmwarebar.pack_start(self.firmwares_combo, True, True, 0)
        self.firmwarebar.pack_start(self.boards_combo, True, True, 0)
        self.firmwarebar.pack_start(self.subsystems_combo, True, True, 0)

        self.modules_combo = Gtk.ComboBoxText.new_with_entry()
        self.find_modules(self.modules_combo)
        self.modules_combo.connect("changed", self.find_module_defines)

        #self.modulebar = Gtk.HBox()
        self.firmwarebar.pack_start(self.btnModules, True, True, 0)
        self.firmwarebar.pack_start(self.btnModuleDefines, True, True, 0)
        self.firmwarebar.pack_start(self.modules_combo, True, True, 0)

        #self.my_vbox.pack_start(self.modulebar, True, True, 0)

        self.my_vbox.pack_start(self.firmwarebar, False, True, 0)




        ##### Middle

        self.editor = Gtk.HBox()

        self.fill_tree_from_airframe()

        self.scrolltree = Gtk.ScrolledWindow()
        self.scrolltree.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
        self.scrolltree.add(self.treeview)
        self.scrolltree.set_size_request(400,600)

        self.editor.pack_start(self.scrolltree, True, True, 0)

        self.fill_datagrid_from_section()
        self.datagrid.set_size_request(900, 600)
        self.editor.pack_start(self.datagrid, True, True, 0)

        self.my_vbox.pack_start(self.editor, True, True, 0)

        self.text_box = Gtk.Label("")
        self.text_box.set_size_request(600, 1000)

        self.scrolltext = Gtk.ScrolledWindow()
        self.scrolltext.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
        self.scrolltext.add_with_viewport(self.text_box)
        self.scrolltext.set_size_request(400, 100)

        self.my_vbox.pack_start(self.scrolltext, True, True, 0)

        self.load_airframe_xml()

        ##### Bottom

        self.searchbar = Gtk.HBox()

        self.textbox = Gtk.Entry()
        #self.textbox.connect("changed",self.textchanged)

        self.btnSearch = Gtk.Button("Search...")
        self.btnSearch.connect("clicked", self.search)

        self.searchbar.pack_start(self.textbox, True, True, 0)
        self.searchbar.pack_start(self.btnSearch, True, True, 0)

        self.my_vbox.pack_start(self.searchbar, False, True, 0)

        self.window.add(self.my_vbox)
        self.window.show_all()
        self.window.connect("destroy", self.destroy)

    def main(self):
        Gtk.main()

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        airframe_file = sys.argv[1]
    gui = AirframeEditor()
    gui.main()

