#!/usr/bin/env python3
import sys
from PyQt5 import QtWidgets
from create_module_ui import Ui_CreateModule_Window
import components
import files_create
import unidecode
import os
from lxml import etree

PPRZ_SRC = os.getenv("PAPARAZZI_SRC")
PPRZ_HOME = os.getenv("PAPARAZZI_HOME")
if PPRZ_SRC is None:
    PPRZ_SRC = os.path.realpath(os.path.dirname(os.path.realpath(__file__)) + "/../../..")
    print("PAPARAZZI_SRC not set. Use {} as paparazzi home.".format(PPRZ_SRC))
if PPRZ_HOME is None:
    PPRZ_HOME = PPRZ_SRC


class CreateModGUI(Ui_CreateModule_Window):
    def __init__(self, parent=None):
        Ui_CreateModule_Window.__init__(parent)
        self.components = []

    def build(self):
        self.scrollAreaWidgetContents.setStyleSheet("QFrame#Init, QFrame#Datalink, QFrame#Periodic,QFrame#Event { border: 2px solid grey}")
        self.create_button.clicked.connect(self.create_mod)
        self.component_add_button.clicked.connect(self.add_component)
        self.directory_combo.addItem("")
        for directory in sorted([f.name for f in os.scandir(PPRZ_SRC + "/sw/airborne/modules") if f.is_dir()]):
            self.directory_combo.addItem(directory)
        self.get_messages()

    @staticmethod
    def get_messages():
        messages_xml = PPRZ_HOME + "/var/messages.xml"
        root = etree.parse(messages_xml).getroot()
        classes = root.findall("msg_class")
        for c in classes:
            if c.attrib["name"] == "datalink":
                messages = [m.attrib["name"] for m in c.findall("message")]
                return messages
        return []

    def add_component(self):
        comp_name = self.components_combo.currentText()
        component = components.ComponentWidget(comp_name)
        if component.comp_type == "Datalink":
            messages = self.get_messages()
            component.ui.message_combo.addItem("")
            for m in messages:
                component.ui.message_combo.addItem(m)
        component.ui.remove_button.clicked.connect(lambda: self.remove_component(component))
        self.scroll_area_layout.insertWidget(self.scroll_area_layout.count()-1, component)
        self.components.append(component)

    def remove_component(self, component):
        component.deleteLater()
        self.components.remove(component)

    def create_mod(self):
        fc = files_create.FilesCreate()
        name = self.name_edit.text()
        directory = self.directory_combo.currentText()
        author = self.author_edit.text()
        email = self.email_edit.text()
        description = unidecode.unidecode(self.description_edit.toPlainText())
        self.description_edit.setPlainText(description)
        if name == "" or author == "" or description == "" or email == "":
            self.statusbar.showMessage("Please fill the name, author, mail and description!")
            self.statusbar.setStyleSheet("background-color:rgb(255,148,148);")
            return
        fc.name = name
        fc.directory = directory
        fc.description = description
        fc.author = author
        fc.email = email

        for comp in self.components:
            if comp.comp_type == "Init":
                init = comp.ui.init_edit.text()
                if init == "":
                    self.statusbar.showMessage("Please fill the Init field!")
                    self.statusbar.setStyleSheet("background-color:rgb(255,148,148);")
                    return
                fc.add_init(init)
            if comp.comp_type == "Periodic":
                periodic = comp.ui.periodic_edit.text()
                freq = comp.ui.freq_spinbox.value()
                stop = comp.ui.stop_edit.text()
                start = comp.ui.start_edit.text()
                autorun = comp.ui.autorun_comboBox.currentText()
                if periodic == "":
                    self.statusbar.showMessage("Please fill the Periodic field!")
                    self.statusbar.setStyleSheet("background-color:rgb(255,148,148);")
                    return
                if freq == 0:
                    self.statusbar.showMessage("Freq should be > 0!")
                    self.statusbar.setStyleSheet("background-color:rgb(255,148,148);")
                    return
                fc.add_periodic(periodic, freq, start, stop, autorun)

            if comp.comp_type == "Event":
                event = comp.ui.event_edit.text()
                if event == "":
                    self.statusbar.showMessage("Please fill the Event field!")
                    self.statusbar.setStyleSheet("background-color:rgb(255,148,148);")
                    return
                fc.add_event(event)
            if comp.comp_type == "Datalink":
                datalink = comp.ui.datalink_edit.text()
                msg = comp.ui.message_combo.currentText()
                if datalink == "" or msg == "":
                    self.statusbar.showMessage("Please fill the Datalink and Message fields!")
                    self.statusbar.setStyleSheet("background-color:rgb(255,148,148);")
                    return
                fc.add_datalink(datalink, msg)
        self.statusbar.setStyleSheet("background-color:rgb(155,232,155);")
        licence = self.licence_comboBox.currentText()
        files = fc.get_filenames()
        existing = list(filter(lambda x: os.path.exists(x) and not os.path.isdir(x), files))
        if len(existing) > 0:
            message = "Do you want to overwrite these files ?\n" + "\n".join(existing)
            buttonReply = QtWidgets.QMessageBox.question(None, 'Overwrite files ?', message,
                                        QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
            if buttonReply == QtWidgets.QMessageBox.No:
                self.statusbar.showMessage("Action canceled! (no files overwritten)")
                self.statusbar.setStyleSheet("background-color:rgb(255,204,0);")
                return
        fc.build_files(licence)
        fc.write_files()
        files_formated = "\n".join(files)
        message = "\n".join([x for x in files if os.path.isfile(x)])
        buttonReply = QtWidgets.QMessageBox.question(None, 'Created files', message,
                QtWidgets.QMessageBox.Ok, QtWidgets.QMessageBox.Ok)
        self.statusbar.showMessage("Ok! Files created")

    def closing(self):
        pass


def main():
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    create_mod = CreateModGUI()
    app.aboutToQuit.connect(create_mod.closing)
    create_mod.setupUi(MainWindow)
    create_mod.build()
    MainWindow.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
