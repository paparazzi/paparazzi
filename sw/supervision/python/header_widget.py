# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
from generated.ui_conf_header import Ui_ConfHeader
from conf import Aircraft, Conf
import utils
import os
import sys
lib_path = os.path.normpath(os.path.join(utils.PAPARAZZI_SRC, 'sw', 'lib', 'python'))
sys.path.append(lib_path)
import paparazzi
from typing import List


class HeaderWidget(QWidget, Ui_ConfHeader):

    set_changed = QtCore.pyqtSignal(str)
    ac_changed = QtCore.pyqtSignal(str)
    ac_edited = QtCore.pyqtSignal(Aircraft)
    ac_rename = QtCore.pyqtSignal(Aircraft)
    ac_delete = QtCore.pyqtSignal(Aircraft)
    ac_duplicate = QtCore.pyqtSignal(Aircraft)
    ac_save = QtCore.pyqtSignal(Aircraft)
    ac_new = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.setupUi(self)
        self.currentAc: Aircraft = None
        self.set_combo.currentTextChanged.connect(self.set_changed)
        self.ac_combo.currentTextChanged.connect(self.ac_changed)
        self.id_spinBox.valueChanged.connect(self.handle_id_changed)

        self.refresh_button.clicked.connect(lambda: self.ac_edited.emit(self.currentAc))
        self.color_button.clicked.connect(self.change_color)
        self.save_button.clicked.connect(lambda: self.ac_save.emit(self.currentAc))

        self.menu_button.addAction(self.rename_action)
        self.rename_action.triggered.connect(lambda: self.ac_rename.emit(self.currentAc))
        self.menu_button.addAction(self.remove_ac_action)
        self.remove_ac_action.triggered.connect(lambda: self.ac_delete.emit(self.currentAc))
        self.menu_button.addAction(self.duplicate_action)
        self.duplicate_action.triggered.connect(lambda: self.ac_duplicate.emit(self.currentAc))
        self.menu_button.addAction(self.new_ac_action)
        self.new_ac_action.triggered.connect(self.ac_new)

    def handle_id_changed(self, new_id):
        self.currentAc.ac_id = new_id
        self.ac_edited.emit(self.currentAc)

    def update_sets(self):
        sets = paparazzi.get_list_of_conf_files()
        conf_init = Conf.get_current_conf()
        self.set_combo.addItems(sets)
        if conf_init in sets:
            self.set_combo.setCurrentText(conf_init)

    def set_acs(self, acs: List[str]):
        self.ac_combo.clear()
        self.ac_combo.addItems(acs)

    def set_ac(self, ac: Aircraft):
        self.currentAc = ac
        self.id_spinBox.blockSignals(True)
        self.id_spinBox.setValue(ac.ac_id)
        self.id_spinBox.blockSignals(False)
        self.color_button.setStyleSheet("background-color: {};".format(ac.get_color()))
        self.ac_combo.setCurrentText(ac.name)

    def add_ac(self, ac: Aircraft):
        self.ac_combo.addItem(ac.name)

    def remove_ac(self, ac):
        for i in range(self.ac_combo.count()):
            if self.ac_combo.itemText(i) == ac.name:
                self.ac_combo.removeItem(i)
                break

    def rename_ac(self, new_name):
        i = self.ac_combo.currentIndex()
        self.ac_combo.setItemText(i, new_name)

    def change_color(self):
        initial = QtGui.QColor(self.currentAc.get_color())
        color = QColorDialog.getColor(initial, self, "AC color")
        if color.isValid():
            color_name = color.name()
            self.currentAc.set_color(color_name)
            self.color_button.setStyleSheet("background-color: {};".format(color_name))
            self.ac_edited.emit(self.currentAc)

    def disable_sets(self):
        self.set_combo.setDisabled(True)

    def enable_sets(self):
        self.set_combo.setDisabled(False)
