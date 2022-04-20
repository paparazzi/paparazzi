# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
from generated.ui_conf_header import Ui_ConfHeader
import conf


class HeaderWidget(QWidget, Ui_ConfHeader):

    set_changed = QtCore.pyqtSignal(str)
    ac_changed = QtCore.pyqtSignal(str)
    id_changed = QtCore.pyqtSignal(int)

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.setupUi(self)
        self.set_combo.currentTextChanged.connect(self.set_changed)
        self.ac_combo.currentTextChanged.connect(self.ac_changed)
        self.id_spinBox.valueChanged.connect(self.id_changed)
        self.menu_button.addAction(self.rename_action)
        self.menu_button.addAction(self.new_ac_action)
        self.menu_button.addAction(self.duplicate_action)
        self.menu_button.addAction(self.remove_ac_action)

    def set_sets(self, sets, conf_init: str = None):
        self.set_combo.addItems(sets)
        if conf_init in sets:
            self.set_combo.setCurrentText(conf_init)

    def set_acs(self, acs):
        self.ac_combo.clear()
        self.ac_combo.addItems(acs)

    def set_ac(self, ac: conf.Aircraft):
        self.id_spinBox.setValue(ac.ac_id)
        self.set_color(ac.get_color())

    def remove_current(self):
        i = self.ac_combo.currentIndex()
        self.ac_combo.removeItem(i)

    def set_current(self, ac_name):
        self.ac_combo.setCurrentText(ac_name)

    def add_ac(self, ac_name):
        self.ac_combo.addItem(ac_name)
        self.set_current(ac_name)

    def rename_ac(self, new_name):
        i = self.ac_combo.currentIndex()
        self.ac_combo.setItemText(i, new_name)

    def set_color(self, color: str):
        self.color_button.setStyleSheet("background-color: {};".format(color))
