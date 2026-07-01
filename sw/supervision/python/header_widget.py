# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
from generated.ui_conf_header import Ui_ConfHeader
from conf import Aircraft, Conf
import paparazzi


class HeaderWidget(QWidget, Ui_ConfHeader):

    set_changed = QtCore.pyqtSignal(str)
    ac_edited = QtCore.pyqtSignal(Aircraft)
    ac_save = QtCore.pyqtSignal(Aircraft)

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.setupUi(self)
        self.currentAc: Aircraft = None
        self.set_combo.currentTextChanged.connect(self.set_changed)

        self.refresh_button.clicked.connect(lambda: self.ac_edited.emit(self.currentAc))
        self.save_button.clicked.connect(lambda: self.ac_save.emit(self.currentAc))

    def update_sets(self):
        sets = paparazzi.get_list_of_conf_files()
        conf_init = Conf.get_current_conf()
        self.set_combo.addItems(sets)
        if conf_init in sets:
            self.set_combo.setCurrentText(conf_init)

    def set_ac(self, ac: Aircraft):
        self.currentAc = ac

    def disable_sets(self):
        self.set_combo.setDisabled(True)

    def enable_sets(self):
        self.set_combo.setDisabled(False)
