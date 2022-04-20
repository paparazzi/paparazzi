# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
from generated.ui_app_settings import Ui_AppSettingsDialog
import utils


class AppSettings(QDialog, Ui_AppSettingsDialog):

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.setupUi(self)
        settings = utils.get_settings()
        self.text_editor_edit.setText(settings.value("text_editor", "", str))
        self.terminal_emulator_edit.setText(settings.value("terminal_emulator", "", str))
        self.keep_changes_checkbox.setChecked(settings.value("always_keep_changes", False, bool))
        self.finished.connect(self.handle_finished)

    def handle_finished(self, result):
        if result:
            settings = utils.get_settings()
            text_editor = self.text_editor_edit.text()
            settings.setValue("text_editor", text_editor)
            terminal_emulator = self.terminal_emulator_edit.text()
            settings.setValue("terminal_emulator", terminal_emulator)
            keep_changes = self.keep_changes_checkbox.isChecked()
            settings.setValue("always_keep_changes", keep_changes)
