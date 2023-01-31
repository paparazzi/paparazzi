from PyQt5.QtWidgets import *
from PyQt5 import QtCore

import conf
import utils
from conf_file_widget import ConfFileWidget
from conf_settings_widget import ConfSettingsWidget


class ConfWidget(QWidget):

    conf_changed = QtCore.pyqtSignal()
    setting_changed = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        lay = QVBoxLayout(self)
        self.airframe = ConfFileWidget("Airframe", self)
        self.airframe.file_changed.connect(self.conf_changed)
        self.flight_plan = ConfFileWidget("Flight Plan", self)
        self.flight_plan.file_changed.connect(self.conf_changed)
        self.flight_plan.edit_alt_button.show()
        self.flight_plan.edit_alt_button.setText("Edit GCS")
        self.settings = ConfSettingsWidget(self)
        self.settings.settings_changed.connect(self.setting_changed)
        self.radio = ConfFileWidget("Radio", self)
        self.radio.file_changed.connect(self.conf_changed)
        self.telemetry = ConfFileWidget("Telemetry", self)
        self.telemetry.file_changed.connect(self.conf_changed)

        self.settings.settings.itemDoubleClicked.connect(self.edit_setting)
        self.settings.settings.itemChanged.connect(self.setting_changed)

        vb = QVBoxLayout()
        vb.addWidget(self.settings)
        lay.addWidget(self.airframe)
        lay.addWidget(self.flight_plan)
        lay.addItem(vb)
        lay.addWidget(self.radio)
        lay.addWidget(self.telemetry)

    def set_ac(self, ac: conf.Aircraft):
        self.airframe.set_path(ac.airframe)
        self.telemetry.set_path(ac.telemetry)
        self.radio.set_path(ac.radio)
        self.flight_plan.set_path(ac.flight_plan)
        self.settings.settings.clear()
        for setting in ac.settings + ac.settings_modules:
            item = QListWidgetItem(setting.name)
            item.setCheckState(QtCore.Qt.Checked if setting.enabled else QtCore.Qt.Unchecked)
            self.settings.settings.addItem(item)

    def edit_setting(self, item: QListWidgetItem):
        utils.edit_file(item.text())

    def reset(self):
        self.airframe.set_path("")
        self.flight_plan.set_path("")
        self.telemetry.set_path("")
        self.radio.set_path("")
        self.settings.settings.clear()

