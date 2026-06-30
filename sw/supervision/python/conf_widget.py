from PyQt5.QtWidgets import *
from PyQt5 import QtCore

import conf
import utils
from conf import AircraftConfig, Setting
from conf_file_widget import ConfFileWidget
from conf_settings_widget import ConfSettingsWidget


class ConfWidget(QWidget):

    conf_changed = QtCore.pyqtSignal()
    setting_changed = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        lay = QVBoxLayout(self)
        self.airframe = ConfFileWidget("Airframe", "airframes", self)
        self.airframe.file_changed.connect(self.conf_changed)
        self.flight_plan = ConfFileWidget("Flight Plan", "flight_plans", self)
        self.flight_plan.file_changed.connect(self.conf_changed)
        self.flight_plan.edit_alt_button.show()
        self.flight_plan.edit_alt_button.setText("Edit GCS")
        self.settings = ConfSettingsWidget(self)
        self.settings.settings_changed.connect(self.setting_changed)
        self.radio = ConfFileWidget("Radio", "radios", self)
        self.radio.file_changed.connect(self.conf_changed)
        self.telemetry = ConfFileWidget("Telemetry", "telemetry", self)
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
        self.set_config(ac.get_config())

    def set_config(self, config: AircraftConfig):
        self.airframe.set_path(config.airframe)
        self.telemetry.set_path(config.telemetry)
        self.radio.set_path(config.radio)
        self.flight_plan.set_path(config.flight_plan)
        self.settings.settings.clear()
        for setting in config.settings + config.settings_modules:
            item = QListWidgetItem(setting.name)
            item.setCheckState(QtCore.Qt.Checked if setting.enabled else QtCore.Qt.Unchecked)
            self.settings.settings.addItem(item)

    def get_config(self) -> AircraftConfig:
        settings, modules = self.get_settings()
        return AircraftConfig(
            self.airframe.path,
            self.radio.path,
            self.telemetry.path,
            self.flight_plan.path,
            settings,
            modules,
        )

    def get_settings(self):
        modules, settings = [], []
        for i in range(self.settings.settings.count()):
            item = self.settings.settings.item(i)
            setting = Setting(item.text(), item.checkState() == QtCore.Qt.Checked)
            if item.text().startswith("module"):
                modules.append(setting)
            else:
                settings.append(setting)
        return settings, modules

    def edit_setting(self, item: QListWidgetItem):
        utils.edit_file(item.text())

    def reset(self):
        self.airframe.set_path("")
        self.flight_plan.set_path("")
        self.telemetry.set_path("")
        self.radio.set_path("")
        self.settings.settings.clear()
