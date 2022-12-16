import os.path

from generated.ui_conf_settings import Ui_SettingsConf
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
import utils


class ConfSettingsWidget(QWidget, Ui_SettingsConf):

    settings_changed = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.setupUi(self)
        self.add_setting_button.clicked.connect(self.open_setting)
        self.remove_setting_button.clicked.connect(self.remove_setting)
        self.settings.currentItemChanged.connect(self.set_remove_button_state)

    def open_setting(self):
        base_settings_path = os.path.join(utils.CONF_DIR, "settings")
        filenames, _ = QFileDialog.getOpenFileNames(self, "Add settings", base_settings_path, "Settings (*.xml)")
        for filename in filenames:
            name = utils.removeprefix(filename, utils.CONF_DIR)
            print(name)
            item = QListWidgetItem(name)
            item.setCheckState(QtCore.Qt.Checked)
            self.settings.addItem(item)
        self.settings_changed.emit()

    def remove_setting(self):
        row = self.settings.currentRow()
        self.settings.takeItem(row)
        self.settings_changed.emit()

    def set_remove_button_state(self, item: QListWidgetItem, _):
        if item.text().startswith("modules/"):
            self.remove_setting_button.setDisabled(True)
        else:
            self.remove_setting_button.setDisabled(False)

