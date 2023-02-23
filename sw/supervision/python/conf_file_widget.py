import os.path

from generated.ui_conf_item import Ui_FileConf
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
import utils


class ConfFileWidget(QWidget, Ui_FileConf):

    file_changed = QtCore.pyqtSignal()
    edit_alt = QtCore.pyqtSignal(str)

    def __init__(self, title: str, conf_subdir: str, parent=None):
        QWidget.__init__(self, parent=parent)
        self.conf_subdir = conf_subdir
        self.path = None
        self.setupUi(self)
        self.edit_alt_button.hide()
        self.title_label.setText(title)
        self.edit_button.clicked.connect(self.edit)
        self.select_button.clicked.connect(self.select_file)
        self.edit_alt_button.clicked.connect(lambda: self.edit_alt.emit(self.path_label.text()))

    def set_path(self, path):
        self.path = path
        self.path_label.setText(path)

    def edit(self):
        if self.path != "":
            utils.edit_file(self.path)

    def select_file(self):
        base_path = os.path.join(utils.CONF_DIR, self.conf_subdir)
        (path, _) = QFileDialog().getOpenFileName(self, "Select {} file".format(self.title_label.text()),
                                                  base_path, "Xml (*.xml)")
        if path != "":
            self.path = os.path.relpath(path, utils.CONF_DIR)
            self.path_label.setText(self.path)
            self.file_changed.emit()
