import os.path

from generated.ui_program import Ui_Program
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
import utils


class ProgramWidget(QWidget):

    file_changed = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.ui = Ui_Program()
