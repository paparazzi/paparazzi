import os.path

from generated.ui_console import Ui_Console
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
import utils


class ConsoleWidget(QWidget):

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.ui = Ui_Console()
        self.ui.setupUi(self)
