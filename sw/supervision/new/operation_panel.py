import os.path

from generated.ui_operation_panel import Ui_OperationPanel
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
import utils
from program_widget import ProgramWidget


class OperationPanel(QWidget):

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.ui = Ui_OperationPanel()
        self.ui.setupUi(self)
        self.ui.session.set_console(self.ui.console)
