import os.path

from generated.ui_operation_panel import Ui_OperationPanel
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
import utils


class OperationPanel(QWidget):

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.ui = Ui_OperationPanel()
        self.ui.setupUi(self)
