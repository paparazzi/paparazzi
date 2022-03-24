import os.path

from generated.ui_session import Ui_Session
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
import utils


class SessionWidget(QWidget):

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.ui = Ui_Session()
        self.ui.setupUi(self)
        self.ui.menu_button.addAction(self.ui.save_session_action)
        self.ui.menu_button.addAction(self.ui.duplicate_session_action)
        self.ui.menu_button.addAction(self.ui.new_session_action)
