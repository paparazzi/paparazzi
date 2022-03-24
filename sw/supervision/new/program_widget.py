import os.path

from generated.ui_program import Ui_Program
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
import utils
from typing import List


class ProgramWidget(QWidget):

    def __init__(self, shortname: str, cmd: List[str], parent=None):
        QWidget.__init__(self, parent=parent)
        self.ui = Ui_Program()
        self.ui.setupUi(self)
        self.ui.program_label.setText(shortname)
        self.ui.program_lineedit.setText(" ".join(cmd))
        # self.ui.program_lineedit.hide()
        self.ui.program_label.hide()
        self.ui.shortview_checkbox.toggled.connect(self.toggle_view)
        self.ui.program_lineedit.setReadOnly(True)
        self.ui.program_lineedit.returnPressed.connect(self.startstop)

    def toggle_view(self, long_view):
        self.ui.program_label.setVisible(long_view)
        self.ui.program_lineedit.setVisible(not long_view)

    def startstop(self):
        print("start or stop program!")
