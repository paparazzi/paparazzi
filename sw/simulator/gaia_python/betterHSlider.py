#!/usr/bin/env python3

import typing

from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import QWidget,QMainWindow,QApplication,\
                            QLabel

from gaia_python.generated.betterHSlider_ui import Ui_Form
from gaia_python.betterAbstractSlider import BetterAbstractSlider


class BetterHSlider(QWidget,Ui_Form,BetterAbstractSlider):
    def __init__(self, parent: QWidget | None = ..., flags: QtCore.Qt.WindowFlags | QtCore.Qt.WindowType = QtCore.Qt.WindowType.Widget) -> None:
        super().__init__(parent, flags)
        self.setupUi(self)
        self.setupSignals()
        
        
if __name__ == "__main__":
    app = QApplication([])
    window = QMainWindow()
    window.setCentralWidget(BetterHSlider(window))
    window.show()
    app.exec()