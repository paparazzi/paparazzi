#!/usr/bin/env python3

import typing

from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import QWidget,QMainWindow,QApplication,\
                            QLabel

from gaia_python.generated.AngleSelector import Ui_Form
from gaia_python.betterAbstractSlider import BetterAbstractSlider


class AngleSelector(QWidget,Ui_Form):
    def __init__(self, parent: QWidget | None = ..., flags: QtCore.Qt.WindowFlags | QtCore.Qt.WindowType = QtCore.Qt.WindowType.Widget) -> None:
        super().__init__(parent, flags)
        self.setupUi(self)
        
        self.valueChanged = self.dial.valueChanged
        
    def setText(self,s:str):
        self.label.setText(s)
        
    def setValue(self,v:int):
        self.dial.setValue(v)
        self.spinBox.setValue(v)
        
    def value(self) -> int:
        return self.dial.value()
    
        
        
if __name__ == "__main__":
    app = QApplication([])
    window = QMainWindow()
    window.setCentralWidget(AngleSelector(window))
    window.show()
    app.exec()