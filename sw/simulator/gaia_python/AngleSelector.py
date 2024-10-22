#!/usr/bin/env python3

import typing

from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import QWidget,QMainWindow,QApplication

from gaia_python.generated.AngleSelector import Ui_Form

class AngleSelector(QWidget):
    def __init__(self, parent: QWidget | None = ..., flags: QtCore.Qt.WindowFlags | QtCore.Qt.WindowType = QtCore.Qt.WindowType.Widget) -> None:
        super().__init__(parent, flags)
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        
        self.ui.dial.valueChanged.connect(lambda v : self.ui.spinBox.setValue((v-180)%360))
        self.ui.spinBox.valueChanged.connect(lambda v : self.ui.dial.setValue((v-180)%360))
        
        self.valueChanged = self.ui.dial.valueChanged
        self.ui.dial.setStyleSheet("color: beige; background: beige; selection-color: beige; selection-background-color: beige;")
#        self.ui.dial.setStyleSheet(
#        """
#QSlider::add-page{
#    border: 1px solid $border;
#    background:$background;
#}
#QSlider::sub-page{
#    border: 1px solid $border;
#    background:$background;
#}
#        """
#        )
        
        
        self.setValue(0)
        
    def setText(self,s:str):
        self.ui.label.setText(s)
        
    def setValue(self,v:int):
        self.ui.dial.setValue((180+v)%360)
        self.ui.spinBox.setValue(v)
        
    def value(self) -> int:
        return self.ui.spinBox.value()
    
        
        
if __name__ == "__main__":
    app = QApplication([])
    window = QMainWindow()
    window.setCentralWidget(AngleSelector(window))
    window.show()
    app.exec()
