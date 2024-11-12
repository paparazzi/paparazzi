#!/usr/bin/env python3

import typing

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QWidget,QMainWindow,QApplication
from gaia_python.ArrowDial import ArrowDial

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(175, 200)
        self.verticalLayout = QtWidgets.QVBoxLayout(Form)
        self.verticalLayout.setObjectName("verticalLayout")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.label_3 = QtWidgets.QLabel(Form)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 0, 1, 1, 1, QtCore.Qt.AlignHCenter|QtCore.Qt.AlignBottom)
        self.label_4 = QtWidgets.QLabel(Form)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 1, 2, 1, 1, QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_2 = QtWidgets.QLabel(Form)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 1, 0, 1, 1, QtCore.Qt.AlignRight|QtCore.Qt.AlignVCenter)
        self.dial = ArrowDial(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.dial.sizePolicy().hasHeightForWidth())
        self.dial.setSizePolicy(sizePolicy)
        self.dial.setMinimumSize(QtCore.QSize(100, 100))
        self.dial.setMaximum(360)
        self.dial.setWrapping(True)
        self.dial.setNotchesVisible(True)
        self.dial.setObjectName("dial")
        self.gridLayout.addWidget(self.dial, 1, 1, 1, 1)
        self.label_5 = QtWidgets.QLabel(Form)
        self.label_5.setObjectName("label_5")
        self.gridLayout.addWidget(self.label_5, 2, 1, 1, 1, QtCore.Qt.AlignHCenter|QtCore.Qt.AlignTop)
        self.verticalLayout.addLayout(self.gridLayout)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(Form)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.spinBox = QtWidgets.QSpinBox(Form)
        self.spinBox.setWrapping(True)
        self.spinBox.setMaximum(360)
        self.spinBox.setObjectName("spinBox")
        self.horizontalLayout.addWidget(self.spinBox)
        self.verticalLayout.addLayout(self.horizontalLayout)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.label_3.setText(_translate("Form", "0"))
        self.label_4.setText(_translate("Form", "90"))
        self.label_2.setText(_translate("Form", "270"))
        self.label_5.setText(_translate("Form", "180"))
        self.label.setText(_translate("Form", "Angle (°)"))


class AngleSelector(QWidget):
    def __init__(self, parent: QWidget | None = ..., flags: QtCore.Qt.WindowFlags | QtCore.Qt.WindowType = QtCore.Qt.WindowType.Widget) -> None:
        super().__init__(parent, flags)
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        
        self.ui.dial.valueChanged.connect(lambda v : self.ui.spinBox.setValue(v))
        self.ui.spinBox.valueChanged.connect(lambda v : self.ui.dial.setValue(v))
        
        self.valueChanged = self.ui.dial.valueChanged
        
        self.setValue(0)
        
    def setText(self,s:str):
        self.ui.label.setText(s)
        
    def setValue(self,v:int):
        self.ui.dial.setValue(v)
        self.ui.spinBox.setValue(v)
        
    def value(self) -> int:
        return self.ui.spinBox.value()
    
        
        
if __name__ == "__main__":
    app = QApplication([])
    window = QMainWindow()
    window.setCentralWidget(AngleSelector(window))
    window.show()
    app.exec()
