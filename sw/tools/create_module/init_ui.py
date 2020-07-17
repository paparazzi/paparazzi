# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'init_ui.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Init(object):
    def setupUi(self, Init):
        Init.setObjectName("Init")
        Init.resize(650, 76)
        self.verticalLayout = QtWidgets.QVBoxLayout(Init)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_11 = QtWidgets.QLabel(Init)
        self.label_11.setStyleSheet("font: bold")
        self.label_11.setAlignment(QtCore.Qt.AlignCenter)
        self.label_11.setObjectName("label_11")
        self.horizontalLayout.addWidget(self.label_11)
        self.remove_button = QtWidgets.QToolButton(Init)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("remove.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.remove_button.setIcon(icon)
        self.remove_button.setObjectName("remove_button")
        self.horizontalLayout.addWidget(self.remove_button)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_10 = QtWidgets.QLabel(Init)
        self.label_10.setObjectName("label_10")
        self.horizontalLayout_2.addWidget(self.label_10)
        self.init_edit = QtWidgets.QLineEdit(Init)
        self.init_edit.setObjectName("init_edit")
        self.horizontalLayout_2.addWidget(self.init_edit)
        self.verticalLayout.addLayout(self.horizontalLayout_2)

        self.retranslateUi(Init)
        QtCore.QMetaObject.connectSlotsByName(Init)

    def retranslateUi(self, Init):
        _translate = QtCore.QCoreApplication.translate
        Init.setWindowTitle(_translate("Init", "Init"))
        self.label_11.setText(_translate("Init", "Init"))
        self.remove_button.setText(_translate("Init", "remove"))
        self.label_10.setText(_translate("Init", "fun"))

