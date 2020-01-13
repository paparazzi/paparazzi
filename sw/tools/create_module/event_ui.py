# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'event_ui.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Event(object):
    def setupUi(self, Event):
        Event.setObjectName("Event")
        Event.resize(619, 77)
        self.verticalLayout = QtWidgets.QVBoxLayout(Event)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_11 = QtWidgets.QLabel(Event)
        self.label_11.setStyleSheet("font: bold")
        self.label_11.setAlignment(QtCore.Qt.AlignCenter)
        self.label_11.setObjectName("label_11")
        self.horizontalLayout.addWidget(self.label_11)
        self.remove_button = QtWidgets.QToolButton(Event)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("remove.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.remove_button.setIcon(icon)
        self.remove_button.setObjectName("remove_button")
        self.horizontalLayout.addWidget(self.remove_button)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_10 = QtWidgets.QLabel(Event)
        self.label_10.setObjectName("label_10")
        self.horizontalLayout_2.addWidget(self.label_10)
        self.event_edit = QtWidgets.QLineEdit(Event)
        self.event_edit.setObjectName("event_edit")
        self.horizontalLayout_2.addWidget(self.event_edit)
        self.verticalLayout.addLayout(self.horizontalLayout_2)

        self.retranslateUi(Event)
        QtCore.QMetaObject.connectSlotsByName(Event)

    def retranslateUi(self, Event):
        _translate = QtCore.QCoreApplication.translate
        Event.setWindowTitle(_translate("Event", "Event"))
        self.label_11.setText(_translate("Event", "Event"))
        self.remove_button.setText(_translate("Event", "remove"))
        self.label_10.setText(_translate("Event", "fun"))

