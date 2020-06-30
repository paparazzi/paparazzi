# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'datalink_ui.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Datalink(object):
    def setupUi(self, Datalink):
        Datalink.setObjectName("Datalink")
        Datalink.resize(753, 79)
        self.verticalLayout = QtWidgets.QVBoxLayout(Datalink)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_11 = QtWidgets.QLabel(Datalink)
        self.label_11.setStyleSheet("font: bold")
        self.label_11.setAlignment(QtCore.Qt.AlignCenter)
        self.label_11.setObjectName("label_11")
        self.horizontalLayout.addWidget(self.label_11)
        self.remove_button = QtWidgets.QToolButton(Datalink)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("remove.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.remove_button.setIcon(icon)
        self.remove_button.setObjectName("remove_button")
        self.horizontalLayout.addWidget(self.remove_button)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.horizontalLayout_20 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_20.setObjectName("horizontalLayout_20")
        self.label_16 = QtWidgets.QLabel(Datalink)
        self.label_16.setObjectName("label_16")
        self.horizontalLayout_20.addWidget(self.label_16)
        self.datalink_edit = QtWidgets.QLineEdit(Datalink)
        self.datalink_edit.setObjectName("datalink_edit")
        self.horizontalLayout_20.addWidget(self.datalink_edit)
        self.horizontalLayout_2.addLayout(self.horizontalLayout_20)
        self.horizontalLayout_19 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_19.setObjectName("horizontalLayout_19")
        self.label_15 = QtWidgets.QLabel(Datalink)
        self.label_15.setObjectName("label_15")
        self.horizontalLayout_19.addWidget(self.label_15)
        self.message_combo = QtWidgets.QComboBox(Datalink)
        self.message_combo.setEditable(True)
        self.message_combo.setObjectName("message_combo")
        self.horizontalLayout_19.addWidget(self.message_combo)
        self.horizontalLayout_2.addLayout(self.horizontalLayout_19)
        self.verticalLayout.addLayout(self.horizontalLayout_2)

        self.retranslateUi(Datalink)
        QtCore.QMetaObject.connectSlotsByName(Datalink)

    def retranslateUi(self, Datalink):
        _translate = QtCore.QCoreApplication.translate
        Datalink.setWindowTitle(_translate("Datalink", "Datalink"))
        self.label_11.setText(_translate("Datalink", "Datalink"))
        self.remove_button.setText(_translate("Datalink", "remove"))
        self.label_16.setText(_translate("Datalink", "fun"))
        self.label_15.setText(_translate("Datalink", "Message"))

