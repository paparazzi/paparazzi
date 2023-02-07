# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/ui_new_ac_dialog.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_NewACDialog(object):
    def setupUi(self, NewACDialog):
        NewACDialog.setObjectName("NewACDialog")
        NewACDialog.resize(187, 106)
        self.gridLayout = QtWidgets.QGridLayout(NewACDialog)
        self.gridLayout.setObjectName("gridLayout")
        self.label = QtWidgets.QLabel(NewACDialog)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)
        self.name_edit = QtWidgets.QLineEdit(NewACDialog)
        self.name_edit.setPlaceholderText("")
        self.name_edit.setObjectName("name_edit")
        self.gridLayout.addWidget(self.name_edit, 0, 1, 1, 1)
        self.label_2 = QtWidgets.QLabel(NewACDialog)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 1, 0, 1, 1)
        self.id_spinbox = QtWidgets.QSpinBox(NewACDialog)
        self.id_spinbox.setMinimum(1)
        self.id_spinbox.setMaximum(255)
        self.id_spinbox.setObjectName("id_spinbox")
        self.gridLayout.addWidget(self.id_spinbox, 1, 1, 1, 1)
        self.buttonBox = QtWidgets.QDialogButtonBox(NewACDialog)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.gridLayout.addWidget(self.buttonBox, 2, 0, 1, 2)

        self.retranslateUi(NewACDialog)
        QtCore.QMetaObject.connectSlotsByName(NewACDialog)

    def retranslateUi(self, NewACDialog):
        _translate = QtCore.QCoreApplication.translate
        NewACDialog.setWindowTitle(_translate("NewACDialog", "Aircraft"))
        self.label.setText(_translate("NewACDialog", "Name"))
        self.label_2.setText(_translate("NewACDialog", "ID"))
