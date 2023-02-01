# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/ui_app_settings.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_AppSettingsDialog(object):
    def setupUi(self, AppSettingsDialog):
        AppSettingsDialog.setObjectName("AppSettingsDialog")
        AppSettingsDialog.resize(381, 131)
        self.gridLayout = QtWidgets.QGridLayout(AppSettingsDialog)
        self.gridLayout.setObjectName("gridLayout")
        self.label = QtWidgets.QLabel(AppSettingsDialog)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)
        self.text_editor_edit = QtWidgets.QLineEdit(AppSettingsDialog)
        self.text_editor_edit.setObjectName("text_editor_edit")
        self.gridLayout.addWidget(self.text_editor_edit, 0, 1, 1, 1)
        self.label_2 = QtWidgets.QLabel(AppSettingsDialog)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 1, 0, 1, 1)
        self.terminal_emulator_edit = QtWidgets.QLineEdit(AppSettingsDialog)
        self.terminal_emulator_edit.setObjectName("terminal_emulator_edit")
        self.gridLayout.addWidget(self.terminal_emulator_edit, 1, 1, 1, 1)
        self.label_3 = QtWidgets.QLabel(AppSettingsDialog)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 2, 0, 1, 1)
        self.keep_changes_checkbox = QtWidgets.QCheckBox(AppSettingsDialog)
        self.keep_changes_checkbox.setText("")
        self.keep_changes_checkbox.setObjectName("keep_changes_checkbox")
        self.gridLayout.addWidget(self.keep_changes_checkbox, 2, 1, 1, 1)
        self.buttonBox = QtWidgets.QDialogButtonBox(AppSettingsDialog)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.gridLayout.addWidget(self.buttonBox, 3, 0, 1, 2)

        self.retranslateUi(AppSettingsDialog)
        self.buttonBox.accepted.connect(AppSettingsDialog.accept)
        self.buttonBox.rejected.connect(AppSettingsDialog.reject)
        QtCore.QMetaObject.connectSlotsByName(AppSettingsDialog)

    def retranslateUi(self, AppSettingsDialog):
        _translate = QtCore.QCoreApplication.translate
        AppSettingsDialog.setWindowTitle(_translate("AppSettingsDialog", "Settings"))
        self.label.setText(_translate("AppSettingsDialog", "Text editor"))
        self.label_2.setText(_translate("AppSettingsDialog", "Terminal Emulator"))
        self.label_3.setText(_translate("AppSettingsDialog", "Keep changes on exit"))
