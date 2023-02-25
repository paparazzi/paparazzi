# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/ui_program.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Program(object):
    def setupUi(self, Program):
        Program.setObjectName("Program")
        Program.resize(374, 25)
        self.horizontalLayout = QtWidgets.QHBoxLayout(Program)
        self.horizontalLayout.setContentsMargins(-1, 0, -1, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.icon_label = QtWidgets.QLabel(Program)
        self.icon_label.setObjectName("icon_label")
        self.horizontalLayout.addWidget(self.icon_label)
        self.program_lineedit = QtWidgets.QLineEdit(Program)
        self.program_lineedit.setObjectName("program_lineedit")
        self.horizontalLayout.addWidget(self.program_lineedit)
        self.line = QtWidgets.QFrame(Program)
        self.line.setFrameShape(QtWidgets.QFrame.VLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.horizontalLayout.addWidget(self.line)
        self.run_button = QtWidgets.QToolButton(Program)
        icon = QtGui.QIcon.fromTheme("media-playback-stop")
        self.run_button.setIcon(icon)
        self.run_button.setObjectName("run_button")
        self.horizontalLayout.addWidget(self.run_button)
        self.remove_button = QtWidgets.QToolButton(Program)
        icon = QtGui.QIcon.fromTheme("list-remove")
        self.remove_button.setIcon(icon)
        self.remove_button.setObjectName("remove_button")
        self.horizontalLayout.addWidget(self.remove_button)

        self.retranslateUi(Program)
        QtCore.QMetaObject.connectSlotsByName(Program)

    def retranslateUi(self, Program):
        _translate = QtCore.QCoreApplication.translate
        Program.setWindowTitle(_translate("Program", "Form"))
        self.icon_label.setText(_translate("Program", "..."))
        self.run_button.setText(_translate("Program", "..."))
        self.remove_button.setText(_translate("Program", "..."))
