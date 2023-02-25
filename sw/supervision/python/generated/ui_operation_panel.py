# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/ui_operation_panel.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_OperationPanel(object):
    def setupUi(self, OperationPanel):
        OperationPanel.setObjectName("OperationPanel")
        OperationPanel.resize(400, 300)
        self.verticalLayout = QtWidgets.QVBoxLayout(OperationPanel)
        self.verticalLayout.setObjectName("verticalLayout")
        self.splitter = QtWidgets.QSplitter(OperationPanel)
        self.splitter.setOrientation(QtCore.Qt.Vertical)
        self.splitter.setObjectName("splitter")
        self.session = SessionWidget(self.splitter)
        self.session.setObjectName("session")
        self.console = ConsoleWidget(self.splitter)
        self.console.setObjectName("console")
        self.verticalLayout.addWidget(self.splitter)

        self.retranslateUi(OperationPanel)
        QtCore.QMetaObject.connectSlotsByName(OperationPanel)

    def retranslateUi(self, OperationPanel):
        _translate = QtCore.QCoreApplication.translate
        OperationPanel.setWindowTitle(_translate("OperationPanel", "Form"))
from console_widget import ConsoleWidget
from session_widget import SessionWidget
