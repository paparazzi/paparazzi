# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/ui_supervision_window.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_SupervisionWindow(object):
    def setupUi(self, SupervisionWindow):
        SupervisionWindow.setObjectName("SupervisionWindow")
        SupervisionWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(SupervisionWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.header = HeaderWidget(self.centralwidget)
        self.header.setObjectName("header")
        self.verticalLayout.addWidget(self.header)
        self.tabwidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabwidget.setObjectName("tabwidget")
        self.configuration_panel = ConfigurationPanel()
        self.configuration_panel.setObjectName("configuration_panel")
        self.tabwidget.addTab(self.configuration_panel, "")
        self.operation_panel = OperationPanel()
        self.operation_panel.setObjectName("operation_panel")
        self.tabwidget.addTab(self.operation_panel, "")
        self.doc_panel = DocPanel()
        self.doc_panel.setObjectName("doc_panel")
        self.tabwidget.addTab(self.doc_panel, "")
        self.verticalLayout.addWidget(self.tabwidget)
        SupervisionWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(SupervisionWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        self.menuHelp = QtWidgets.QMenu(self.menubar)
        self.menuHelp.setObjectName("menuHelp")
        SupervisionWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(SupervisionWindow)
        self.statusbar.setObjectName("statusbar")
        SupervisionWindow.setStatusBar(self.statusbar)
        self.settings_action = QtWidgets.QAction(SupervisionWindow)
        self.settings_action.setObjectName("settings_action")
        self.about_action = QtWidgets.QAction(SupervisionWindow)
        self.about_action.setObjectName("about_action")
        self.menuFile.addAction(self.settings_action)
        self.menuHelp.addAction(self.about_action)
        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())

        self.retranslateUi(SupervisionWindow)
        self.tabwidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(SupervisionWindow)

    def retranslateUi(self, SupervisionWindow):
        _translate = QtCore.QCoreApplication.translate
        SupervisionWindow.setWindowTitle(_translate("SupervisionWindow", "Paparazzi Center"))
        self.tabwidget.setTabText(self.tabwidget.indexOf(self.configuration_panel), _translate("SupervisionWindow", "Configuration"))
        self.tabwidget.setTabText(self.tabwidget.indexOf(self.operation_panel), _translate("SupervisionWindow", "Operation"))
        self.tabwidget.setTabText(self.tabwidget.indexOf(self.doc_panel), _translate("SupervisionWindow", "Documentation"))
        self.menuFile.setTitle(_translate("SupervisionWindow", "File"))
        self.menuHelp.setTitle(_translate("SupervisionWindow", "Help"))
        self.settings_action.setText(_translate("SupervisionWindow", "Edit Settings"))
        self.about_action.setText(_translate("SupervisionWindow", "About"))
from configuration_panel import ConfigurationPanel
from doc_panel import DocPanel
from header_widget import HeaderWidget
from operation_panel import OperationPanel
