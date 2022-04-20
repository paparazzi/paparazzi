# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/configuration_panel.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_ConfigurationPanel(object):
    def setupUi(self, ConfigurationPanel):
        ConfigurationPanel.setObjectName("ConfigurationPanel")
        ConfigurationPanel.resize(562, 480)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(ConfigurationPanel)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.header = HeaderWidget(ConfigurationPanel)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.header.sizePolicy().hasHeightForWidth())
        self.header.setSizePolicy(sizePolicy)
        self.header.setObjectName("header")
        self.verticalLayout_2.addWidget(self.header)
        self.line = QtWidgets.QFrame(ConfigurationPanel)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.verticalLayout_2.addWidget(self.line)
        self.splitter = QtWidgets.QSplitter(ConfigurationPanel)
        self.splitter.setOrientation(QtCore.Qt.Horizontal)
        self.splitter.setObjectName("splitter")
        self.conf_widget = ConfWidget(self.splitter)
        self.conf_widget.setObjectName("conf_widget")
        self.widget_2 = QtWidgets.QWidget(self.splitter)
        self.widget_2.setObjectName("widget_2")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.widget_2)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.build_widget = BuildWidget(self.widget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.build_widget.sizePolicy().hasHeightForWidth())
        self.build_widget.setSizePolicy(sizePolicy)
        self.build_widget.setMinimumSize(QtCore.QSize(0, 0))
        self.build_widget.setObjectName("build_widget")
        self.verticalLayout.addWidget(self.build_widget)
        self.programs_widget = QtWidgets.QFrame(self.widget_2)
        self.programs_widget.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.programs_widget.setFrameShadow(QtWidgets.QFrame.Raised)
        self.programs_widget.setObjectName("programs_widget")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.programs_widget)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.verticalLayout.addWidget(self.programs_widget)
        self.console_widget = ConsoleWidget(self.widget_2)
        self.console_widget.setObjectName("console_widget")
        self.verticalLayout.addWidget(self.console_widget)
        self.verticalLayout.setStretch(2, 1)
        self.verticalLayout_2.addWidget(self.splitter)
        self.save_conf_action = QtWidgets.QAction(ConfigurationPanel)
        self.save_conf_action.setObjectName("save_conf_action")

        self.retranslateUi(ConfigurationPanel)
        QtCore.QMetaObject.connectSlotsByName(ConfigurationPanel)

    def retranslateUi(self, ConfigurationPanel):
        _translate = QtCore.QCoreApplication.translate
        ConfigurationPanel.setWindowTitle(_translate("ConfigurationPanel", "Form"))
        self.save_conf_action.setText(_translate("ConfigurationPanel", "Save configuration"))
        self.save_conf_action.setShortcut(_translate("ConfigurationPanel", "Ctrl+S"))
from build_widget import BuildWidget
from conf_widget import ConfWidget
from console_widget import ConsoleWidget
from header_widget import HeaderWidget
