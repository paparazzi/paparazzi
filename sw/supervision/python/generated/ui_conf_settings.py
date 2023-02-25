# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/ui_conf_settings.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_SettingsConf(object):
    def setupUi(self, SettingsConf):
        SettingsConf.setObjectName("SettingsConf")
        SettingsConf.resize(400, 300)
        self.verticalLayout = QtWidgets.QVBoxLayout(SettingsConf)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(SettingsConf)
        self.label.setStyleSheet("font-weight: bold;")
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.add_setting_button = QtWidgets.QToolButton(SettingsConf)
        icon = QtGui.QIcon.fromTheme("list-add")
        self.add_setting_button.setIcon(icon)
        self.add_setting_button.setObjectName("add_setting_button")
        self.horizontalLayout.addWidget(self.add_setting_button)
        self.remove_setting_button = QtWidgets.QToolButton(SettingsConf)
        icon = QtGui.QIcon.fromTheme("list-remove")
        self.remove_setting_button.setIcon(icon)
        self.remove_setting_button.setObjectName("remove_setting_button")
        self.horizontalLayout.addWidget(self.remove_setting_button)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.settings = QtWidgets.QListWidget(SettingsConf)
        self.settings.setObjectName("settings")
        self.verticalLayout.addWidget(self.settings)

        self.retranslateUi(SettingsConf)
        QtCore.QMetaObject.connectSlotsByName(SettingsConf)

    def retranslateUi(self, SettingsConf):
        _translate = QtCore.QCoreApplication.translate
        SettingsConf.setWindowTitle(_translate("SettingsConf", "Form"))
        self.label.setText(_translate("SettingsConf", "Settings"))
        self.add_setting_button.setText(_translate("SettingsConf", "..."))
        self.remove_setting_button.setText(_translate("SettingsConf", "..."))
