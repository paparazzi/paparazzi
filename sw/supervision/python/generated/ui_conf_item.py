# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/ui_conf_item.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_FileConf(object):
    def setupUi(self, FileConf):
        FileConf.setObjectName("FileConf")
        FileConf.resize(306, 129)
        self.verticalLayout = QtWidgets.QVBoxLayout(FileConf)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.title_label = QtWidgets.QLabel(FileConf)
        self.title_label.setStyleSheet("font-weight: bold;")
        self.title_label.setObjectName("title_label")
        self.horizontalLayout_5.addWidget(self.title_label)
        self.edit_alt_button = QtWidgets.QPushButton(FileConf)
        self.edit_alt_button.setObjectName("edit_alt_button")
        self.horizontalLayout_5.addWidget(self.edit_alt_button)
        self.edit_button = QtWidgets.QToolButton(FileConf)
        self.edit_button.setObjectName("edit_button")
        self.horizontalLayout_5.addWidget(self.edit_button)
        self.select_button = QtWidgets.QToolButton(FileConf)
        self.select_button.setObjectName("select_button")
        self.horizontalLayout_5.addWidget(self.select_button)
        self.verticalLayout.addLayout(self.horizontalLayout_5)
        self.path_label = QtWidgets.QLabel(FileConf)
        self.path_label.setWordWrap(True)
        self.path_label.setOpenExternalLinks(False)
        self.path_label.setTextInteractionFlags(QtCore.Qt.LinksAccessibleByMouse|QtCore.Qt.TextSelectableByKeyboard|QtCore.Qt.TextSelectableByMouse)
        self.path_label.setObjectName("path_label")
        self.verticalLayout.addWidget(self.path_label)

        self.retranslateUi(FileConf)
        QtCore.QMetaObject.connectSlotsByName(FileConf)

    def retranslateUi(self, FileConf):
        _translate = QtCore.QCoreApplication.translate
        FileConf.setWindowTitle(_translate("FileConf", "Form"))
        self.title_label.setText(_translate("FileConf", "Flight Plan"))
        self.edit_alt_button.setText(_translate("FileConf", "Edit alt"))
        self.edit_button.setText(_translate("FileConf", "Edit"))
        self.select_button.setText(_translate("FileConf", "Select"))
        self.path_label.setText(_translate("FileConf", "/path/to/file.xml"))
