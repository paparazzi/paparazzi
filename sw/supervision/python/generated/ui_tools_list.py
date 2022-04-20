# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/tools_list.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_ToolsList(object):
    def setupUi(self, ToolsList):
        ToolsList.setObjectName("ToolsList")
        ToolsList.resize(328, 357)
        self.verticalLayout = QtWidgets.QVBoxLayout(ToolsList)
        self.verticalLayout.setObjectName("verticalLayout")
        self.filter_lineedit = QtWidgets.QLineEdit(ToolsList)
        self.filter_lineedit.setClearButtonEnabled(True)
        self.filter_lineedit.setObjectName("filter_lineedit")
        self.verticalLayout.addWidget(self.filter_lineedit)
        self.scrollArea = QtWidgets.QScrollArea(ToolsList)
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName("scrollArea")
        self.content_widget = QtWidgets.QWidget()
        self.content_widget.setGeometry(QtCore.QRect(0, 0, 308, 306))
        self.content_widget.setObjectName("content_widget")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.content_widget)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.scrollArea.setWidget(self.content_widget)
        self.verticalLayout.addWidget(self.scrollArea)

        self.retranslateUi(ToolsList)
        QtCore.QMetaObject.connectSlotsByName(ToolsList)

    def retranslateUi(self, ToolsList):
        _translate = QtCore.QCoreApplication.translate
        ToolsList.setWindowTitle(_translate("ToolsList", "Form"))
