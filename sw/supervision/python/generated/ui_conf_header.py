# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui/conf_header.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_ConfHeader(object):
    def setupUi(self, ConfHeader):
        ConfHeader.setObjectName("ConfHeader")
        ConfHeader.resize(664, 192)
        self.horizontalLayout = QtWidgets.QHBoxLayout(ConfHeader)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.id_spinBox = QtWidgets.QSpinBox(ConfHeader)
        self.id_spinBox.setMaximum(255)
        self.id_spinBox.setObjectName("id_spinBox")
        self.horizontalLayout.addWidget(self.id_spinBox)
        self.color_button = QtWidgets.QToolButton(ConfHeader)
        self.color_button.setStyleSheet("")
        self.color_button.setText("")
        self.color_button.setObjectName("color_button")
        self.horizontalLayout.addWidget(self.color_button)
        self.ac_combo = QtWidgets.QComboBox(ConfHeader)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ac_combo.sizePolicy().hasHeightForWidth())
        self.ac_combo.setSizePolicy(sizePolicy)
        self.ac_combo.setEditable(False)
        self.ac_combo.setObjectName("ac_combo")
        self.horizontalLayout.addWidget(self.ac_combo)
        self.menu_button = QtWidgets.QToolButton(ConfHeader)
        self.menu_button.setContextMenuPolicy(QtCore.Qt.ActionsContextMenu)
        self.menu_button.setText("")
        icon = QtGui.QIcon.fromTheme("format-justify-fill")
        self.menu_button.setIcon(icon)
        self.menu_button.setObjectName("menu_button")
        self.horizontalLayout.addWidget(self.menu_button)
        self.refresh_button = QtWidgets.QToolButton(ConfHeader)
        icon = QtGui.QIcon.fromTheme("view-refresh")
        self.refresh_button.setIcon(icon)
        self.refresh_button.setObjectName("refresh_button")
        self.horizontalLayout.addWidget(self.refresh_button)
        self.save_button = QtWidgets.QToolButton(ConfHeader)
        icon = QtGui.QIcon.fromTheme("document-save")
        self.save_button.setIcon(icon)
        self.save_button.setObjectName("save_button")
        self.horizontalLayout.addWidget(self.save_button)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.label_4 = QtWidgets.QLabel(ConfHeader)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout.addWidget(self.label_4)
        self.set_combo = QtWidgets.QComboBox(ConfHeader)
        self.set_combo.setObjectName("set_combo")
        self.horizontalLayout.addWidget(self.set_combo)
        self.new_ac_action = QtWidgets.QAction(ConfHeader)
        icon = QtGui.QIcon.fromTheme("document-new")
        self.new_ac_action.setIcon(icon)
        self.new_ac_action.setObjectName("new_ac_action")
        self.remove_ac_action = QtWidgets.QAction(ConfHeader)
        icon = QtGui.QIcon.fromTheme("edit-delete")
        self.remove_ac_action.setIcon(icon)
        self.remove_ac_action.setObjectName("remove_ac_action")
        self.duplicate_action = QtWidgets.QAction(ConfHeader)
        icon = QtGui.QIcon.fromTheme("edit-copy")
        self.duplicate_action.setIcon(icon)
        self.duplicate_action.setObjectName("duplicate_action")
        self.rename_action = QtWidgets.QAction(ConfHeader)
        icon = QtGui.QIcon.fromTheme("format-text-italic")
        self.rename_action.setIcon(icon)
        self.rename_action.setObjectName("rename_action")

        self.retranslateUi(ConfHeader)
        self.menu_button.clicked.connect(self.menu_button.showMenu)
        QtCore.QMetaObject.connectSlotsByName(ConfHeader)

    def retranslateUi(self, ConfHeader):
        _translate = QtCore.QCoreApplication.translate
        ConfHeader.setWindowTitle(_translate("ConfHeader", "Form"))
        self.refresh_button.setToolTip(_translate("ConfHeader", "refresh Aircraft"))
        self.refresh_button.setText(_translate("ConfHeader", "..."))
        self.save_button.setToolTip(_translate("ConfHeader", "save conf"))
        self.save_button.setText(_translate("ConfHeader", "..."))
        self.label_4.setText(_translate("ConfHeader", "Set"))
        self.new_ac_action.setText(_translate("ConfHeader", "New AC"))
        self.remove_ac_action.setText(_translate("ConfHeader", "Remove"))
        self.duplicate_action.setText(_translate("ConfHeader", "Duplicate"))
        self.rename_action.setText(_translate("ConfHeader", "Rename"))
