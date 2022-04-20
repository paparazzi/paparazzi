# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
from PyQt5 import QtCore, QtGui
from programs_conf import *
import os
import utils
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from generated.ui_tools_list import Ui_ToolsList

ICON_SIZE = (40, 40)


class ToolMenu(QWidget, Ui_ToolsList):

    tool_clicked = QtCore.pyqtSignal(str)

    def __init__(self):
        super(ToolMenu, self).__init__()
        self.setupUi(self)
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint | QtCore.Qt.Popup)
        self.tools_buttons: Dict[str, QToolButton] = {}
        self.filter_lineedit.textChanged.connect(self.filter)
        self.setFocusProxy(self.filter_lineedit)
        # self.gridLayout.setContentsMargins(10, 10, 24, 10)
        # self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        # self.setWidgetResizable(True)

    def add_tool(self, t: Tool):
        button = QToolButton()
        button.setText(t.name)
        icon = QtGui.QIcon(os.path.join(utils.PAPARAZZI_HOME, "data", "pictures", "tools_icons", t.icon))
        button.setIcon(icon)
        button.setIconSize(QtCore.QSize(*ICON_SIZE))
        # button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        button.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        button.clicked.connect(lambda: self.tool_clicked.emit(button.text()))
        self.tools_buttons[t.name] = button
        self.content_widget.layout().addWidget(button)

    def filter(self, txt: str):
        for name, button in self.tools_buttons.items():
            show = txt.lower() in name.lower()
            button.setVisible(show)

    def keyPressEvent(self, e: QtGui.QKeyEvent) -> None:
        if e.key() == Qt.Key_Escape:
            self.releaseMouse()
            self.close()

    def show(self) -> None:
        super(ToolMenu, self).show()
        self.filter_lineedit.clear()
