# Copyright (C) 2024 Mael FEURGARD <mael.feurgard@enac.fr>
# 
# This file is part of messages_python.
# 
# messages_python is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# messages_python is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with messages_python.  If not, see <https://www.gnu.org/licenses/>.

from msgWidgets.messagesWidget import MessagesView,FilteredIvyModel


from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QWidget,QFrame

from PyQt5.QtCore import Qt


class Ui_Frame(object):
    def setupUi(self, ivy:FilteredIvyModel,Frame):
        Frame.setObjectName("Frame")
        Frame.resize(400, 300)
        self.verticalLayout = QtWidgets.QVBoxLayout(Frame)
        self.verticalLayout.setObjectName("verticalLayout")
        self.allPinnedLabel = QtWidgets.QLabel(Frame)
        self.allPinnedLabel.setObjectName("allPinnedLabel")
        self.verticalLayout.addWidget(self.allPinnedLabel)
        self.treeView = MessagesView(ivy,Frame)
        self.treeView.setObjectName("treeView")
        self.verticalLayout.addWidget(self.treeView)

        self.retranslateUi(Frame)
        QtCore.QMetaObject.connectSlotsByName(Frame)

    def retranslateUi(self, Frame):
        _translate = QtCore.QCoreApplication.translate
        Frame.setWindowTitle(_translate("Frame", "Frame"))
        self.allPinnedLabel.setText(_translate("Frame", "All pinned messages"))

class PinnedMessages(QFrame):
    def __init__(self, ivy:FilteredIvyModel,parent: QWidget | None = None, flags: Qt.WindowFlags | Qt.WindowType = Qt.WindowType.Widget) -> None:
        super().__init__(parent, flags)
        
        self.ui = Ui_Frame()
        self.ui.setupUi(ivy,self)
        
        
        