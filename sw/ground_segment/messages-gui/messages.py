#!/usr/bin/env python3

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

import typing

from msgRecord.ivyRecorder import IvyRecorder
from msgRecord.qtMessageModel import IvyModel,FilteredIvyModel

from PyQt5.QtWidgets import QWidget,QMainWindow,QApplication,\
                            QVBoxLayout,QTabWidget,QSplitter,QTreeView
                            

from PyQt5.QtCore import Qt,pyqtSlot

from msgWidgets.messagesWidget import MessagesWidget
from msgWidgets.pinnedMessagesView import PinnedMessages


class MessagesMain(QSplitter):
    def __init__(self, ivy:IvyRecorder,parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setOrientation(Qt.Orientation.Vertical)
        
        self.ivyRecorder = ivy
        
        self.model = IvyModel(ivy)
        self.filteredModel = FilteredIvyModel(self.model)
        
        self.addWidget(MessagesWidget(ivy,self.model,self.filteredModel,self))
        
        self.pinnedModel = FilteredIvyModel(self.model)
        self.pinnedModel.setCheckedOnly(True)
        self.pinnedWidget = PinnedMessages(self.pinnedModel,self)
        
        self.setStyleSheet("""
QSplitter::handle {
    background-color: #d61a1f;
    border: 1px solid #777;
    width: 13px;
    margin-left: 2px;
    margin-right: 2px;
    border-radius: 4px;
}
        """)
        
        self.addWidget(self.pinnedWidget)
        
        self.setCollapsible(0,False)
        ss = self.sizes()
        ss[-1] = 0
        self.setSizes(ss)
        
        self.showPinnedOnce = False
        
        self.model.newPin.connect(self.showPinned)
        
    def showPinned(self,*args):
        if not(self.showPinnedOnce):
            self.showPinnedOnce = True
            ss = self.sizes()
            if ss[-1] == 0:
                ss[0] = ss[0]//2
                ss[-1] = ss[0]
            self.setSizes(ss)
        



if __name__ == "__main__":
    app = QApplication([])
    app.setApplicationName("messages")
    
    ivy = IvyRecorder(buffer_size=1)
    window = QMainWindow()
    window.setCentralWidget(MessagesMain(ivy,window))
    window.setWindowTitle("Paparazzi link Messages")
    
    app.aboutToQuit.connect(ivy.stop)
    
    window.show()
    app.exec()