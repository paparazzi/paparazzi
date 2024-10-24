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

from PyQt5.QtWidgets import QWidget,QApplication

from PyQt5.QtCore import Qt,QObject,pyqtSignal

from generated.messagesFilter import Ui_Form

class MessagesFilter(QWidget):
    
    filteringChanged = pyqtSignal(str) # Current text for filtering
    pinFiltering = pyqtSignal(bool) # Are we keeping only pinned messages ?
    multiSenderPin = pyqtSignal(bool) # Pinning across all senders
    filteringDone = pyqtSignal()
    
    def __init__(self, parent: QWidget | None = None, flags: Qt.WindowFlags | Qt.WindowType = Qt.WindowType.Widget) -> None:
        super().__init__(parent, flags)
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        
        self.ui.filterLineEdit.textChanged.connect(self.__emitNewFilter)
        self.ui.filterLineEdit.editingFinished.connect(self.filteringDone.emit)
        
        self.ui.pinCheckBox.stateChanged.connect(
            lambda t : self.pinFiltering.emit(True if t == Qt.CheckState.Checked else False)
        )
        
        self.ui.multiSenderPinCheckBox.stateChanged.connect(
            lambda t : self.multiSenderPin.emit(True if t == Qt.CheckState.Checked else False)
        )
        
    def __emitNewFilter(self):
        self.filteringChanged.emit(self.ui.filterLineEdit.text())
    
    def pinFilter(self) -> Qt.CheckState:
        return self.ui.pinCheckBox.checkState()
    
    def multiSenderPinning(self) -> Qt.CheckState:
        return self.ui.multiSenderPinCheckBox.checkState()


if __name__ == "__main__":
    app = QApplication([])
            
    window = MessagesFilter()
    
    window.show()
    app.exec()