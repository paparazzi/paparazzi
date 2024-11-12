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

import pathlib

from msgRecord.ivyRecorder import IvyRecorder,MessageLog
from msgRecord.qtMessageModel import FilteredIvyModel,IvyModel,\
                                        FieldItem,SenderItem,MessageItem,MessageClassItem,MessageSubgroupItem


from PyQt5.QtWidgets import QWidget,QApplication,\
                            QSizePolicy,QHeaderView,\
                            QTreeView,QVBoxLayout,QWidget,QTabWidget,QMenu,QMessageBox

from PyQt5.QtCore import Qt,QModelIndex,pyqtSlot,QPoint


from msgWidgets.messagesFilter import MessagesFilter

class MessagesView(QTreeView):
    def __init__(self, ivyModel:FilteredIvyModel,parent: QWidget | None = None) -> None:
        super().__init__(parent)
                
        self.ivyModel = ivyModel
        
        self.contextMenu = QMenu()
        
        self.setModel(self.ivyModel)
        
        self.setSortingEnabled(True)
        self.sortByColumn(0,Qt.SortOrder.AscendingOrder)
        
        self.setSizeAdjustPolicy(QTreeView.SizeAdjustPolicy.AdjustToContents)
        self.sizePolicy().setHorizontalPolicy(QSizePolicy.Policy.Minimum)
        self.sizePolicy().setVerticalPolicy(QSizePolicy.Policy.Minimum)
        self.header().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.setWordWrap(False)
        
        self.doubleClicked.connect(self.__expandAllOnDoubleClick)
        self.model().rowsInserted.connect(self._autoExpandTopItems)
        self.customContextMenuRequested.connect(self._onCustomContextMenu)
        
        self.setSelectionBehavior(QTreeView.SelectionBehavior.SelectRows)
        self.setSelectionMode(QTreeView.SelectionMode.SingleSelection)
        self.setDragDropMode(self.DragDropMode.DragOnly)
                        
        localdir =  pathlib.Path(__file__).parent.parent
        
        style = """
QTreeView::indicator:unchecked{
    image: url(:/icons/pin/normal/16.png);
}

QTreeView::indicator:checked{
    image: url(:/icons/pin/active_straight/16.png);
}

QTreeView::indicator:indeterminate{
    image: url(:/icons/pin/active/16.png);
}

/* 
QTreeView::branch:has-siblings:!adjoins-item {
    border-image: url(:/images/vline.png) 0;
}
*/

QTreeView::branch:has-siblings:adjoins-item {
    border-image: url(:/images/branch-more.png) 0;
}

QTreeView::branch:!has-children:!has-siblings:adjoins-item {
    border-image: url(:/images/branch-end.png) 0;
}

QTreeView::branch:has-children:!has-siblings:closed,
QTreeView::branch:closed:has-children:has-siblings {
        border-image: none;
        image: url(:/images/branch-closed.png);
}

QTreeView::branch:open:has-children:!has-siblings,
QTreeView::branch:open:has-children:has-siblings  {
        border-image: none;
        image: url(:/images/branch-open.png);
}
                            """.replace('url(:',f'url({localdir}')
                        
        self.setStyleSheet(style)
        
    @pyqtSlot(QPoint)
    def _onCustomContextMenu(self,point:QPoint):
        index = self.indexAt(point)
        if not(index.isValid()):
            return
        
        rootIndex = index.sibling(index.row(),0)
        item = self.model().itemFromIndex(rootIndex)
        
        def gen_cb(item:MessageItem,fstr:str):
            msg = item.msg
            field = msg.get_full_field(fstr)
            if not('int8' in field.typestr or 'char' in field.typestr):      
                return lambda: (item.toSubgroups(fstr) if QMessageBox.warning(self,'Confirm message grouping',
                        f"The field '{field.name}' in message '{msg.msg_name()}' is of type '{field.typestr}', which is likely to have a lot of different values. Are you sure you want to proceed ?",
                        QMessageBox.StandardButton.Cancel | QMessageBox.StandardButton.Yes,
                        QMessageBox.StandardButton.Cancel) == QMessageBox.StandardButton.Yes else None)
            else:
                return lambda: item.toSubgroups(fstr)
        
        menu = QMenu(self)
        if isinstance(item,FieldItem):
            parentItem = item.parent()
            
            if isinstance(parentItem,MessageItem):
                field = parentItem.msg.get_full_field(item.fieldName())
                if field.array_type:
                    return
                
                menu.addAction(f"Group {parentItem.msg.msg_name()} by {item.fieldName()}").triggered.connect(gen_cb(parentItem,item.fieldName()))
            else:
                act = menu.addAction("Clear grouping")
                msgitem = parentItem.parent()
                act.triggered.connect(msgitem.clearSubgroups)
                
                

        elif isinstance(item,MessageItem):
            if item.hasSubgroups():
                act = menu.addAction("Clear grouping")
                act.triggered.connect(item.clearSubgroups)
            else:
                submenu = menu.addMenu(f"Group by field:")
                
                for f in item.fieldMap.keys():
                    field = item.msg.get_full_field(f)
                    if field.array_type:
                        continue
                    
                    act = submenu.addAction(f)
                    act.triggered.connect(gen_cb(item,f))
                    

        elif isinstance(item,MessageSubgroupItem):
            parentItem = item.parent()
            act = menu.addAction("Clear grouping")
            act.triggered.connect(parentItem.clearSubgroups)

        elif isinstance(item,MessageClassItem):
            return
        elif isinstance(item,SenderItem):
            return
        
        menu.popup(self.viewport().mapToGlobal(point))

          
    @pyqtSlot(QModelIndex)
    def __expandAllOnDoubleClick(self,index:QModelIndex):
        if index.column() == 0:
            self.__expandAllOnDoubleClick(index.sibling(index.row(),1))
        
        rootIndex = index.sibling(index.row(),0)
            
        if self.isExpanded(rootIndex):
            self.collapse(rootIndex)
        else:
            self.expand(rootIndex)
            
    @pyqtSlot(QModelIndex,int,int)
    def _autoExpandTopItems(self,parent:QModelIndex,start:int,stop:int):
        if not(parent.isValid()):
            return
        else:
            index = parent.child(start,0)
            if not(self.isExpanded(parent)):
                self.expand(parent)
            
            if not(self.isExpanded(index)):
                self.expand(index)
    
    
class SenderMessagesView(MessagesView):
    def __init__(self, ivyModel:FilteredIvyModel,sender_id:int, parent: QWidget | None = None) -> None:
        super().__init__(ivyModel,parent)
        
        self.senderId = sender_id

        self.setRootIndex(self.ivyModel.senderIndex(sender_id))
        
        
    @pyqtSlot()
    def safeExpandAll(self):
        if self.model().messageCount(self.senderId) < 5:
            self.expandAll()
            
    @pyqtSlot(QModelIndex,int,int)
    def _autoExpandTopItems(self,parent:QModelIndex,start:int,stop:int):
        if not(parent.isValid()):
            return
        
        if not(parent.parent().isValid()):
            if not(self.isExpanded(parent)):
                self.expand(parent)



class MessagesWidget(QWidget):
    def __init__(self, ivy:IvyRecorder,ivyModel:IvyModel, filteredModel:FilteredIvyModel,parent: QWidget | None = None,flags: Qt.WindowFlags | Qt.WindowType = Qt.WindowType.Widget) -> None:
        super().__init__(parent, flags)
        
        self.filterWidget = MessagesFilter(self)
        
        self.ivy = ivy
        self.model = ivyModel
        self.model.setMultiSenderPinning(True if self.filterWidget.multiSenderPinning() == Qt.CheckState.Checked else False)
        self.filteredModel = filteredModel
        
        
        self.tabWidget = QTabWidget(self)
        self.tabWidget.setTabsClosable(False)
        self.tabWidget.tabBar().setMovable(True)
        
        self.vLayout =  QVBoxLayout(self)
        self.vLayout.addWidget(self.filterWidget)
        self.vLayout.addWidget(self.tabWidget)
        
        self.setLayout(self.vLayout)
        
        self.ivy.new_sender.connect(self.newSender)
        
        self.filterWidget.filteringChanged.connect(filteredModel.setFilterRegularExpression)

        self.filterWidget.pinFiltering.connect(filteredModel.setCheckedOnly)
        
        self.filterWidget.multiSenderPin.connect(ivyModel.setMultiSenderPinning)

        
    @pyqtSlot(int)
    def newSender(self,id:int):
        self.ivy.recordSender(id)
        self.model.update()
        
        newView = SenderMessagesView(self.filteredModel,id,self)
        
        self.filterWidget.filteringDone.connect(newView.safeExpandAll)
        
        self.tabWidget.addTab(newView,f"Sender {id}")

        
if __name__ == "__main__":
    
    app = QApplication([])
    
    ivy = IvyRecorder(buffer_size=1)
            
    window = MessagesWidget(ivy,None)
    
    app.aboutToQuit.connect(ivy.stop)
    
    window.show()
    app.exec()