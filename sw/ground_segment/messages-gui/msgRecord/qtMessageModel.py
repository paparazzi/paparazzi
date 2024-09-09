# Copyright (C) 2024 Mael FEURGARD <mael.feurgard@enac.fr>
# 
# This file is part of python-pprz-messages.
# 
# python-pprz-messages is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# python-pprz-messages is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with python-pprz-messages.  If not, see <https://www.gnu.org/licenses/>.

import typing
import time
import math

import enum

from pprzlink.message import PprzMessageField

from msgRecord.ivyRecorder import IvyRecorder,MessageLog
from msgRecord.messageLog import NoMessageError

from PyQt5 import QtCore
from PyQt5.QtWidgets import QInputDialog,QMessageBox
from PyQt5.QtCore import Qt,QObject,QSortFilterProxyModel,QModelIndex,QTimer,pyqtSlot,pyqtSignal
from PyQt5.QtGui import QColor,QStandardItem,QStandardItemModel

#################### Columns mapping ####################

COLUMN_COUNT = 3


class FieldColumns(enum.IntEnum):
        ROOT = 0
        VALUE = 1
        ALT_VALUE = 2

assert len(FieldColumns) <= COLUMN_COUNT

class MessageSubgroupColumns(enum.IntEnum):
    ROOT = 0
    VALUE = 1
    ALT_VALUE = 2
    
assert len(MessageSubgroupColumns) <= COLUMN_COUNT

class MessageColumns(enum.IntEnum):
        ROOT = 0
        ID = 1
        RECEPTION = 2

assert len(MessageColumns) <= COLUMN_COUNT


class MessageClassColumns(enum.IntEnum):
        ROOT = 0
        
assert len(MessageClassColumns) <= COLUMN_COUNT


class SenderColumns(enum.IntEnum):
        ROOT = 0
        
assert len(SenderColumns) <= COLUMN_COUNT

#################### Helper function ####################

def format_field_vals(field:PprzMessageField,val:typing.Optional[typing.Any] = None) -> typing.Tuple[str,str]: 
    if val is None:
        val = field.val
       
    if field.format and '%' in field.format:
        valstr = field.format % val
    else:
        valstr = str(val)
    
    if field.unit and field.unit != 'none':
        valstr += " " + field.unit
    
    if field.is_enum:
        valstr += f" ({field.val_enum})"
        
        
    altstr = ""
    if field.val is not None and not(field.array_type):
            alt_coef = 1. if field.alt_unit_coef is None else field.alt_unit_coef
            
            if alt_coef != 1.:
                altstr = f"{field.val * field.alt_unit_coef:.3f}"
                
                if field.alt_unit:
                    altstr += " " + field.alt_unit
                    
        
    return valstr,altstr


#################### Specific items ####################

class FieldItem(QStandardItem):
    def __init__(self,name:str):
        super().__init__(name)
        self.setFieldName(name)
        
    def senderId(self) -> int:
        return self.parent().senderId()
        
    def setFieldName(self,name:str):
        self.setData(name,Qt.ItemDataRole.UserRole)
    
    def fieldName(self) -> str:
        return self.data(Qt.ItemDataRole.UserRole)

class MessageSubgroupItem(QStandardItem):
    def __init__(self,topMsg:MessageLog,fieldName:str,fieldVal):
        super().__init__(fieldName)
        self.fieldMap:dict[str,int] = dict() # Field_name -> Row_number
        self.msg = topMsg
        self.fieldName = fieldName
        self.fieldVal = fieldVal
        
    
    def senderId(self) -> int:
        return self.parent().senderId()
    
    def pinMessage(self,msg:MessageLog,field:typing.Optional[str],value):
        if field is None:
            self.checkChildren(value)
        else:
            try:
                r = self.fieldMap[field]
            except KeyError:
                return
            
            fieldItem:FieldItem = self.child(r,0)
            fieldItem.setData(value,Qt.ItemDataRole.CheckStateRole)
            
        self.setCheckFromChildren()
    
    def setCheckFromChildren(self):
        resultState = self.child(0,0).checkState()
        
        for i in range(1,self.rowCount()):
            pinItem = self.child(i,0)
            s = pinItem.checkState()
            if s != resultState:
                self.setCheckState(Qt.CheckState.PartiallyChecked)
                return
        
        self.setCheckState(resultState)
    
    def checkChildren(self,checked:typing.Optional[typing.Union[bool,Qt.CheckState]]=None):
        if checked is None:
            checked = self.checkState()
            
        if checked == Qt.CheckState.PartiallyChecked:
            return
            
        if isinstance(checked,bool):
            checked = Qt.CheckState.Checked if checked else Qt.CheckState.Unchecked
            
            
        for i in range(self.rowCount()):
            pinItem = self.child(i,0)
            pinItem.setCheckState(checked)
        
    def updateAllFields(self,msg:MessageLog):
        self.msg = msg
        
        for f in msg.fieldnames():
            if f == self.fieldName:
                continue
            
            self.updateField(msg,f)
    
    def updateField(self,msg:MessageLog,fieldname:str):
        self.msg = msg
        field = msg.get_full_field(fieldname)
        
        try:
            rowNumber = self.fieldMap[fieldname]
            
            fieldRootItem:FieldItem = self.child(rowNumber,FieldColumns.ROOT)
            fieldValueItem = self.child(rowNumber,FieldColumns.VALUE)
            fieldAltValItem = self.child(rowNumber,FieldColumns.ALT_VALUE)
        except KeyError:
            self.fieldMap[fieldname] = self.rowCount()
    
            fieldRootItem = FieldItem(field.name)
            fieldRootItem.setText(f"({field.typestr}) {field.name}")
            fieldRootItem.setCheckable(True)
            fieldRootItem.setEditable(False)
            fieldRootItem.setData(field.name,Qt.ItemDataRole.UserRole)
            
            fieldValueItem = QStandardItem()
            fieldValueItem.setEditable(False)
            
            fieldAltValItem = QStandardItem()
            fieldAltValItem.setEditable(False)
            
            newitems = [None] * COLUMN_COUNT#####
            newitems[FieldColumns.ROOT] = fieldRootItem
            newitems[FieldColumns.VALUE] = fieldValueItem
            newitems[FieldColumns.ALT_VALUE] = fieldAltValItem
            
            self.appendRow(newitems)
            
        
        fieldValueItem.setData(field.val,Qt.ItemDataRole.UserRole)
            
        valstr,altstr = format_field_vals(field)
        
        fieldValueItem.setText(valstr)
        fieldAltValItem.setText(altstr)
                    
        if field.val is not None and not(field.array_type):
            alt_coef = 1. if field.alt_unit_coef is None else field.alt_unit_coef
            
            fieldAltValItem.setData(alt_coef * field.val,Qt.ItemDataRole.UserRole)
            fieldAltValItem.setData(alt_coef,Qt.ItemDataRole.UserRole+1)

                
        
class MessageItem(QStandardItem):
    def __init__(self,msg:MessageLog):
        super().__init__(msg.msg_name())
        self.fieldMap:dict[str,int] = dict() # Field_name -> Row_number
        self.msg = msg
        
        self.groupedMap:dict = dict() # Value -> Row_number
        
    def hasSubgroups(self) -> bool:
        return self.msg.grouped()
        
    def senderId(self) -> int:
        return self.parent().senderId()
    
    def pinMessage(self,msg:MessageLog,field:typing.Optional[str],value) -> QModelIndex:
        if field is None:
            self.checkChildren(value)
        else:
            try:
                r = self.fieldMap[field]
            except KeyError:
                return
            
            fieldItem:FieldItem = self.child(r,0)
            fieldItem.setData(value,Qt.ItemDataRole.CheckStateRole)
            
        self.setCheckFromChildren()
        
        return self.index()
    
    def setCheckFromChildren(self):
        resultState = self.child(0,0).checkState()
        
        for i in range(1,self.rowCount()):
            pinItem = self.child(i,0)
            s = pinItem.checkState()
            if s != resultState:
                self.setCheckState(Qt.CheckState.PartiallyChecked)
                return
        
        self.setCheckState(resultState)
    
    def checkChildren(self,checked:typing.Optional[typing.Union[bool,Qt.CheckState]]=None):
        if checked is None:
            checked = self.checkState()
            
        if checked == Qt.CheckState.PartiallyChecked:
            return
            
        if isinstance(checked,bool):
            checked = Qt.CheckState.Checked if checked else Qt.CheckState.Unchecked
            
            
        for i in range(self.rowCount()):
            pinItem = self.child(i,0)
            pinItem.setCheckState(checked)
        
    def toSubgroups(self,fieldName:str):
        rowCount = self.rowCount()
        self.removeRows(0,rowCount)
        
        self.fieldMap.clear()
        self.groupedMap.clear()

        self.msg.groupBy(fieldName)

        m:IvyModel =self.model()
        m.update()

    
    def clearSubgroups(self):
        rowCount = self.rowCount()
        self.removeRows(0,rowCount)
        
        self.fieldMap.clear()
        self.groupedMap.clear()

        self.msg.clearGroupBy()

        m:IvyModel =self.model()
        m.update()

        
    def updateAllFields(self,msg:MessageLog):
        self.msg = msg

        if self.hasSubgroups():
            for v,m in self.msg.subgroups().items():
                self.updateSubgroup(m,v)
                
        else:
            for f in msg.fieldnames():
                self.updateField(msg,f)
        
    def updateSubgroup(self,submsg:MessageLog,val):
        field = self.msg.get_full_field(self.msg.groupedBy())
        
        try:
            rowNumber = self.groupedMap[val]
            
            submsgRootItem:MessageSubgroupItem = self.child(rowNumber,MessageSubgroupColumns.ROOT)
            submsgValueItem = self.child(rowNumber,MessageSubgroupColumns.VALUE)
            submsgAltValItem = self.child(rowNumber,MessageSubgroupColumns.ALT_VALUE)
        except KeyError:
            self.groupedMap[val] = self.rowCount()
    
            submsgRootItem = MessageSubgroupItem(self.msg,self.msg.groupedBy(),val)
            
            submsgRootItem.setText(f"({field.typestr}) {field.name}")
            submsgRootItem.setCheckable(True)
            submsgRootItem.setEditable(False)
            submsgRootItem.setData(field.name,Qt.ItemDataRole.UserRole)
            
            submsgValueItem = QStandardItem()
            submsgValueItem.setEditable(False)
            
            submsgAltValItem = QStandardItem()
            submsgAltValItem.setEditable(False)
            
            newitems = [None] * COLUMN_COUNT
            newitems[MessageSubgroupColumns.ROOT] = submsgRootItem
            newitems[MessageSubgroupColumns.VALUE] = submsgValueItem
            newitems[MessageSubgroupColumns.ALT_VALUE] = submsgAltValItem
            
            self.appendRow(newitems)
            
        submsgValueItem.setData(val,Qt.ItemDataRole.UserRole)
        
        valstr,altstr = format_field_vals(field,val)
        
        submsgValueItem.setText(valstr)
        submsgAltValItem.setText(altstr)
                                        
        if val is not None and not(field.array_type):
            alt_coef = 1. if field.alt_unit_coef is None else field.alt_unit_coef
            
            submsgAltValItem.setData(alt_coef * val,Qt.ItemDataRole.UserRole)
            submsgAltValItem.setData(alt_coef,Qt.ItemDataRole.UserRole+1)
                
        submsgRootItem.updateAllFields(submsg)
                

    
    def updateField(self,msg:MessageLog,fieldname:str):
        self.msg = msg
        field = msg.get_full_field(fieldname)
        
        try:
            rowNumber = self.fieldMap[fieldname]
            
            fieldRootItem:FieldItem = self.child(rowNumber,FieldColumns.ROOT)
            fieldValueItem = self.child(rowNumber,FieldColumns.VALUE)
            fieldAltValItem = self.child(rowNumber,FieldColumns.ALT_VALUE)
        except KeyError:
            self.fieldMap[fieldname] = self.rowCount()
    
            fieldRootItem = FieldItem(field.name)
            fieldRootItem.setText(f"({field.typestr}) {field.name}")
            fieldRootItem.setCheckable(True)
            fieldRootItem.setEditable(False)
            fieldRootItem.setData(field.name,Qt.ItemDataRole.UserRole)
            
            fieldValueItem = QStandardItem()
            fieldValueItem.setEditable(False)
            
            fieldAltValItem = QStandardItem()
            fieldAltValItem.setEditable(False)
            
            newitems = [None] * COLUMN_COUNT
            newitems[FieldColumns.ROOT] = fieldRootItem
            newitems[FieldColumns.VALUE] = fieldValueItem
            newitems[FieldColumns.ALT_VALUE] = fieldAltValItem
            
            self.appendRow(newitems)
            
        
        fieldValueItem.setData(field.val,Qt.ItemDataRole.UserRole)
            
        # if field.array_type:
        #     print(f"Message {msg.msg_name()}, field {field.name}: {field.val} ({type(field.val)})")
        
        valstr,altstr = format_field_vals(field)
        
        fieldValueItem.setText(valstr)
        fieldAltValItem.setText(altstr)
                    
        if field.val is not None and not(field.array_type):
            alt_coef = 1. if field.alt_unit_coef is None else field.alt_unit_coef
            
            fieldAltValItem.setData(alt_coef * field.val,Qt.ItemDataRole.UserRole)
            fieldAltValItem.setData(alt_coef,Qt.ItemDataRole.UserRole+1)
                    
                

class MessageClassItem(QStandardItem):
    COLOR_FREQ = True
    EXTINCTION_TIME = 5 # Time before total fade to black of freq coloring, in seconds
    
    def __init__(self,name:str):
        super().__init__(name)
        self.messagesMap:dict[int,int] = dict() # Message_id -> Row_number
        
    def senderId(self) -> int:
        return self.parent().senderId()
    
    def pinMessage(self,msg:MessageLog,field:typing.Optional[str],value) -> QModelIndex:
        try:
            r = self.messagesMap[msg.msg_id()]
        except KeyError:
            return
        
        msgItem:MessageItem = self.child(r,0)
        return msgItem.pinMessage(msg,field,value)
        
        
        
    def updateMessage(self,msg:MessageLog):
        id = msg.msg_id()
        name = msg.msg_name()
        timestamp = msg.newest().timestamp
        dt = (time.time_ns() - timestamp)/10e9
        
        try:
            freq = msg.meanFreq()
        except NoMessageError:
            freq = 0
        
        try:
            rowNumber = self.messagesMap[id]
            
            msgRootItem:MessageItem = self.child(rowNumber, MessageColumns.ROOT)
            msgReceptionItem = self.child(rowNumber,MessageColumns.RECEPTION)
            
            
        except KeyError:
            self.messagesMap[id] = self.rowCount()
            
            msgRootItem = MessageItem(msg)
            msgRootItem.setEditable(False)
            msgRootItem.setCheckable(True)
            msgRootItem.setAutoTristate(True)
            msgRootItem.setCheckState(Qt.CheckState.Unchecked)
            msgRootItem.setData(msg.msg_name(),Qt.ItemDataRole.UserRole)
            # msgItem.setIcon(self.model().pinIcon)            
            
            msgIdItem = QStandardItem(str(id))
            msgIdItem.setData(id,Qt.ItemDataRole.DisplayRole)
            msgIdItem.setData(id,Qt.ItemDataRole.UserRole)
            msgIdItem.setEditable(False)
            
            msgReceptionItem = QStandardItem()
            msgReceptionItem.setEditable(False)
            
            newitems = [None] * COLUMN_COUNT
            newitems[MessageColumns.ROOT] = msgRootItem
            newitems[MessageColumns.ID] = msgIdItem
            newitems[MessageColumns.RECEPTION] = msgReceptionItem
            
            self.appendRow(newitems)
            
            print(f"Added row for {name}")

        msgReceptionItem.setData(dt,Qt.ItemDataRole.UserRole)
        msgReceptionItem.setText(f" {dt:.0f}s ({freq:.1f} Hz) ")
                
        msgRootItem.updateAllFields(msg)
        
        intfreq = int(freq*10)
        if intfreq >= 100:
            intfreq = round(intfreq/100)*100
        
        msgReceptionItem.setData(intfreq,Qt.ItemDataRole.UserRole)
        
        if self.COLOR_FREQ:
            green = int(max(255*(1 - (dt/self.EXTINCTION_TIME)),0))
                    
            back_color = QColor(0,green,0)
            
            srgb = back_color.getRgbF()
                    
            light_remap = lambda v : v/12.92 if v <= 0.03928 else ((v+0.055)/1.055)**2.4
            RGB = tuple(light_remap(v) for v in srgb)
            
            bg_relative_luminance = RGB[0] * 0.2126 + RGB[1] * 0.7152 + RGB[2] * 0.0722
            
            # Original treshold
            # if bg_relative_luminance > math.sqrt(1.05*0.05)-0.05:
            
            # Adjusted for personal preferences
            if bg_relative_luminance > math.sqrt(1.05*0.05)+0.01:
                front_color = QColor(0,0,0)
            else:
                front_color = QColor(255,255,255)
                
            msgReceptionItem.setBackground(back_color)
            msgReceptionItem.setForeground(front_color)
        

class SenderItem(QStandardItem):
    def __init__(self,senderId:int):
        super().__init__(str(senderId))
        self.setSenderId(senderId)
        self.classMap:dict[int,int] = dict() # Class_id -> Row_number
        
    def setSenderId(self,id:int):
        self.setData(id,Qt.ItemDataRole.UserRole)
        
    def senderId(self) -> int:
        return self.data(Qt.ItemDataRole.UserRole)
        
    def updateMessage(self,msg:MessageLog):
        class_name = msg.msg_class()
        class_id = msg.class_id()
        
        try:
            rowNumber = self.classMap[class_id]
            
            clsRootItem = self.child(rowNumber,MessageClassColumns.ROOT)
        except KeyError:
            self.classMap[class_id] = self.rowCount()
            
            clsRootItem = MessageClassItem(class_name)
            clsRootItem.setData(class_name,Qt.ItemDataRole.UserRole)
            
            newItems = [QStandardItem() for i in range(COLUMN_COUNT)]
            newItems[MessageClassColumns.ROOT] = clsRootItem
            
            for i in newItems:
                i.setEditable(False)
                i.setDragEnabled(False)
            
            self.appendRow(newItems)
            
        clsRootItem.updateMessage(msg)
        
    def pinMessage(self,msg:MessageLog,field:typing.Optional[str],value) -> QModelIndex:
        try:
            r = self.classMap[msg.class_id()]
        except KeyError:
            return
        
        msgClassItem:MessageClassItem = self.child(r,0)
        return msgClassItem.pinMessage(msg,field,value)
        
        
    def updateMessageClass(self,ivy:IvyRecorder,class_id:int):
        senderId = self.senderId()
        try:
            msgDict = ivy.records[senderId][class_id]
        except KeyError:
            return
        
        for msg in msgDict.values():
            self.updateMessage(msg)
            

#################### Model ####################

class IvyModel(QStandardItemModel):
    _COLS = enum.IntEnum('MessagesModelHeader',["Class"],start=0)
    newPin = pyqtSignal(int,int,int,str,bool)
    multiPinningDone = pyqtSignal()
    
    def __init__(self,ivy_recorder:IvyRecorder, parent: typing.Optional[QObject] = None):
        super().__init__(parent)
                
        self.setHorizontalHeaderLabels(["Name","Id/Value","Time/Alt Value"])
        
        self.ivyRecorder = ivy_recorder
        
        self.senderMap:dict[int,int] = dict() # Sender_id -> Row_number
              
        # self.ivyRecorder.data_updated.connect(self.updateModel)
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        
        self.timerDt = 500 # Time between updates, in ms
        
        self.timer.start(self.timerDt) 
        
        self.__multiSenderPinning:bool = False # Allow pinning accross all different senders
        
    def multiSenderPinning(self) -> bool:
        return self.__multiSenderPinning
        
    def setMultiSenderPinning(self,b:bool):
        self.__multiSenderPinning = b
        
    def supportedDropActions(self) -> Qt.DropActions:
        return Qt.DropAction.CopyAction
        
    
    ############### MIME aspects (Drag N Drop) ###############
    
    def mimeTypes(self) -> typing.List[str]:
        return ["text/plain"]
    
    def __mimeStrFromIndex(self,index:QModelIndex) -> typing.Union[str,None,bool]:        
        item = self.itemFromIndex(index)
        parent = item.parent()
        rootItem = parent.child(item.row(),0)
        
        if isinstance(rootItem,MessageClassItem) or isinstance(rootItem,SenderItem):
            return None
        
        elif isinstance(rootItem,MessageItem):
            sender = rootItem.senderId()
            class_name = rootItem.msg.msg_class()
            msg_name = rootItem.msg.msg_name()
            
            return f"{sender}:{class_name}:{msg_name}"
            
        elif isinstance(rootItem,FieldItem):
            parent:MessageItem
            
            sender = parent.senderId()
            class_name = parent.msg.msg_class()
            msg_name = parent.msg.msg_name()
            field_name = rootItem.fieldName()
            
            field = parent.msg.get_full_field(field_name)
            field_type = field.typestr
            field_scale = field.alt_unit_coef
            
            if '[' in field_type:
                array_len = len(field.val)
                array_range,ok = QInputDialog.getText(None,
                                                   "Input index or range",
                                                   "Either a number, or a range (both inclusive)",
                                                   text=f"0-{array_len-1}",
                                                   inputMethodHints=Qt.InputMethodHint.ImhFormattedNumbersOnly)
                if not(ok):
                    return False
            else:
                array_range = None
            
            if field_scale is None:
                field_scale = 1.
            
            if array_range is None:
                return f"{sender}:{class_name}:{msg_name}:{field_name}:{field_scale}"
            else:
                return f"{sender}:{class_name}:{msg_name}:{field_name}[{array_range}]:{field_scale}"
        
        else:
            return None
        
    def mimeData(self, indexes: typing.Iterable[QModelIndex]) -> QtCore.QMimeData:
        data = QtCore.QMimeData()
        
        for index in indexes:
            if index.isValid():
                str_data = self.__mimeStrFromIndex(index)
                if str_data is not None:
                    if str_data is False:
                        return
                    data.setText(str_data)
                    break
        
        return data    
    
    ############### setData (modified for pinning) ###############
    
    def setData(self, index: QModelIndex, value: typing.Any, role: int = ...) -> bool:
        ret = super().setData(index, value, role)
        
        item = self.itemFromIndex(index)
        parent = item.parent()
        if role == Qt.ItemDataRole.CheckStateRole:
            msg = None
            field = None 
            if isinstance(item,MessageItem):
                item.checkChildren(value)
                msg = item.msg
            elif isinstance(parent,MessageItem):
                parent.setCheckFromChildren()
                msg = parent.msg
                field = item.fieldName()
                
            self.newPin.emit(item.senderId(),msg.class_id(),msg.msg_id(),"" if field is None else field, value)
                
            if self.multiSenderPinning() and msg is not None:
                self.multiPin(item.senderId(),msg,field,value)
                self.multiPinningDone.emit()
                
                
        return ret
    
    def multiPin(self,senderId:int,msg:MessageLog,field:typing.Optional[str],value):
        for i in range(self.rowCount()):
            senderItem:SenderItem = self.item(i,0)
            senderItem.pinMessage(msg,field,value)
            
            
            
            
    def pauseUpdates(self,b:bool):
        if b:
            self.timer.stop()
        else:
            self.timer.start()
    
    ############### Updating the model ###############
    
    @pyqtSlot()
    def update(self):
        for senderId in self.ivyRecorder.records.keys():
            try:
                rowNumber = self.senderMap[senderId]
                
                senderItem:SenderItem = self.item(rowNumber,0)
            except KeyError:
                self.senderMap[senderId] = self.rowCount() 
                
                senderItem = SenderItem(senderId)
                
                newItems = [QStandardItem()] * COLUMN_COUNT
                newItems[SenderColumns.ROOT] = senderItem
                
                for i in newItems:
                    i.setEditable(False)
                    i.setDragEnabled(False)
                
                self.appendRow(newItems)
            
            for clsId in self.ivyRecorder.records[senderId].keys():
                senderItem.updateMessageClass(self.ivyRecorder,clsId)
                            


class FilteredIvyModel(QSortFilterProxyModel):
    def __init__(self, ivyModel:IvyModel, parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        
        self.setSourceModel(ivyModel)
        self.setFilterCaseSensitivity(Qt.CaseSensitivity.CaseInsensitive)
        self.setSortRole(Qt.ItemDataRole.UserRole)
        
        self.__checkedOnly = False
        
        ivyModel.dataChanged.connect(lambda tl,br,r : self.dataChanged.emit(self.mapFromSource(tl),self.mapFromSource(br),r))
        ivyModel.multiPinningDone.connect(lambda : self.invalidateFilter())
    
    def senderIndex(self,senderId:int) -> QModelIndex:
        ivyModel:IvyModel = self.sourceModel()
        ivyIndex = ivyModel.index(ivyModel.senderMap[senderId],0)
        return self.mapFromSource(ivyIndex)
    
    def messageCount(self,senderId:int) -> int:
        self.blockSignals(True)
        total = 0
        srcModel:IvyModel = self.sourceModel()
        srcSenderIndex = srcModel.index(srcModel.senderMap[senderId],0)
        senderIndex = self.mapFromSource(srcSenderIndex)
        
        
        classCount = self.rowCount(senderIndex)
        for i in range(classCount):
            total += self.rowCount(self.index(i,0,senderIndex))
        self.blockSignals(False)
        return total
    
    def setCheckedOnly(self,b:bool):
        self.__checkedOnly = b
        self.invalidateFilter()
    
    def itemFromIndex(self, index:QModelIndex) -> QStandardItem:
        return self.sourceModel().itemFromIndex(self.mapToSource(index))
    
    def filterAcceptsRow(self, source_row: int, source_parent: QModelIndex) -> bool:
        firstIndex = source_parent.child(source_row,0)
        model:IvyModel = self.sourceModel()
        item = model.itemFromIndex(firstIndex)
        
        if self.__checkedOnly:
            if isinstance(item,MessageItem) or isinstance(item,FieldItem):
                try:
                    checkstatus = item.checkState() != Qt.CheckState.Unchecked
                except AttributeError:
                    checkstatus = True
            else:
                checkstatus = True
        else:
            checkstatus = True
        
        regex = self.filterRegularExpression()
        
        if len(regex.pattern()) == 0:
            regex_result = True
        elif isinstance(item,MessageItem):
            msgName = item.msg.msg_name()
            msgId = item.msg.msg_id()
            
            regex_result = regex.match(msgName).hasMatch() or regex.match(str(msgId)).hasMatch()
            
            if not(regex_result):
                # Child rows results:
                for i in range(item.rowCount()):
                    if self.filterAcceptsRow(i,item.index()):
                        regex_result = True
                        break
            
        elif isinstance(item,FieldItem):
            fieldName = item.fieldName()
            regex_result = regex.match(fieldName).hasMatch()
            
            
            if not (regex_result):
                msgItem:MessageItem = item.parent()
                msgName = msgItem.msg.msg_name()
                msgId = msgItem.msg.msg_id()
        
                regex_result = regex.match(msgName).hasMatch() or regex.match(str(msgId)).hasMatch()
            
        else:
            regex_result = True
            
        return checkstatus and regex_result
    
    