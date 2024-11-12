#!/usr/bin/env python3

import typing
import dataclasses

import numpy as np
import time


import pyqtgraph as pg
from pyqtgraph.GraphicsScene.mouseEvents import MouseClickEvent

from PyQt5.QtCore import Qt,QTimer,QPoint,QPointF,QObject, pyqtSlot, pyqtSignal
from PyQt5.QtGui import QDropEvent,QDragEnterEvent,QPen
from PyQt5.QtWidgets import QAction,QActionGroup,QMenu
                            
from msgRecord.messageLog import MessageLog,MessageIndex,FieldIndex
from msgRecord.ivyRecorder import IvyRecorder

from pprzlink.message import PprzMessage

# Fetch color palette from: http://tsitsul.in/blog/coloropt/
# Bright and Dark:
BRIGHT_RGBS = [(239,230,69),(233,53,161),(0,227,255),(225,86,44),(83,126,255),(0,203,133),(238,238,238)]
DARK_RGBS = [(0, 89, 0), (0, 0, 120), (73, 13, 0), (138, 3, 79), (0, 90, 138), (68, 53, 0), (88, 88, 88)]

HIGHLIGHT_COUNT = 5
ALPHA_START = 150
ALPHA_STOP = 50
LINEWIDTH_START = 1
LINEWIDTH_STOP = 8


class SelectablePlotDataItem(pg.PlotDataItem):
    deleteMe = pyqtSignal(object)
    
    def __init__(self, *args, **kargs):
        
        ##### Setup Highligh curves #####
        
        try:
            base_pen = kargs['pen']
        except KeyError:
            base_pen = BRIGHT_RGBS[-1]
            
        if not(isinstance(base_pen,QPen)):
            base_pen = pg.mkPen(base_pen)
            kargs['pen'] = base_pen
        
        
        base_color = base_pen.color()
        base_alpha = base_color.alpha()
        base_width = base_pen.width()
        
        self._highlightItems:list[pg.PlotDataItem] = []
        
        alphas = np.linspace(ALPHA_START,ALPHA_STOP,HIGHLIGHT_COUNT,endpoint=True,dtype=int)
        lws = np.linspace(LINEWIDTH_START,LINEWIDTH_STOP,HIGHLIGHT_COUNT,endpoint=True,dtype=int)
        
        
        
        for alpha, lw in zip(alphas, lws):
            base_color.setAlpha(alpha)
            base_pen.setColor(base_color)
            base_pen.setWidth(lw)
            
            self._highlightItems.append(pg.PlotDataItem([], [],
                                            pen=QPen(base_pen)))
            
        ##### Restore parameters for main curve plotting #####
        
        base_color.setAlpha(base_alpha)
        base_pen.setColor(base_color)
        base_pen.setWidth(base_width)
        
        super().__init__(*args, **kargs)
        
        self.setFlag(self.GraphicsItemFlag.ItemIsSelectable,True)
        self.setAcceptedMouseButtons(Qt.MouseButton.LeftButton | Qt.MouseButton.RightButton)
        self.setCurveClickable(True,24)
        
        #TODO : Change that, it's black magick
        self.curve.mouseClickEvent = self.mouseClickEvent
        
        
    def mouseClickEvent(self, ev:MouseClickEvent):
        if not self.curve.clickable:
            return
        if self.curve.mouseShape().contains(ev.pos()):
            ev.accept()
            self.sigClicked.emit(self, ev)
            
            self.setSelected(True)
            
            if ev.button() == Qt.MouseButton.RightButton:
                self._generateContextMenu(ev.screenPos().toQPoint())
            
        
    def _generateContextMenu(self,pos:typing.Union[QPoint,QPointF]):
        menu = QMenu(f'{self.curve.name()}',self.window())
        
        
        remove_act = QAction("Remove",self)
        remove_act.triggered.connect(lambda : self.deleteMe.emit(self))
                
        menu.addAction(remove_act)
        
        menu.exec(pos,None)
    
    # def contextMenuEvent(self, event: QGraphicsSceneContextMenuEvent) -> None:
    #     return
                
        
    def setData(self, *args, **kargs):
        r = super().setData(*args, **kargs)
        
        if self.isSelected():
            for hp in self._highlightItems:
                hp.setData(*args,**kargs)
        else:
            for hp in self._highlightItems:
                hp.setData([],[])
                
        return r

    def highlightCurves(self):
        return self._highlightItems

class FieldPlotInfo(QObject):
    deleteMe= pyqtSignal(QObject)
    
    def __init__(self,index:FieldIndex,plotItem:SelectablePlotDataItem,rescale:float = 1., parent: QObject | None = None) -> None:
        super().__init__(parent)
        
        self.index = index
        self.plotItem = plotItem
        self.rescale = rescale
        
        self.plotItem.deleteMe.connect(lambda : self.deleteMe.emit(self))
        
    def updatePlot(self,times:np.ndarray,values:np.ndarray):
        self.plotItem.setData(times,values*self.rescale)
    
    @staticmethod
    def from_MIMEtxt(txt:str, line_id:int) -> tuple[list,int]:
        
        # Expected format for field description:
        # "{sender}:{class_name}:{msg_name}:{field_name}:{field_scale}"
        # OR, if the field is of array type (range is both inclusive):
        # "{sender}:{class_name}:{msg_name}:{field_name}[{array_range}]:{field_scale}"
        split = txt.split(':')
        
        try:
            assert len(split) >= 4
        except AssertionError as e:
            print(f"Unexpected splitted length: {len(split)} (instead of at least 4)")
            print(f"Split is:\n{split}\nText is:\n{txt}")
            raise e
        
        sender = int(split[0])
        class_name = split[1]
        msg_name = split[2]
        field_info = split[3]
        # field_scale = float(split[4])
        
        
        msg = PprzMessage(class_name,msg_name)
        
        if '[' in field_info:
            # This is an array field
            field_info = field_info.split('[')
            field_name = field_info[0]
            array_range_txt = field_info[1][:-1] # Remove trailing ']'
            if '-' in array_range_txt:
                art_split = array_range_txt.split('-')
                index_range = range(int(art_split[0]),int(art_split[1])+1)
            else:
                index_range = [int(array_range_txt)]
            
        else:
            field_name = field_info
            index_range = [None]
            
        field = msg.get_full_field(field_name)
        if field.alt_unit_coef is not None:
            field_scale = field.alt_unit_coef
            field_unit = field.alt_unit
        else:
            field_scale = 1.
            field_unit = field.unit
            

        output = []
        for e in index_range:
            index = FieldIndex.from_ints(sender,msg.class_id,msg.msg_id,field_name,e)
            
            title = f"{sender}:{class_name}:{msg_name}:{field_name}" + ("" if e is None else f"[{e}]") + ("(a.u.)" if field_unit is None else f" ({field_unit})")
            
            color = pg.mkColor(BRIGHT_RGBS[line_id % len(BRIGHT_RGBS)])
            color.setAlphaF(0.7)
            pen = pg.mkPen(color)
            
            pltitem = SelectablePlotDataItem([],[],
                            title=title,
                            name=title,
                            pen=pen)
                        
            line_id = (line_id+1) % len(BRIGHT_RGBS)
            
            output.append(FieldPlotInfo(index,pltitem,field_scale))
                        
        return output,line_id
            
        
    def getMIMEtxt(self) -> str:
        sender = self.index.sender_id
        msg = self.index.pprzMsg()
        class_name = msg.msg_class
        msg_name = msg.name
        
        field_name = self.index.field
        
        full_field = msg.get_full_field(field_name)
        field_scale = full_field.alt_unit_coef
        if field_scale is None:
            field_scale = 1.
        
        
        if self.index.array_index is None:
            txt = f"{sender}:{class_name}:{msg_name}:{field_name}:{field_scale}"
        else:
            txt = f"{sender}:{class_name}:{msg_name}:{field_name}[{self.index.array_index}]:{field_scale}"
        
        return txt



class PlotWidget(pg.PlotWidget):
    def __init__(self, ivy:IvyRecorder, parent=None, background='default', **kargs):
        
        super().__init__(parent, background, None, **kargs)
        
        self.ivyRecorder = ivy
        
        self.setAcceptDrops(True)
        
        self.plotItem.setAcceptDrops(True)
        self.plotItem.dropEvent = lambda e : self.dropEvent(e)
        
        self.plotItem.addLegend()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        
        self.timerDt = 500 # Time between updates, in ms
        
        self.timer.start(self.timerDt) 
        
        ax_item:pg.AxisItem = self.plotItem.getAxis('bottom')
        ax_item.setLabel(text='Time since reception',units='s')
        
        self.__line_count = 0
                
        # Map : sender_id -> class_id -> message_id -> field_name -> [FieldPlotInfo | array_index -> FieldPlotInfo]
        self.plotItemMap:dict[int,dict[int,dict[int,dict[str,typing.Union[FieldPlotInfo,dict[int,FieldPlotInfo]]]]]] = dict()
    
    def getPlotItem(self,index:FieldIndex):
        a = self.plotItemMap[index.sender_id][index.class_id][index.message_id][index.field] 
        if index.array_index is None:
            return a
        else:
            return a[index.array_index]         
    
    
    def pauseUpdates(self,b:bool):
        if b:
            self.timer.stop()
        else:
            self.timer.start()

    @pyqtSlot()
    def update(self):
        now = time.time_ns()
        
        for s,sd in self.plotItemMap.items(): # Senders
            for c,cd in sd.items(): # Classes
                for m,md in cd.items(): # Messages
                    mIndex = MessageIndex(s,c,m)
                    
                    try:
                        msgLog = self.ivyRecorder.getMessage(mIndex)
                    except KeyError:
                        continue
                    
                    times = []
                    
                    data:dict[str,list] = dict()
                    for f in md.keys(): # Fields
                        data[f] = []
                    
                    for mm in msgLog.queue:
                        times.append((mm.timestamp-now)/10**9)
                        for f,l in data.items():
                            l.append(mm[f])
                    
                    for f,p in md.items():
                        if isinstance(p,dict):
                            for a_id,pp in p.items():
                                dd = np.asarray(data[f]).T[a_id]
                                pp.updatePlot(times,dd)
                        else:
                            p.updatePlot(times,np.asarray(data[f]))
                            
    @pyqtSlot(FieldPlotInfo)
    def removePlotItem(self,p:FieldPlotInfo):
        index = p.index
        try:
            a_dict = self.plotItemMap[index.sender_id][index.class_id][index.message_id]
            a = self.plotItemMap[index.sender_id][index.class_id][index.message_id][index.field]
            if index.array_index is None:
                del a_dict[index.field]
            else:
                del a[index.array_index]
                if len(a) == 0:
                    del a_dict[index.field]
            
            #TODO : Find a proper way to unsubscribe, in the case there are multiple plotters sharing the same logger
            if len(a_dict) == 0:
                del self.plotItemMap[index.sender_id][index.class_id][index.message_id]
        except KeyError:
            pass
        
        for hc in p.plotItem.highlightCurves():
            self.removeItem(hc)
            hc.deleteLater()
            
        self.removeItem(p.plotItem)
        p.plotItem.deleteLater()
        
         
                        
    ########## MIME aspects (Drag N Drop)  ##########
    
    def dragEnterEvent(self,e:QDragEnterEvent):
        mimedata = e.mimeData()
        
        if (mimedata.hasText()):
            txt = mimedata.text()
            
            if ':' in txt:
                e.accept()

    
    def dropEvent(self,e:QDropEvent):
        mimedata = e.mimeData()
                
        # Expected format for field description:
        # "{sender}:{class_name}:{msg_name}:{field_name}:{field_scale}"
        # OR, if the field is of array type:
        # "{sender}:{class_name}:{msg_name}:{field_name}[{array_range}]:{field_scale}"
        txt = mimedata.text()
        
        pltInfo_lst,self.__line_count = FieldPlotInfo.from_MIMEtxt(txt,self.__line_count)
        
        for p in pltInfo_lst:
            p:FieldPlotInfo
            try:
                pp = self.getPlotItem(p.index)
                continue
            except KeyError:
                try:
                    s_dict = self.plotItemMap[p.index.sender_id]
                except KeyError:
                    self.plotItemMap[p.index.sender_id] = dict()
                    s_dict = self.plotItemMap[p.index.sender_id]
                
                try:
                    c_dict = s_dict[p.index.class_id]
                except KeyError:
                    s_dict[p.index.class_id] = dict()
                    c_dict = s_dict[p.index.class_id]
                    
                try:
                    m_dict = c_dict[p.index.message_id]
                except KeyError:
                    c_dict[p.index.message_id] = dict()
                    m_dict = c_dict[p.index.message_id]
                    
                if p.index.array_index is not None:
                    try:
                        a_dict = m_dict[p.index.field]
                    except KeyError:
                        m_dict[p.index.field] = dict()
                        a_dict = m_dict[p.index.field]
                    a_dict[p.index.array_index] = p
                else:
                    m_dict[p.index.field] = p
                
                
                
                self.addItem(p.plotItem)
                
                print(f"Added {p.getMIMEtxt()}")
                
                for c in p.plotItem.highlightCurves():
                    self.addItem(c)
                
                p.deleteMe.connect(self.removePlotItem)

            self.ivyRecorder.recordMessage(p.index.sender_id,p.index.pprzMsg())
                
        self.update()
        
        e.acceptProposedAction()
        