#!/usr/bin/env python3

import typing

from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import QWidget,QMainWindow,QApplication,\
                            QLabel
                           
# from generated.betterHSlider_ui import Ui_Form
                         
class BetterAbstractSlider():
        
    
    ## Setters
    
    def setText(self,txt:str):
        self.label_variable.setText(txt)
        
    def setDecimals(self,d:int):
        d = int(d)
        assert(d > 0)
        self.doubleSpinBox.setDecimals(d)
        self.doubleSpinBox.setSingleStep(10**(-d))
    
    def setRange(self,min:float,max:float):
        # Round using the precision
        min = round(min,self.decimals())
        max = round(max,self.decimals())
        
        intmin = int(min*10**self.decimals())
        intmax = int(max*10**self.decimals())
        
        self.label_min.setText(str(min))
        self.label_max.setText(str(max))
        self.slider.setRange(intmin,intmax)
    
    def setValue(self,v:float):
        self.doubleSpinBox.setValue(round(v,self.decimals()))
        self.slider.setValue(int(v*10**self.decimals()))
    
    ## Getters
    
    def decimals(self) -> int:
        return self.doubleSpinBox.decimals()
    
    def value(self) -> float:
        return self.doubleSpinBox.value()
    
    def minimum(self) -> float:
        return self.doubleSpinBox.minimum()
    
    def maximum(self) -> float:
        return self.doubleSpinBox.maximum()
    
    ## Connections
    
    
    def setupSignals(self):
        self.doubleSpinBox.valueChanged.connect(self.__spinbox_to_slider_value)
        self.slider.valueChanged.connect(self.__slider_to_spinbox_value)
        
        self.slider.rangeChanged.connect(self.__slider_range_changed)
        
        self.setText("Label")
        self.setDecimals(2)
        self.setRange(0,2)
        
        self.valueChanged = self.doubleSpinBox.valueChanged
        
    def __spinbox_to_slider_value(self,value:float):
        self.slider.setValue(int(value*10**self.decimals()))
        
    def __slider_to_spinbox_value(self,value:int):
        self.doubleSpinBox.setValue(value/10**self.decimals())
        
    def __slider_range_changed(self,intmin:int,intmax:int):
        min = round(intmin/10**self.decimals(),self.decimals())
        max = round(intmax/10**self.decimals(),self.decimals())
        self.doubleSpinBox.setRange(min,max)
        
        
    