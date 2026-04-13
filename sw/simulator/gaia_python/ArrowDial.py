#!/usr/bin/env python3

import typing,math

from PyQt5.QtCore import Qt,QPoint,QPointF
from PyQt5.QtWidgets import QWidget,QDial,QMainWindow,QApplication
from PyQt5.QtGui import QColor,QPainter,QPaintEvent,QBrush,QPen

# Custom Arrow dial based on:
# http://thecodeinn.blogspot.com/2015/02/customizing-qdials-in-qt-part-1.html
# http://thecodeinn.blogspot.com/2015/03/customizing-qdials-in-qt-part-2.html

class ArrowDial(QDial):
    
    # Points defining the arrow, in a 200x200 square, pointing Downward, with coordinates:
    #
    #  (-100,-100) - - - - - (   0,-100) - - - - - ( 100,-100)
    #       |                      _                    |
    #       |                     | |                   |
    #       |                     | |                   |
    #       |                     | |                   |
    #       |                     | |                   |
    #  (-100,   0)           (    0,   0)          ( 100,   0)
    #       |                     | |                   |
    #       |                     | |                   |
    #       |                    _| |_                  |
    #       |                    \   /                  |
    #       |                     \_/                   |
    #  (-100, 100) - - - - - (   0, 100) - - - - - ( 100, 100)
    
    ARROW_POINTS = [
        QPoint(0,80),  # Head point
        QPoint(6,60),  # Right triangle
        QPoint(2,60),  # Head-line right  
        QPoint(2,-70), # End right
        QPoint(-2,-70),# End left
        QPoint(-2,60), # Head-line left
        QPoint(-6,60), # Left triangle
    ]
    
    # Use a 4-point arrow(head) instead:
    #   |V|
    #   | |
    #    v
    
    ARROW_4POINTS = [
        QPoint(0,80),   # Tip
        QPoint(8,-80),  # Right base
        QPoint(0,-75),  # Chevron
        QPoint(-8,-80), # Left base
    ]
    
    def __init__(self, parent:QWidget = None):
        super().__init__(parent)
    
        self._reverseArrow = False
    
    ##### Setters and Getters #####
    
    @property
    def reverseArrow(self) -> bool:
        return self._reverseArrow

    @reverseArrow.setter
    def reverseArrow(self,val:bool):
        self._reverseArrow = val
    
    ##### Painting #####
    
    def resizeEvent(self, re):
        pass
    
    def paintEvent(self, pe:typing.Optional[QPaintEvent]):
        
        painter = QPainter(self)
        
        # painter.setBrush(QColor(0,0,0,255))
        # painter.drawRect(0,0,self.width(),self.height())
        
        ## Referential transform
        # Set origin in the middle 
        painter.translate(self.width() / 2, self.height() / 2)

        # Rescale to a Square 200x200
        side = min(self.width(), self.height())
        painter.scale(side / 200.0, side / 200.0)
        
        ## Color and design
        # Colors 
        edgeColor = QColor(255,255,255,255)
        fillColor = QColor(0,0,0,255)
        
        pen = painter.pen()
        pen.setWidth(1)
        pen.setColor(edgeColor)
        painter.setPen(pen)
        painter.setBrush(fillColor)
        
        # Antialiasing
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        ## Paint arrow
        painter.save()
        
        val = self.value()
        minimum = self.minimum()
        maximum = self.maximum()
        percent = (val - minimum)/(maximum-minimum)
        angle = 360*percent
        
        if self._reverseArrow:
            painter.rotate(angle+180)
        else:
            painter.rotate(angle)
            
        painter.drawPolygon(self.ARROW_4POINTS,Qt.FillRule.OddEvenFill)
        
        painter.restore()
        
        ## Paint ticks
        
        if self.notchesVisible():
            # Majors (0°,90°,180°,270°) aka (N,E,S,W)
            painter.save()
            
            for _ in range(4):
                painter.drawRect(-2,85,4,13)
                painter.rotate(90)
                
            painter.restore()
            
            # Halves (45°,135°, 225°,315°) aka (NE,SE,SW,NW)

            painter.save()
            
            painter.rotate(45)
            
            for _ in range(4):
                painter.drawRect(-1,90,2,8)
                painter.rotate(90)
                
            painter.restore()
            
            # Minors (10°s which are not Majors)
            
            painter.save()
            
            for i in range(0,360,10):
                if i % 90 != 0:
                    painter.drawRect(-1,94,2,4)
                painter.rotate(10)
                
            painter.restore()
                
        ## Done
        painter.end()
      
        
        
        
if __name__ == '__main__':
    app = QApplication([])
    window = QMainWindow()
    window.setCentralWidget(ArrowDial(window))
    window.show()
    app.exec()