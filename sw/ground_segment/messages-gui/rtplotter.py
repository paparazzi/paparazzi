#!/usr/bin/env python3

import numpy as np



from PyQt5.QtWidgets import QSplitter,QMainWindow,QMdiArea,QMdiSubWindow,QApplication,QWidget,\
                            QTreeView
from PyQt5.QtCore import Qt

from msgRecord.ivyRecorder import IvyRecorder

from plotting.plotWidget import PlotWidget

class RTPlotterMain(QSplitter):
    def __init__(self, ivy:IvyRecorder,parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setOrientation(Qt.Orientation.Horizontal)
        
        self.ivyRecorder = ivy
        
        # self.model = IvyModel(ivy)
        # self.filteredModel = FilteredIvyModel(self.model)
        
        self.addWidget(QTreeView(self))
        
        self.setStyleSheet("""
QSplitter::handle {
    background-color: #575757;
    border: 1px solid #777;
    width: 13px;
    margin-left: 2px;
    margin-right: 2px;
    border-radius: 4px;
}
        """)
        
        mdi = QMdiArea(self)
        mdi.setViewMode(QMdiArea.ViewMode.TabbedView)
        
        self.addWidget(mdi)

        



if __name__ == '__main__':
    app = QApplication([])
    app.setApplicationName("RT Plotter")
    
    ivy = IvyRecorder(buffer_size=200)
    window = QMainWindow()
    # window.setAcceptDrops(True)
    window.setCentralWidget(PlotWidget(ivy,window))
    window.setWindowTitle("Paparazzi RT Plotter")

    # window = PlotWidget(ivy)
    
    app.aboutToQuit.connect(ivy.stop)
    
    window.show()
    app.exec()