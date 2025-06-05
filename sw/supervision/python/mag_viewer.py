import sys
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import pyqtgraph.opengl as gl

import utils

sys.path.append(utils.PAPARAZZI_SRC + "/sw/lib/python")
sys.path.append(utils.PAPARAZZI_HOME + "/var/lib/python") # pprzlink

from pprzlink.message import PprzMessage
from pprz_connect import PprzConnect, PprzConfig

class MagViewer(QtWidgets.QWidget):

    mag_sig = QtCore.pyqtSignal(tuple)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.mag_data = np.empty((0,3))

        self.layout = QtWidgets.QVBoxLayout(self)
        self.clear_button = QtWidgets.QPushButton("Clear data")
        self.layout.addWidget(self.clear_button)
        self.clear_button.clicked.connect(self.reset_data)
        self.gl_widget = gl.GLViewWidget()
        self.layout.addWidget(self.gl_widget)
        self.gl_widget.setCameraPosition(distance=20)

        # Example axis
        axis = gl.GLAxisItem()
        axis.setSize(2, 2, 2)
        self.gl_widget.addItem(axis)
        # Axis Texts
        txt_x = gl.GLTextItem(text='X', pos=[2, 0, 0])
        self.gl_widget.addItem(txt_x)
        txt_y = gl.GLTextItem(text='Y', pos=[0, 2, 0])
        self.gl_widget.addItem(txt_y)
        txt_z = gl.GLTextItem(text='Z', pos=[0, 0, 2])
        self.gl_widget.addItem(txt_z)
        # XY grid
        xy_grid = gl.GLGridItem()
        xy_grid.setSize(2, 2)
        xy_grid.setSpacing(0.2, 0.2)
        xy_grid.rotate(0, 1, 0, 0)
        self.gl_widget.addItem(xy_grid)

        shaft = np.array([[0,0,0], [1,0,0]])
        self.arrow = gl.GLLinePlotItem(pos=shaft, width=2)
        self.gl_widget.addItem(self.arrow)

        # Placeholder for scatter plot
        self.scatter = None
        self.setMinimumSize(100, 100)
        #self.gl_widget.setBackgroundColor((20, 20, 20, 255))

        self.connect = PprzConnect(notify=self.connect_cb)
        self.connect.ivy.subscribe(self.mag_raw_cb, PprzMessage("telemetry", "IMU_MAG_RAW"))
        self.mag_sig.connect(self.update_mag)
    
    def stop(self):
        self.connect.shutdown()

    def reset_data(self):
        self.mag_data = np.empty((0,3))
        self.set_points(self.mag_data)
    
    def connect_cb(self, conf):
        pass

    def mag_raw_cb(self, sender, msg):
        mid = msg['id']
        mx = msg['mx']
        my = msg['my']
        mz = msg['mz']
        mag = np.array([mx, my, mz])
        self.mag_sig.emit((mid, mag))
    
    def update_mag(self, mag_data):
        mid, mag = mag_data
        mag_unit = mag / np.linalg.norm(mag)
        self.mag_data = np.vstack([self.mag_data, mag_unit])
        self.set_points(self.mag_data)
        shaft = np.array([[0,0,0], mag_unit])
        self.arrow.setData(pos=shaft)

    def set_points(self, points, color=(1, 0, 0, 1), size=5):
        """
        points: Nx3 numpy array
        color: tuple (r, g, b, a)
        size: point size
        """
        if self.scatter:
            self.gl_widget.removeItem(self.scatter)
        self.scatter = gl.GLScatterPlotItem(pos=points, color=color, size=size)
        self.gl_widget.addItem(self.scatter)
