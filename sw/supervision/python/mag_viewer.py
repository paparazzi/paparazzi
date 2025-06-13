import sys
import numpy as np
from PyQt5 import QtWidgets, QtCore
import utils
from PyQt5.QtWidgets import QSizePolicy as QSP

try:
    import pyqtgraph as pg
    import pyqtgraph.opengl as gl

    #raise ModuleNotFoundError()

    sys.path.append(utils.PAPARAZZI_SRC + "/sw/lib/python")
    sys.path.append(utils.PAPARAZZI_HOME + "/var/lib/python") # pprzlink

    from pprzlink.message import PprzMessage
    from pprz_connect import PprzConnect, PprzConfig

    class MagViewer(QtWidgets.QWidget):

        mag_sig = QtCore.pyqtSignal(tuple)

        def __init__(self, parent=None):
            super().__init__(parent)

            self.mag_data = np.empty((0,3))

            self.vlay = QtWidgets.QVBoxLayout(self)

            self.cmds_lay = QtWidgets.QHBoxLayout()
            self.vlay.addLayout(self.cmds_lay)
            
            self.setup_cmds()

            self.gl_widget = gl.GLViewWidget()
            self.vlay.addWidget(self.gl_widget)
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
            self.mag_sig.connect(self.update_mag)

            self.raw_bind_id = None
            self.confs:dict[int, PprzConfig] = {}
        
        def setup_cmds(self):
            mag_lbl = QtWidgets.QLabel("mags:", self)
            mag_lbl.setSizePolicy(QSP.Policy.Fixed, QSP.Policy.Fixed)
            self.cmds_lay.addWidget(mag_lbl)

            self.mag_combo = QtWidgets.QComboBox(self)
            self.cmds_lay.addWidget(self.mag_combo)
            self.mag_combo.addItem("---")

            self.ivy_sub_button = QtWidgets.QPushButton("Subscribe", self)
            self.cmds_lay.addWidget(self.ivy_sub_button)
            self.ivy_sub_button.clicked.connect(self.sub_unsub)

            clear_button = QtWidgets.QPushButton("Clear data")
            clear_button.setSizePolicy(QSP.Policy.Fixed, QSP.Policy.Fixed)
            self.cmds_lay.addWidget(clear_button)
            clear_button.clicked.connect(self.reset_data)
        
        def stop(self):
            self.connect.shutdown()
        
        def sub_unsub(self):
            if self.raw_bind_id is None:
                self.raw_bind_id = self.connect.ivy.subscribe(self.mag_raw_cb, PprzMessage("telemetry", "IMU_MAG_RAW"))
                self.ivy_sub_button.setText("Unsubscribe")
            else:
                self.connect.ivy.unsubscribe(self.raw_bind_id)
                self.ivy_sub_button.setText("Subscribe")
                self.raw_bind_id = None

        def reset_data(self):
            self.mag_data = np.empty((0,3))
            self.set_points(self.mag_data)
        
        def connect_cb(self, conf: PprzConfig):
            self.confs[conf.id] = conf

        def mag_raw_cb(self, sender, msg):
            ac_id = str(sender)
            if ac_id not in self.confs:
                return
            ac_name = self.confs[ac_id].name
            mag_id = f"{ac_name}: {msg['id']}"

            mag_idx = self.mag_combo.findText(mag_id)
            cur_idx = self.mag_combo.currentIndex()

            if cur_idx == mag_idx:
                mx = msg['mx']
                my = msg['my']
                mz = msg['mz']
                mag = np.array([mx, my, mz])
                self.mag_sig.emit((mag_id, mag))
            elif mag_idx == -1:
                self.mag_combo.addItem(mag_id)
        
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
except ImportError:
    
    class MagViewer(QtWidgets.QWidget):
        def __init__(self, parent=None):
            super().__init__(parent)
            self.vlay = QtWidgets.QVBoxLayout(self)
            self.error_label = QtWidgets.QLabel("module not found!")
            self.vlay.addWidget(self.error_label)
        
        def stop(self):
            pass
