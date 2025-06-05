#!/usr/bin/env python3
import sys
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import random
from PyQt5.QtCore import QTimer

import pyqtgraph.opengl as gl
import trimesh
from os import getenv, path
from math import radians, degrees

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.message import PprzMessage
from pprz_connect import PprzConnect, PprzConfig
from scipy.spatial.transform import Rotation as R

NED_to_ENU = R.from_matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

MESH_ROT = R.identity()

class DroneTrajectoryVisualizer(QtWidgets.QWidget):

    pos_sig = QtCore.pyqtSignal(np.ndarray)
    att_sig = QtCore.pyqtSignal(R)

    def __init__(self, mesh_file, scale=1):
        super().__init__()
        self.drone_attitude = R.identity()
        self.trajectory = np.empty((0, 3))

        self.setup(mesh_file, scale)
        
        self.connect = PprzConnect(notify=self.connect_cb)
        self.connect.ivy.subscribe(self.nps_pos_cb, PprzMessage("telemetry", "NPS_SPEED_POS"))
        self.connect.ivy.subscribe(self.nps_att_cb, PprzMessage("telemetry", "NPS_RATE_ATTITUDE"))
        self.pos_sig.connect(self.update_drone_position)
        self.att_sig.connect(self.update_drone_attitude)
    
    def connect_cb(self, conf):
        #print(conf)
        pass
    
    def nps_pos_cb(self, _sender, msg):
        x = msg['ltpp_x']
        y = msg['ltpp_y']
        z = msg['ltpp_z']
        pos_enu = NED_to_ENU.apply([x, y, z])
        self.pos_sig.emit(pos_enu)

    def nps_att_cb(self, _sender, msg):
        phi = msg['phi']
        theta = msg['theta']
        psi = msg['psi']
        r = R.from_euler('zyx', [psi, theta, phi], degrees=True)
        r_enu = NED_to_ENU * r
        self.att_sig.emit(r_enu)

    def update_drone_position(self, pos):
        """
        Callback to update the drone's position.
        :param pos: [x, y, z] np.ndarray
        """
        pos = np.array(pos).reshape(1, 3)
        self.trajectory = np.vstack([self.trajectory, pos])
        self.line.setData(pos=self.trajectory)
    
    def update_drone_attitude(self, attitude: R):
        """
        Callback to update the drone's attitude.
        """
        self.drone_attitude = attitude
        self.drone_marker.resetTransform()
        mesh_att = MESH_ROT * self.drone_attitude
        axis = mesh_att.as_rotvec(degrees=True)
        angle = np.linalg.norm(axis)
        #axis = axis / np.linalg.norm(axis)
        self.drone_marker.rotate(angle, *axis)
        if len(self.trajectory) > 0:
            self.drone_marker.translate(*self.trajectory[-1])
    
      
    def setup(self, mesh_file, scale):
        self.setWindowTitle('3D Drone Trajectory')
        self.layout = QtWidgets.QVBoxLayout()
        self.setLayout(self.layout)
        self.gl_widget = gl.GLViewWidget(rotationMethod='quaternion')
        self.layout.addWidget(self.gl_widget)
        self.gl_widget.setCameraPosition(distance=20)

        # Axis
        axis = gl.GLAxisItem()
        axis.setSize(10, 10, 10)
        self.gl_widget.addItem(axis)
        # Axis Texts
        txt_x = gl.GLTextItem(text='X (east)', pos=[10, 0, 0])
        self.gl_widget.addItem(txt_x)
        txt_y = gl.GLTextItem(text='Y (north)', pos=[0, 10, 0])
        self.gl_widget.addItem(txt_y)
        txt_z = gl.GLTextItem(text='Z (up)', pos=[0, 0, 10])
        self.gl_widget.addItem(txt_z)
        # XY grid
        xy_grid = gl.GLGridItem()
        xy_grid.setSize(100, 100)
        xy_grid.setSpacing(10, 10)
        xy_grid.rotate(0, 1, 0, 0)
        self.gl_widget.addItem(xy_grid)

        self.gl_widget.setBackgroundColor((20, 20, 20, 255))

        # Trajectory line
        self.line = gl.GLLinePlotItem(pos=self.trajectory, color=(1, 0, 0, 1), width=2, antialias=True)
        self.gl_widget.addItem(self.line)

        # drone model
        mesh = trimesh.load(mesh_file)
        mesh_data = gl.MeshData(vertexes=mesh.vertices, faces=mesh.faces)
        # Scale the vertices directly
        vertices = mesh_data.vertexes() * scale
        mesh_data.setVertexes(vertices)
        self.drone_marker = gl.GLMeshItem(meshdata=mesh_data, color=(0, 1, 0, 1), smooth=True, shader='shaded', drawEdges=False)

        self.drone_marker.setGLOptions('opaque')
        self.gl_widget.addItem(self.drone_marker)

        # Resize the window
        self.resize(800, 600)

    def stop(self):
        self.connect.shutdown()

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    vis = DroneTrajectoryVisualizer('avion.stl', 5)
    vis.show()

    app.aboutToQuit.connect(vis.stop)

    timer = QTimer()
    timer.start(100)  # update every 100 ms

    sys.exit(app.exec_())
